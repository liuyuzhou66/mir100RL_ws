#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import math
import time
import rospy 
import threading
from pathlib import Path
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from std_msgs.msg import Header
from dataclasses import dataclass
from scipy.spatial.distance import cdist
from gazebo_msgs.srv import GetModelState, SetModelState, SpawnModel, DeleteModel
from gazebo_msgs.msg import ModelState 
from geometry_msgs.msg import Pose, PoseStamped, Quaternion, Point, PoseWithCovarianceStamped
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from agents.q_agent import QAgent
from std_srvs.srv import Empty, EmptyRequest
# Brings in the SimpleActionClient
import actionlib
import move_base_msgs.msg
import geometry_msgs.msg

BASE_PATH = Path(os.path.dirname(__file__))
results_path = BASE_PATH.parent.parent.parent / 'Results_Plot'
if not results_path.exists():
    results_path.mkdir()


WAYPOINT_POSITIONS = [-13.25,10.0,0, -5.0,10.0,0, 9.3,12.3,0, 12.0,-12.0,0, -4.0,-10.5,0, -15.3,-10.0,0]

WAYPOINT_YAWS = [90, 20, 90, -90, -90, 0]

SPEED = 1

subscriber = None

def dist(x1, y1, x2, y2):
    return cdist(
            [[x1, y1]],
            [[x2, y2]])[0][0]


@dataclass
class Waypoint:
    position: Point
    yaw: int
    # By considering a long-term decision making strategy
    ## We initialize the Q matrix with the time(distance/speed) matrix between (waypoints) and (waypoints + starting point) 
    def duration(self, wp):
        # time it takes to move from one waypoint to another
        # should be replaced with a table later on
        return dist(
            self.position.x, self.position.y, 
            wp.position.x, wp.position.y) / SPEED

# Robot starts at 0-position (the index of starting point is 0)
START_POINT = Waypoint(Point(-19.0, 0, 0), 0)
# START_POINT = Waypoint(Point(34.0, -8.0, 0), 0)

WP_Label = '''<?xml version='1.0'?>
<sdf version='1.6'>
  <model name='NAME'>
    <pose>0 0 2.0 0 0 0</pose>
    <link name='NAME_link'>
      <gravity>0</gravity>
      <self_collide>0</self_collide>
      <kinematic>0</kinematic>
      <enable_wind>0</enable_wind>
      <visual name='NAME_visual'>
        <geometry>
          <sphere>
            <radius>0.5</radius>
          </sphere>
        </geometry>
        <material>
          <lighting>1</lighting>
          <script>
            <uri>file://media/materials/scripts/gazebo.material</uri>
            <name>Gazebo/COLOR</name>
          </script>
          <shader type='pixel'>
            <normal_map>__default__</normal_map>
          </shader>
        </material>
        <transparency>0</transparency>
        <cast_shadows>1</cast_shadows>
      </visual>
    </link>
    <static>1</static>
    <allow_auto_disable>1</allow_auto_disable>
  </model>
</sdf>
'''

class Waypoint_Label:
    def __init__(self, model_name):
        self.model_name = model_name
        self.spawned = False
        self.current_color = ''
        #self.rate = rospy.Rate(3)

    def delete(self):
        spawn_model_prox = rospy.ServiceProxy(
            'gazebo/delete_model', DeleteModel)
        rospy.wait_for_service('gazebo/delete_model', 4.0)
        rospy.sleep(.2)
        res = spawn_model_prox(self.model_name)
        rospy.loginfo(f'Deleted {self.model_name}, {res}')
        self.spawned = False

    def _spawn(self, x, y, color):
        if self.current_color == color:
            return
        if self.spawned:
            self.delete()
        rospy.sleep(.3)
        rospy.wait_for_service('gazebo/spawn_sdf_model', 4.0)
        spawn_model_prox = rospy.ServiceProxy(
            'gazebo/spawn_sdf_model', SpawnModel)
        spawn_pose = Pose()
        spawn_pose.position.x = x
        spawn_pose.position.y = y
        spawn_pose.position.z = 2.5 
        model_xml = WP_Label.replace('NAME', self.model_name)
        model_xml = model_xml.replace('COLOR', color)
        rospy.sleep(.2)
        
        res = spawn_model_prox(
            self.model_name, model_xml, '', spawn_pose, 'world')
        rospy.loginfo(f'Spawned {self.model_name} ({color}): {res}')
        self.current_color = color
        self.spawned = True

    def spawn_blue(self, x, y):
        self._spawn(x, y, 'SkyBlue')
    
    def spawn_green(self, x, y):
        self._spawn(x, y, 'Green')
    
    def spawn_red(self, x, y):
        self._spawn(x, y, 'Red')

class DynamicObstacleNavigationMir100Test:
    def __init__(self, **kwargs):
        self._name = 'mir100_rl_test'
        rospy.loginfo(f'[{self._name}] starting node <DynamicObstacleNavigationMir100Sim>')
        self.waypoints = []
        self.waypoints += [
            Waypoint(Point(*WAYPOINT_POSITIONS[i*3:(i*3)+3]),
            WAYPOINT_YAWS[i]
            )
            for i in range(len(WAYPOINT_YAWS))]

        # Create a dictionary to store the spawn status of waypoint labels
        self.waypoint_labels = {}
        for j in range(len(WAYPOINT_YAWS)):
            wp_name = f"Waypoint_{j}"
            sphere = Waypoint_Label(wp_name)
            self.waypoint_labels[wp_name] = sphere

        # Set the robot initial position to starting point(0, -38.5, 0)
        self.all_points = [START_POINT]
        self.all_points += self.waypoints
        # self.all_points.append(START_POINT)

        self.StartPoint_x = START_POINT.position.x
        self.StartPoint_y = START_POINT.position.y
        self.StartPoint_z = START_POINT.position.z
        self.StartPoint_ori = quaternion_from_euler(0, 0, np.deg2rad(START_POINT.yaw))

        # Number of all_points, including starting point
        self.num_all_points = len(self.all_points)

        # Number of waypoints
        self.num_waypoints = len(self.waypoints)

        # Initialize the action_space and state_space
        self.state_space = self.num_all_points  # state_space includes all points
        self.action_space = self.num_waypoints  # action_space includes waypoints only  

        # Initialize
        self.visited_points = []    # The index of visited points, the index of starting point is "0"
        self.waypoints_time = []     # Time since waypoints have been reached
        self.overall_time = None    # Time since episode starts

    def reset(self):
        rospy.loginfo(f"[{self._name}] reset!")

        # Reset the waypoint status (1 is reached, 0 is not reached)
        self.waypoints_status = [0] * self.num_waypoints

        # teleport robot to the starting point
        rospy.sleep(1.0)
        self.teleport(self.StartPoint_x, self.StartPoint_y)

        # Get the time robot start path planning
        time = rospy.get_rostime()
        self.time_start = time.to_sec()
        rospy.loginfo(f"[{self._name}] The time to reset time_start is [{self.time_start}]")

        # Spawn blue sphere to waypoint by default
        for j in range(len(WAYPOINT_YAWS)):
            if self.waypoints_status[j] == 0:
                wp_name = f"Waypoint_{j}"
                sphere = self.waypoint_labels[wp_name]
                sphere.spawn_blue(self.waypoints[j].position.x, self.waypoints[j].position.y)


    def move_to_waypoint(self, wp_idx):
        done = False

        # Sets the color of the point the robot is currently heading to to red
        for j in range(len(WAYPOINT_YAWS)):
            if j == wp_idx:
                wp_name = f"Waypoint_{j}"
                sphere = self.waypoint_labels[wp_name]
                sphere.spawn_red(self.waypoints[j].position.x, self.waypoints[j].position.y)

        # Move robot to next_waypoint
        wp = self.waypoints[wp_idx]
        wp_x = wp.position.x
        wp_y = wp.position.y
        wp_ori = wp.yaw
        self.move_base_action_client(wp_x, wp_y, wp_ori)

        # If the robot reaches the waypoint, then get current robot position
        self._get_robot_position()

        # Update the waypoint status
        _dist = dist(
            self.mir100_pos_x, self.mir100_pos_y, 
            self.waypoints[wp_idx].position.x, self.waypoints[wp_idx].position.y)
        if _dist < 0.5: # distance of the waypoint to the robot < 0.5 m
            self.waypoints_status[wp_idx] = 1
        rospy.loginfo(f"[{self._name}] updates waypoints_status, done!")
        rospy.loginfo(f"[{self._name}]'s current waypoints_status: {self.waypoints_status}")

        # Update the oveall time since agent left START_POINT
        time = rospy.get_rostime()
        tim_s = time.to_sec()
        o_time = tim_s - self.time_start
        self.overall_time = round(o_time, 2)
        rospy.loginfo(f"[{self._name}] updates overall_time, done!")
        rospy.loginfo(f"[{self._name}] overall_time taken: {self.overall_time}")

        # End when all waypoints have been visited
        if np.sum(self.waypoints_status) == self.num_waypoints:
            done = True
            rospy.loginfo("All waypoints have been reached!")
            rospy.loginfo(f"[{self._name}] overall_time taken: {self.overall_time}")

        # Update the color of waypoints that have been visited by the robot to green
        for j in range(len(WAYPOINT_YAWS)):
            if self.waypoints_status[j] == 1:
                wp_name = f"Waypoint_{j}"
                sphere = self.waypoint_labels[wp_name]
                sphere.spawn_green(self.waypoints[j].position.x, self.waypoints[j].position.y)

        return done


    def move_base_action_client(self, goalx=0,goaly=0,goaltheta=0):
        rospy.loginfo(f"[{self._name}] starts moving to the next waypoint!")
        goal_pose = Pose()
        goal_pose.position.x = goalx
        goal_pose.position.y = goaly
        q_angle = quaternion_from_euler(0.0, 0.0, np.deg2rad(goaltheta))
        goal_pose.orientation = geometry_msgs.msg.Quaternion(*q_angle)

        thread1 = threading.Thread(
            target=self.move_base_thread,
            args=(goal_pose,))
        thread2 = threading.Thread(
            target=self.move_base_watcher_thread,
            args=(goal_pose,))

        thread1.start()
        thread2.start()

        thread1.join()
        thread2.join()
        return


    def move_base_thread(self, goal_pose):
        client=actionlib.SimpleActionClient(
            'move_base',
            move_base_msgs.msg.MoveBaseAction)
        self.action_client = client
        client.wait_for_server(rospy.Duration(20))
        goal=move_base_msgs.msg.MoveBaseGoal()
        goal.target_pose.header.frame_id='map'
        goal.target_pose.header.stamp=rospy.Time.now()
        goal.target_pose.pose = goal_pose
        rospy.loginfo(
            f"[{self._name}] is sending next wayppoint position ("
            f"{goal_pose.position.x}, {goal_pose.position.y})")
        client.send_goal(goal)
        client.wait_for_result(rospy.Duration(300))
        res = client.get_result()
        rospy.loginfo(f"RESULT: {res}")


    def move_base_watcher_thread(self, goal_pose):
        while True:
            rospy.sleep(1)
            x, y, yaw = self._get_robot_position()
            _dist = dist(x, y, goal_pose.position.x, goal_pose.position.y)
            if _dist < 0.5:
                rospy.loginfo(f'Close to waypoint, [{self._name}] is considered to have reached the waypoint, distance to waypoint: {_dist}')
                if hasattr(self, 'action_client'):
                    self.action_client.cancel_all_goals()
                return


    def teleport(self, x, y):

        rospy.loginfo(f"Teleport [{self._name}] robot to the starting point!")

        state_msg = ModelState()
        state_msg.model_name = "mir"
        state_msg.pose.position.x = x
        state_msg.pose.position.y = y

        rospy.wait_for_service('/gazebo/set_model_state')
        set_state = rospy.ServiceProxy(
            '/gazebo/set_model_state', SetModelState)
        resp = set_state(state_msg )
        rospy.loginfo(f'[{self._name}] is teleported to the starting point({x}, {y})!')

        pose_with_stamp_msg = PoseWithCovarianceStamped()
        pose_with_stamp_msg.pose.pose.position.x = x
        pose_with_stamp_msg.pose.pose.position.y = y
        #rospy.wait_for_service('/initialpose')
        pose_with_stamp = rospy.Publisher(
            '/initialpose', PoseWithCovarianceStamped, queue_size=10)
        resp = pose_with_stamp.publish(pose_with_stamp_msg )
    

    def _get_robot_position(self):
        rospy.wait_for_service('/gazebo/get_model_state')
        get_state = rospy.ServiceProxy(
            '/gazebo/get_model_state', GetModelState)
        resp = get_state(model_name = "mir")
        self.mir100_pos_x = resp.pose.position.x
        self.mir100_pos_y = resp.pose.position.y
        euler_angles = euler_from_quaternion([resp.pose.orientation.x, resp.pose.orientation.y, resp.pose.orientation.z, resp.pose.orientation.w])
        yaw = euler_angles[2]   # roll, pitch, <<yaw>> (in radian)
        self.mir100_pos_yaw = yaw
        # rospy.loginfo(f"[{self._name}] get robot current position: ({self.mir100_pos_x}, {self.mir100_pos_y})!")
        return self.mir100_pos_x, self.mir100_pos_y, self.mir100_pos_yaw


def run_episode(env,agent):
    
    rospy.loginfo("[mir100_rl_test] starts a new episode!")
    
    # The initial state of the robot is start_point_index
    s = 0
    agent.reset_memory()
    env.reset()
    # Max steps per episode (6 steps for 6 waypoints)
    max_step = env.num_waypoints
    
    i = 0
    while i < max_step:

        # Remember the wapoint robot just reached, except the starting point
        if s != 0:
            wp_index = s - 1
            agent.wp_arrived(wp_index)

        # Choose an action
        a = agent.act(s)
        rospy.loginfo(f"[mir100_rl_test] picks an action ({a})!")

        # Move to the waypoint
        done = env.move_to_waypoint(a)
        
        # Take the action, and get the reward from environment
        s_next = a
        rospy.loginfo(f"for action ({a}), [mir100_rl_test]'s new state: {s_next}; episode done: {done}!")
        
        # Update the caches
        s = s_next
        rospy.loginfo(f"[mir100_rl_test]'s step num.<{i}> is finished!")
        
        # If the episode is terminated
        i += 1
        if done:
            break
            
    return env,agent

class QAgent:
    def __init__(self,*args,**kwargs):
        super().__init__(*args,**kwargs)
        self.reset_memory()

        # Load Q table
        self.Q = np.load(results_path / 'RL_Qtable.npy')

        rospy.loginfo("----type----")
        rospy.loginfo(type(self.Q))
        rospy.loginfo("----shape----")
        rospy.loginfo(self.Q.shape)
        rospy.loginfo("----data----")
        rospy.loginfo(self.Q)

    def act(self,s):
        # Get Q Vector
        q = np.copy(self.Q[s,:])

        # Avoid already visited waypoints
        q[self.wp_memory] = -np.inf

        # Pick one waypoint index based on the Q table
        a = np.argmax(q)

        return a

    def wp_arrived(self,wp_index):
        self.wp_memory.append(wp_index)
    
    def reset_memory(self):
        self.wp_memory = []


def run_num_episode(env, agent, num_episodes = 10):
    overall_times = []

    results_path = BASE_PATH.parent.parent.parent / 'Results_Plot'
    if not results_path.exists():
        results_path.mkdir()

    for i in range(num_episodes):
        # Run the episode
        rospy.loginfo(f"[mir100_rl_test] episode <{i}>! (total number of episode: {num_episodes})")
        overall_times.append(run_episode(env, agent))

        shortest_t = min(overall_times)
        index_shortest_t = overall_times.index(shortest_t)
        episode_shortest_t = index_shortest_t + 1
        rospy.loginfo(f"The minimum time for the [mir100_rl_test] to complete the task is {shortest_t} seconds in episode <{episode_shortest_t}>!")

        # Calculate the average time
        avg_time = np.mean(overall_times)

        # Show overall_times
        plt.figure(figsize = (15,5))
        plt.title(f"Greedy Method: Overall Time Taken ({i} Episodes), Average time: {avg_time}",  fontsize=16, fontweight= 'bold', pad=10)
        plt.xlabel("Episode")
        plt.ylabel("Overall Time (Unit: second)")
        plt.plot(overall_times)
        plt.savefig(results_path / 'Greedy_OverallTime.png', dpi = 200)
        rospy.loginfo(f"[mir100_rl_test] successfully saves Greedy_OverallTime.png to {results_path}!")




if __name__ == u'__main__':
    rospy.init_node('mir100_rl_test', anonymous=True)

    rospy.loginfo('sleep for 2 second')
    time.sleep(2)
    rospy.loginfo('wait for service to unpause')
    rospy.wait_for_service('/gazebo/unpause_physics')
    rospy.loginfo('calling service to unpause')
    unpause_physics_client = rospy.ServiceProxy('/gazebo/unpause_physics', Empty)
    unpause_physics_client(EmptyRequest())
    rospy.loginfo('should be unpaused')
    
    env = DynamicObstacleNavigationMir100Test
    agent = QAgent

    num_episodes = 10
    run_num_episode(env, agent, num_episodes)

    pause_physics_client = rospy.ServiceProxy('/gazebo/pause_physics', Empty)
    pause_physics_client(EmptyRequest())