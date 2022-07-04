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
from tf.transformations import quaternion_from_euler
from agents.q_agent import QAgent
from std_srvs.srv import Empty, EmptyRequest
# Brings in the SimpleActionClient
import actionlib
import move_base_msgs.msg
import geometry_msgs.msg

import sys
sys.path.append("../")

BASE_PATH = Path(os.path.dirname(__file__))

WAYPOINT_POSITIONS = [-13.25,10.0,0, -5.0,10.0,0, 9.3,12.3,0, 12.0,-12.0,0, -4.0,-10.5,0, -15.3,-10.0,0]

WAYPOINT_YAWS = [90, 20, 90, -90, -90, 0]

SPEED = 1


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

"""
Q matrix:

                                           Action space
                             0       1       2       3       4       5    

                         |   A   |   B   |   C   |   D   |   E   |   F   |
                 ---------------------------------------------------------
              0      S   |       |       |       |       |       |       |
                 ---------------------------------------------------------
              1      A   |   -   |       |       |       |       |       |
                 ---------------------------------------------------------
              2      B   |       |   -   |       |       |       |       |
                 ---------------------------------------------------------
State space   3      C   |       |       |   -   |       |       |       |
                 ---------------------------------------------------------
              4      D   |       |       |       |   -   |       |       |
                 ---------------------------------------------------------
              5      E   |       |       |       |       |   -   |       |
                 ---------------------------------------------------------
              6      F   |       |       |       |       |       |   -   |

S: starting point

A B C D E F: waypoints

State: The waypoint that the robot has reached so far

Action: Next waypoint to go

Reward: 1-  agent gets a reward of -1 for each second 
        
        2-  If a waypoint is delivered in that timestep, 
            it gets a positive reward inversely proportional to the time taken to deliver.
        
        3-  If all the waypoints are delivered and the agent is back to the start point, 
            it gets an additional reward inversely proportional to time since start of episode.
"""


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



class DynamicObstacleNavigationMir100Sim:
    def __init__(self,**kwargs):
        self._name = 'mir100_rl_1'
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
        """
        self.at_startpoint = None   # Check whether robot is at the starting point (0: not at; 1: at)
        """
        self.overall_time = None    # Time since episode starts
        self.max_time = 400         # Maximum time for each episode: 0.5 h = 30 min = 1800 s
        self.initialize_Q_table_by_time = False

        # The Multiplier value for additional reward. This needs to be adjusted!
        #self.addi_reward_multiply_value = 10   # We are more concerned with reducing the overall time rather than the time it takes to get to a certain waypoint.

        # Initialize basic reward in Q-table based on the time taken between (the starting point + waypoints) and (waypoints) in an obstacle-free environment
        if self.initialize_Q_table_by_time:
            self._generate_q_values()

        self.restart_episode = False
        # Restart the environment on a new episode. Return the start_point_index
        # self.reset()


    def reset(self):
        rospy.loginfo(f"[{self._name}] reset!")
        self.restart_episode = False
        # visited points index placeholder (visited_points is an array of the index of visited points)
        ## Equal to the state memory
        start_point_index = 0
        self.visited_points = [start_point_index]

        # Reset the waypoint status (1 is reached, 0 is not reached)
        self.waypoints_status = [0] * self.num_waypoints
        
        # Reset time since waypoints have been reached to 0
        self.waypoints_time = [0] * self.num_waypoints

        # teleport robot to the starting point
        rospy.sleep(1.0)
        self.teleport(self.StartPoint_x, self.StartPoint_y)

        # Get the time robot start path planning
        time = rospy.get_rostime()
        self.time_start = time.to_sec()
        rospy.loginfo(f"[{self._name}] The time to reset time_start is [{self.time_start}]")

        # Reset the overall time to 0
        self.overall_time = 0
        rospy.loginfo(f"[{self._name}] reset overall_time = 0")

        # Spawn blue sphere to waypoint by default
        for j in range(len(WAYPOINT_YAWS)):
            if self.waypoints_status[j] == 0:
                wp_name = f"Waypoint_{j}"
                sphere = self.waypoint_labels[wp_name]
                sphere.spawn_blue(self.waypoints[j].position.x, self.waypoints[j].position.y)

        # Return the index of the first point robot starts (state)
        return start_point_index

    
    def render(self):
        pass


    # The action is next waypoint to go (next_waypoint)
    def step(self, next_waypoint):
        rospy.loginfo(f"[{self._name}] executes step(action), action: index of next waypoint to go is [{next_waypoint}] ({self.waypoints[next_waypoint].position.x}, {self.waypoints[next_waypoint].position.y})!")
        # print('action', next_waypoint)
        info = {}
        done = False
        action =  next_waypoint

        # Get current state (return the index of last point visited, initially, is the starting point)
        ## Actually, visited_points = state_memory
        state = self.visited_points[-1]

        # Update the new state
        new_state = action + 1

        """
        # Get reward from previous state and action
        reward_before_action = self._get_reward()
        reward_before_action = round(reward_before_action, 2)
        rospy.loginfo(f"[{self._name}] gets reward before action: {reward_before_action}!")
        """

        # Update the waypoints_status and the time elapsed
        self._play_action(action)
        
        # Update the color of waypoints that have been visited by the robot to green
        for j in range(len(WAYPOINT_YAWS)):
            if self.waypoints_status[j] == 1:
                wp_name = f"Waypoint_{j}"
                sphere = self.waypoint_labels[wp_name]
                sphere.spawn_green(self.waypoints[j].position.x, self.waypoints[j].position.y)

        # Update visited_points
        visited_point_index = new_state
        # + 1 beacuse the index of the starting point is "0" 
        ## e.g. if we have 6 waypoints, then point index = 1 2 3 4 5 6
        if visited_point_index not in self.visited_points:
            self.visited_points.append(visited_point_index)
        rospy.loginfo(f"[{self._name}] visited point list: {self.visited_points}!")

        # Calculate initial reward based on time in a known environment without obstacles
        if self.initialize_Q_table_by_time:
            basic_reward = -1 * self.q_visited_points[state, action] # try all three options to compare which way is better.
        else:
            basic_reward = 0

        # Get reward for new action
        reward = self._get_reward(action) + basic_reward
        ## reward = self._get_reward() - reward_before_action + basic_reward
        rospy.loginfo(f"[{self._name}] gets reward for new action [{action}]: {reward}!")

        # End episode when all waypoints have been visited
        if np.sum(self.waypoints_status) == self.num_waypoints:
            done = True
            rospy.loginfo("All waypoints have been reached!")
            """
            # If robot completed the route give additional reward
            ## Additional reward inversely proportional to time since start of episode.
            additional_reward = (self.max_time/(self.overall_time + 0.01)) * self.addi_reward_multiply_value
            additional_reward = round(additional_reward, 2)
            rospy.loginfo(f"[{self._name}] gets additional_reward for this episode: {additional_reward}!")
            reward += additional_reward
            """
            
        # # Episode end if maximum time is reached
        # if self.overall_time >= self.max_time:
        #     done = True
        #     info['final_status'] = 'max_time_exceeded'
        #     rospy.loginfo(f"[{self._name}] max_time_exceeded!")
        
        return new_state, reward, done, info


    def _play_action(self, action):
        """
        e.g.
        visited_points: [0, 2, 6]
        state = visited_points[-1] = 6
        action = next_waypoint = 3
        new_state = action + 1 = 3 + 1 = 4
        visited_points = visited_points.append(visited_point_index) = [0, 2, 6, 4] 
        # ignore the first element, only look at "2, 6, 4", 
        ## "visited_point_index" is the index of the next point to visit: 1 2 3 4 5 6
        
        waypoints_status            =   [0, 1, 0, 1, 0, 1]    in this case, we have 6 waypoints
        update waypoint status i    =    0  1  2  3  4  5
        """

        # Sets the color of the point the robot is currently heading to to red
        for j in range(len(WAYPOINT_YAWS)):
            if j == action:
                wp_name = f"Waypoint_{j}"
                sphere = self.waypoint_labels[wp_name]
                sphere.spawn_red(self.waypoints[j].position.x, self.waypoints[j].position.y)

        # Calculate the start time to each waypoint
        time = rospy.get_rostime()
        self.waypoint_time_start = time.to_sec()
        rospy.loginfo(f"[{self._name}] The time to start timing for action [{action}] is [{self.waypoint_time_start}]")

        # Move robot to next_waypoint
        wp = self.waypoints[action]
        wp_x = wp.position.x
        wp_y = wp.position.y
        wp_ori = wp.yaw
        self.move_base_action_client(wp_x, wp_y, wp_ori)

        # If the robot reaches the waypoint, then get current robot position
        self._get_robot_position()

        # Update the elapsed time for reaching each waypoint
        if self.waypoints_status[action] == 0:
            time = rospy.get_rostime()
            time_s = time.to_sec()
            wp_time = time_s - self.waypoint_time_start
            self.waypoints_time[action] = round(wp_time, 2)
            wp_time = self.waypoints_time[action]
            rospy.loginfo(f"[{self._name}] waypoints_time[{action}] = {wp_time}, done!")
        rospy.loginfo(f"[{self._name}] updates waypoints_time, done!")
        rospy.loginfo(f"[{self._name}] waypoints_time list: {self.waypoints_time}")

        # Update the waypoints_status
        _dist = dist(
            self.mir100_pos_x, self.mir100_pos_y, 
            self.waypoints[action].position.x, self.waypoints[action].position.y)
        if _dist < 0.8: # distance of the waypoint to the robot < 0.8 m
            self.waypoints_status[action] = 1
        rospy.loginfo(f"[{self._name}] updates waypoints_status, done!")
        rospy.loginfo(f"[{self._name}]'s current waypoints_status: {self.waypoints_status}")

        # Update the oveall time since agent left START_POINT
        time = rospy.get_rostime()
        tim_s = time.to_sec()
        o_time = tim_s - self.time_start
        self.overall_time = round(o_time, 2)
        rospy.loginfo(f"[{self._name}] updates overall_time, done!")
        rospy.loginfo(f"[{self._name}] overall_time taken: {self.overall_time}")
        

    # def move_to_wp(self, wp_x, wp_y, wp_ori):
    #     pub = rospy.Publisher('move_base_simple/goal', PoseStamped, queue_size=10)
    #     pub.publish(PoseStamped(header=Header(time=rospy.Time.now(), frame_id='map'), pose=Pose(position=Point(x=wp_x, y=wp_y, z=0), orientation=Quaternion(*wp_ori))))


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

            # Update the oveall time since agent left START_POINT
            time = rospy.get_rostime()
            tim_s = time.to_sec()
            o_time = tim_s - self.time_start
            self.overall_time = round(o_time, 2)
            # If maximum time exceed, restart this episode
            if self.overall_time >= self.max_time:
                self.restart_episode = True
                rospy.loginfo(f"[mir100_rl_1] needs to restart episode due to maximum time exceed!")
                if hasattr(self, 'action_client'):
                    self.action_client.cancel_all_goals()
                return

            x, y = self._get_robot_position()
            _dist = dist(x, y, goal_pose.position.x, goal_pose.position.y)
            if _dist < 0.6:
                rospy.loginfo(f'Close to waypoint, [{self._name}] is considered to have reached the waypoint, distance to waypoint: {_dist}')
                if hasattr(self, 'action_client'):
                    self.action_client.cancel_all_goals()
                return


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
        # rospy.loginfo(f"[{self._name}] get robot current position: ({self.mir100_pos_x}, {self.mir100_pos_y})!")
        return self.mir100_pos_x, self.mir100_pos_y


    # Generate the value for Q-matrix
    def _generate_q_values(self):
        """
        Generate actual Q Values corresponding to time elapsed between two waypoints
        """
        self.q_visited_points = []
        for i, wp_state in enumerate(self.all_points):
            self.q_visited_points.append([])
            # Calculate time that it takes from one waypoint to another 
            ## Here we just use the assumed values of distance and speed. 
            ### In the future, we should get the time directly based on movebase.
            for j, wp_action in enumerate(self.waypoints):
                self.q_visited_points[i].append(wp_state.duration(wp_action))


    # Get the reward for each pair of (state, action)
    def _get_reward(self, action):
        """
        agent gets a reward of -1 for each time step
        
        If a waypoint is delivered in that timestep, 
        it gets a positive reward inversely proportional to the time taken to deliver.
        
        If all the waypoints are delivered and the agent is back to the start point, 
        it gets an additional reward inversely proportional to time since start of episode.
        """
        # common_reward = np.sum(np.asarray(self.waypoints_status) * self.max_time / (np.asarray(self.waypoints_time) + 0.01)) - self.overall_time
        common_reward = -1 * self.waypoints_time[action]
        # common_reward = round(common_reward, 2)
        rospy.loginfo(f"[{self._name}] calculates common reward: {common_reward}")

        return common_reward


#----------------------------------------------------------------------------------------#

def run_episode(env,agent,current_episode,results_path):
    
    rospy.loginfo("[mir100_rl_1] starts a new episode!")
    
    # For each new episode, the environment is reset and return the initial state of the robot which is start_point_index
    s = env.reset()
    
    agent.reset_memory()

    # Max steps per episode (6 steps for 6 waypoints)
    max_step = env.num_waypoints
    
    episode_reward = 0
    
    i = 0
    while i < max_step:

        # Remember the wapoint robot just reached, except the starting point
        if s != 0:
            wp_index = s - 1
            agent.wp_arrived(wp_index)

        # Choose an action
        a = agent.act(s)
        rospy.loginfo(f"[mir100_rl_1] picks an action ({a})!")
        
        # Take the action, and get the reward from environment
        s_next,r,done,info = env.step(a)
        rospy.loginfo(f"for action ({a}), [mir100_rl_1]'s new state: {s_next}; reward: {r}; episode done: {done}; info: {info}!")
        
        # If there is a simulation error, restart this episode
        if env.restart_episode:
            rospy.loginfo(f"[mir100_rl_1] restarts episode <{current_episode}>")
            s = env.reset()
            agent.reset_memory()
            episode_reward = 0
            rospy.loginfo(f"[mir100_rl_1] reset episode_reward = 0, done!")

            # Read the previous Q table
            initial_q_table = np.load(results_path / f"RL_Qtable/RL_Qtable_{current_episode - 1}.npy")
            agent.Q = initial_q_table
            rospy.loginfo(f"[mir100_rl_1] read previous episode <{current_episode - 1}>'s Q_table, done!")

            # Read the previous episode's exploration rate
            exp_rate = []
            with open(results_path / f"RL_exploration_rate/RL_exploration_rate_{current_episode - 1}.txt") as f:
                for e in f.readlines():
                    exp_rate.append(float(e))
            agent.epsilon = exp_rate[-1]    # Use the last element of exploration_rate list as the initial epsilon value
            agent.exploration_rate = exp_rate
            rospy.loginfo(f"[mir100_rl_1] read previous episode <{current_episode - 1}>'s exploration_rate, done!")

            i = 0

        else:
            # For each action, use the Bellman equation to update our knowledge in the existing Q-table
            agent.train(s,a,r,s_next)
            # Update the caches
            episode_reward += r # The total reward of each episode
            s = s_next
            rospy.loginfo(f"[mir100_rl_1]'s step num.<{i}> is finished!")
            # If not, continue
            i += 1
        
        # If the episode is terminated
        if done:
            break
            
    return env,agent,episode_reward



class DeliveryQAgent(QAgent):

    def __init__(self,*args,**kwargs):
        self._name = 'mir100_rl_1'
        rospy.loginfo(f"Initialize QAgent [{self._name}]!")

        super().__init__(*args,**kwargs)
        self.reset_memory()

    def act(self,s):
        # Get Q Vector
        q = np.copy(self.Q[s,:])

        # Avoid already visited waypoints
        q[self.wp_memory] = -np.inf

        # Since the epsilon (Exploration rate) is gradually decreasing, if the rendom number between 0 & 1 is larger than the epsilon
        # the action will be the index of the maximum q value along the axis (make decisions based on Q-table)
        if np.random.rand() > self.epsilon:
            a = np.argmax(q)    # --> exploitation
        else:
            # else, at the very beginning, we will keep exploring by randomly selecting remaining waypoints as the action
            a = np.random.choice([x for x in range(self.actions_size) if x not in self.wp_memory])  # --> exploration

        return a

    def wp_arrived(self,wp_index):
        self.wp_memory.append(wp_index)

    def reset_memory(self):
        self.wp_memory = []



def run_num_episodes(env, agent, num_episodes=100, external_data=False):

    results_path = BASE_PATH.parent.parent.parent / 'Results_Plot'
    if not results_path.exists():
        results_path.mkdir()

    rewards = []
    overall_times = []
    rest_num_episode = num_episodes
    
    # Store the rewards by empty list of input from externally
    if external_data:
        # Read rewards
        rewards = []
        with open(results_path / "RL_rewards/RL_rewards.txt") as f:
            for r in f.readlines():
                rewards.append(float(r))
        
        # Read overall_times
        overall_times = []
        with open(results_path / "RL_overall_times/RL_overall_times.txt") as f:
            for t in f.readlines():
                overall_times.append(float(t))

        # Read exploration rate
        exp_rate = []
        with open(results_path / "RL_exploration_rate/RL_exploration_rate.txt") as f:
            for e in f.readlines():
                exp_rate.append(float(e))

        agent.epsilon = exp_rate[-1]    # Use the last element of exploration_rate list as the initial epsilon value
        agent.exploration_rate = exp_rate

        # Read the Q table
        initial_q_table = np.load(results_path / "RL_Qtable/RL_Qtable.npy")
        agent.Q = initial_q_table

        cur_num_episode = len(rewards)
        rest_num_episode = num_episodes - cur_num_episode

    
    for i in range(rest_num_episode):

        if external_data:
            cur_i = i + cur_num_episode + 1
        else:
            cur_i = i + 1

        # Run the episode
        rospy.loginfo(f"[mir100_rl_1] episode <{cur_i}>! (total number of episode: {num_episodes})")
        env,agent,episode_reward = run_episode(env,agent,cur_i,results_path)

        # Update the "overall_times"
        overall_times.append(env.overall_time)
        rospy.loginfo(f"[mir100_rl_1] appends oveall_times into list: {overall_times}!")
        np.savetxt(results_path / f"RL_overall_times/RL_overall_times_{cur_i}.txt", np.array(overall_times), fmt='%f',delimiter=',')
        rospy.loginfo(f"[mir100_rl_1] successfully writes the overall_times to {results_path}!")
        
        # Update the "rewards"
        rewards.append(episode_reward)
        rospy.loginfo(f"[mir100_rl_1] appends episode_reward into list: {rewards}!")
        np.savetxt(results_path / f"RL_rewards/RL_rewards_{cur_i}.txt", np.array(rewards), fmt='%f',delimiter=',')
        rospy.loginfo(f"[mir100_rl_1] successfully writes the rewards to {results_path}!")

        # Update the "epsilon"
        np.savetxt(results_path / f"RL_exploration_rate/RL_exploration_rate_{cur_i}.txt", np.array(agent.exploration_rate), fmt='%f',delimiter=',')
        rospy.loginfo(f"[mir100_rl_1] successfully writes the exploration_rate to {results_path}!")

        longest_t = max(overall_times)
        shortest_t = min(overall_times)
        index_shortest_t = overall_times.index(shortest_t)
        episode_shortest_t = index_shortest_t + 1
        shortest_t_reward = rewards[index_shortest_t]
        rospy.loginfo(f"The minimum time for the [mir100_rl_1] to complete the task is {shortest_t} seconds in episode <{episode_shortest_t}> with reward ({shortest_t_reward})!")
        rospy.loginfo(f"The maximum time for the [mir100_rl_1] to complete the task is {longest_t} seconds!")

        # Show rewards, overall time, and exploration rate(epsilon)
        fig1, ax1 = plt.subplots(3, 1, figsize=(15,15))
        ax1[0].plot(rewards)
        ax1[1].plot(overall_times)
        ax1[2].plot(agent.exploration_rate)
        ax1[0].set_title(f"RL Method: Rewards over Training({cur_i} Episodes)", fontsize=16, fontweight= 'bold', pad=10)
        ax1[1].set_title(f"RL Method: Overall Time Taken over Training({cur_i} Episodes)", fontsize=16, fontweight= 'bold', pad=10)
        ax1[2].set_title(f"RL Method: Exploration Rate(Exponential decay rate: {agent.epsilon_decay})", fontsize=16, fontweight= 'bold', pad=10)
        ax1[0].set_xlabel("Episode")
        ax1[0].set_ylabel("Rewards")
        ax1[1].set_xlabel("Episode")
        ax1[1].set_ylabel("Overall Time (Unit: second)")
        ax1[2].set_xlabel(f"Number of Training")
        ax1[2].set_ylabel("Epsilon")
        fig1.tight_layout()
        plt.subplots_adjust(wspace=0,hspace=0.25)
        plt.savefig(results_path / 'RL_Rewards_and_OverallTime.png', dpi = 200)
        #plt.show()
        rospy.loginfo(f"[mir100_rl_1] successfully saves Rewards_and_OverallTime.png to {results_path}!")

        # Plot the Q table
        fig2, ax2 = plt.subplots(1, 1, figsize=(12,3))
        data = agent.Q
        column_labels = ['waypoint %d' % y for y in range(env.action_space)]
        row_labels = ['Starting point']
        row_labels += column_labels
        df = pd.DataFrame(data,columns=column_labels)
        ax2.axis('tight')
        ax2.axis('off')
        ax2.table(cellText=df.values,
                rowLabels=row_labels,
                colLabels=df.columns,
                rowColours =["yellow"] * env.state_space,  
                colColours =["yellow"] * env.action_space,
                loc="center",
                cellLoc="center",
                rowLoc="center")
        ax2.set_title(f"Q Table ({cur_i} Episodes)", fontsize=16, fontweight= 'bold', pad=10)
        np.save(results_path / f"RL_Qtable/RL_Qtable_{cur_i}.npy", data)
        plt.savefig(results_path / 'RL_Q_table.png', dpi = 200)
        # plt.show()
        rospy.loginfo(f"[mir100_rl_1] successfully saves Q_table.png to {results_path}!")

    rospy.loginfo("All training of the [mir100_rl_1] robot is over!")

    return env,agent


#----------------------------------------------------------------------------------------#


if __name__ == u'__main__':
    rospy.init_node('mir100_rl_1', anonymous=True)

    rospy.loginfo('sleep for 2 second')
    time.sleep(2)
    rospy.loginfo('wait for service to unpause')
    rospy.wait_for_service('/gazebo/unpause_physics')
    rospy.loginfo('calling service to unpause')
    unpause_physics_client = rospy.ServiceProxy('/gazebo/unpause_physics', Empty)
    unpause_physics_client(EmptyRequest())
    rospy.loginfo('should be unpaused')

    env = DynamicObstacleNavigationMir100Sim()

    """        
    gamma           # Discounting rate
    lr              # Learning rate
    
    # Exploration parameters
    epsilon         # Exploration rate
    epsilon_min     # Minimum exploration rate
    epsilon_decay   # Exponential decay rate 
    """
    agent = DeliveryQAgent(env.state_space,env.action_space,epsilon = 1.0,epsilon_min = 0.01,epsilon_decay = 0.999,gamma = 0.95,lr = 0.8)

    num_episodes = 800
    run_num_episodes(env,agent,num_episodes,external_data=False)

    pause_physics_client = rospy.ServiceProxy('/gazebo/pause_physics', Empty)
    pause_physics_client(EmptyRequest())

