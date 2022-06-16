#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import math
import time
import rospy, roslib
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

from nav_msgs.msg import Path   #?????????
from move_base_msgs.msg import MoveBaseActionGoal
from actionlib_msgs.msg import GoalID

import sys
sys.path.append("../")

BASE_PATH = Path(os.path.dirname(__file__))

WAYPOINT_POSITIONS = [-13.0,10.0,0, -5.0,10.0,0, 9.3,12.3,0, 12.0,-12.0,0, -4.0,-10.5,0, -15.3,-10.0,0]

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



class PathPlanning:
    def __init__(self):
        self._name = 'mir100_classic_1'
        rospy.loginfo(f'[{self._name}] starting node <Path_Planning>')
        # Create a list to store all the waypoints
        self.waypoints = []
        self.waypoints += [
            Waypoint(Point(*WAYPOINT_POSITIONS[i*3:(i*3)+3]),
            WAYPOINT_YAWS[i]
            )
            for i in range(len(WAYPOINT_YAWS))]

        self.num_waypoints = len(self.waypoints)

        # Create a dictionary to store the spawn status of waypoint labels
        self.waypoint_labels = {}
        for j in range(len(WAYPOINT_YAWS)):
            wp_name = f"Waypoint_{j}"
            sphere = Waypoint_Label(wp_name)
            self.waypoint_labels[wp_name] = sphere
        self.StartPoint_x = START_POINT.position.x
        self.StartPoint_y = START_POINT.position.y
        self.StartPoint_z = START_POINT.position.z
        self.StartPoint_ori = quaternion_from_euler(0, 0, np.deg2rad(START_POINT.yaw))

        self.path_dist = []
        self.waypoints_status = [0] * self.num_waypoints

        # Get the time robot start path planning
        time = rospy.get_rostime()
        self.time_start = time.to_sec()
        rospy.loginfo(f"[{self._name}] starts timing at [{self.time_start}]")


    def reset_path_dist_list(self):
        self.path_dist = []
        

    def move_to_waypoint(self, wp_idx):
        done = False
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

        return done


    def pick_waypoint(self):
        for wp_i in range(len(WAYPOINT_YAWS)):
            wp = self.waypoints[wp_i]
            wp_x = wp.position.x
            wp_y = wp.position.y
            wp_ori = wp.yaw
            total_dist = self.calculate_path_distance(wp_x, wp_y, wp_ori)
            self.path_dist.append(total_dist)
        
        # Get the waypoint index of waypoint with the shortest path distance
        min_dist = min(self.path_dist)
        wp_index = self.path_dist.index(min_dist)

        return wp_index


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


    def move_base_watcher_thread(self, goal_pose):
        while True:
            rospy.sleep(1)
            x, y = self._get_robot_position()
            _dist = dist(x, y, goal_pose.position.x, goal_pose.position.y)
            if _dist < 0.5:
                rospy.loginfo(f'Close to waypoint, [{self._name}] is considered to have reached the waypoint, distance to waypoint: {_dist}')
                if hasattr(self, 'action_client'):
                    self.action_client.cancel_all_goals()
                return


    def _get_robot_position(self):
        rospy.wait_for_service('/gazebo/get_model_state')
        get_state = rospy.ServiceProxy(
            '/gazebo/get_model_state', GetModelState)
        resp = get_state(model_name = "mir")
        self.mir100_pos_x = resp.pose.position.x
        self.mir100_pos_y = resp.pose.position.y
        # rospy.loginfo(f"[{self._name}] get robot current position: ({self.mir100_pos_x}, {self.mir100_pos_y})!")
        return self.mir100_pos_x, self.mir100_pos_y



def run_greedy(P):
    # Pick a waypoint which has shortest distance
    wp_idx = P.pick_waypoint()

    done,overall_time = P.move_to_waypoint(wp_idx)

    if done:
        break

    return overall_time


if __name__ == u'__main__':
    planning = PathPlanning()

    run_greedy(planning)

