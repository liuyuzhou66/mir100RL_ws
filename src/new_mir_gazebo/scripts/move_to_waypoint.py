#!/usr/bin/env python3
# -*- coding: utf-8 -*-
from cmath import sqrt
from turtle import distance
import rospy 
import time
import math
from geometry_msgs.msg import Pose, Point
from gazebo_msgs.msg import ModelState 
from gazebo_msgs.srv import SetModelState, SpawnModel, GetModelState


class Waypoint:
    def __init__(self, x, y, time):
        self.x = x
        self.y = y
        self.time = time # time in seconds after last wp to move to this one


OBS = '''<?xml version='1.0'?>
<sdf version='1.6'>
  <model name='NAME'>
    <pose>0 0 0.9 0 0 0</pose>
    <link name='NAME_link'>
      <inertial>
        <mass>50</mass>
      </inertial>
      <collision name='NAME_collision'>
        <geometry>
          <box>
            <size>1.0 1.0 1.8</size>
          </box>
        </geometry>
      </collision>
      <visual name='NAME_visual'>
        <geometry>
          <box>
            <size>0.8 0.8 1.8</size>
          </box>
        </geometry>
        <material>
          <lighting>1</lighting>
          <script>
            <uri>file://media/materials/scripts/gazebo.material</uri>
            <name>Gazebo/Purple</name>
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
  </model>
</sdf>
'''

class Obstacle:
    def __init__(self, model_name, waypoints):
        rospy.wait_for_service('/gazebo/set_model_state')
        self.model_name = model_name
        self.waypoints = waypoints
        self.pause_start = None
        self.pause_percentage = None
        self.position = Point()

    def spawn(self, x, y):
        rospy.wait_for_service('gazebo/spawn_sdf_model', 5.0)
        spawn_model_prox = rospy.ServiceProxy(
            'gazebo/spawn_sdf_model', SpawnModel)
        self.position.x = x
        self.position.y = y
        spawn_pose = Pose(position=self.position)
       
        model_xml = OBS.replace('NAME', self.model_name)

        res = spawn_model_prox(
            self.model_name, model_xml, '', spawn_pose, 'world')

    def move_init(self):
        self.waypoint_idx = 0
        path = self.waypoints
        self.last_wp = path[self.waypoint_idx]
        self.next_wp = path[self.waypoint_idx+1]
        self.start = 0
        self.add_percentage = 0
        self.percentage = 0
        self.end = self.next_wp.time

    def move_step(self, robot_pos):
        path = self.waypoints
        time = rospy.get_rostime()
        now = time.to_sec()

        # check if the robot is just 2 meter away
        if self.robot_distance(robot_pos) < 2.0:
            if not self.pause_start:  # start to wait infront of the robot
                self.pause_percentage = self.percentage
                self.pause_start = now
            elif self.pause_start and now - self.pause_start > 30:
                # we waited for 30 seconds, allow the robot to move, we
                # teleport 2 waypoints further
                idx = (self.waypoint_idx + 2) % len(path)
                self.waypoint_idx = idx
                self.last_wp = path[self.waypoint_idx]
                position = Point()
                position.x = self.last_wp.x
                position.y = self.last_wp.y
                self.teleport(position, robot_pos)
                self.pause_percentage = 0
                self.pause_start = None
                self.add_percentage = 0
                self.percentage = 0
            return
        if self.pause_start:
            # robot is out of the way but we waited
            # continue moving from here
            self.start = now
            self.end = now + self.next_wp.time
            self.add_percentage = self.pause_percentage
            self.pause_start = None

        percentage = (now - self.start) / (self.end - self.start)
        # rospy.loginfo(f'calculated percentage: {percentage}')
        percentage += self.add_percentage
        # rospy.loginfo(f'wait time: {self.add_percentage}')
        # check if we need to get the next waypoint
        if percentage >= 1.0:
            # rospy.loginfo(f'switch to next waypoint')
            self.waypoint_idx += 1
            if self.waypoint_idx + 1 >= len(path):
                # we are at the last waypoint, loop!
                self.waypoint_idx = 0
            self.last_wp = path[self.waypoint_idx]
            self.next_wp = path[self.waypoint_idx+1]
            self.start = now
            self.end = now + self.next_wp.time
            self.pause_percentage = 0
            self.add_percentage = 0
            self.pause_start = None

        # calculate how far we should have walked during this time (in percentage)
        distance_x = (self.next_wp.x - self.last_wp.x) * percentage
        distance_y = (self.next_wp.y - self.last_wp.y) * percentage
        self.percentage = percentage
        # set model pose, add distance x and y to last position, copied from https://answers.gazebosim.org/question/22125/how-to-set-a-models-position-using-gazeboset_model_state-service-in-python/
        position = Point()
        position.x = self.last_wp.x + distance_x
        position.y = self.last_wp.y + distance_y
        self.teleport(position, robot_pos)

    def teleport(self, position, robot_pos):
        # make sure we do not teleport into the robot
        if self.robot_distance(robot_pos) < 0.9:
            return
        
        # all okay, generate state-message
        state_msg = ModelState()
        state_msg.model_name = self.model_name
        self.position.x = position.x
        self.position.y = position.y
        self.position.z = 0.9        
        state_msg.pose.position = self.position
        set_state = rospy.ServiceProxy(
            '/gazebo/set_model_state', SetModelState)
        resp = set_state(state_msg)
        if not resp.success:
            rospy.loginfo(f'Error sending robot state {state_msg}')
        
    def robot_distance(self, robot_pos):
        # calculatae distance from obstacle to robot
        return math.sqrt(
            (self.position.x - robot_pos.x)**2 + 
            (self.position.y - robot_pos.y)**2)


class MoveToWaypoint:
    def __init__(self, obstacles):
        self.obstacles = obstacles
        rospy.wait_for_service('/gazebo/get_model_state')
        for obstacle in obstacles:
            pos = obstacle.waypoints[0]
            obstacle.spawn(pos.x, pos.y)

    def move_obstacles(self):
        FPS = 30
        sleep_time = 1.0/(FPS*len(self.obstacles))
        for ob in self.obstacles:
            ob.move_init()

        while not rospy.is_shutdown():
            robot_pos = self.get_robot_position()
            for ob in self.obstacles:
                ob.move_step(robot_pos)
                # rospy.sleep(sleep_time)

    def get_robot_position(self):
        get_state = rospy.ServiceProxy(
            '/gazebo/get_model_state', GetModelState)
        resp = get_state(model_name = "mir")
        return resp.pose.position

if __name__ == u'__main__':
    rospy.init_node('move_obstacle', anonymous=True)

    obstacle1 = Obstacle(
        'obstacle_1', 
        [
            Waypoint(-17.5, -5.0, 0),
            Waypoint(-17.5, -10.0, 6),
            Waypoint(-13.35, -10.0, 6),
            Waypoint(-13.35, -15.0, 6),
            Waypoint(-17.35, -15.0, 6),
            Waypoint(-17.5, -10.0, 6),
            Waypoint(-13.35, -10.0, 6),
            Waypoint(-13.35, -5.0, 6),
            Waypoint(-17.5, -5.0, 6)
        ]
    )

    obstacle2 = Obstacle(
        'obstacle_2',
        [
            Waypoint(-10.3, -7.5, 0),
            Waypoint(-10.3, -15.0, 7),
            Waypoint(-7.4, -15.0, 3),
            Waypoint(-7.4, -7.5, 7),
            Waypoint(-10.3, -7.5, 3)
        ]
    )

    obstacle3 = Obstacle(
        'obstacle_3',
        [
            Waypoint(-0.55, -7.8, 0),
            Waypoint(6.0, -7.8, 9),
            Waypoint(7.5, -15.0, 10),
            Waypoint(-0.55, -15.0, 10),
            Waypoint(-0.55, -7.8, 7)
        ]
    )

    obstacle4 = Obstacle(
        'obstacle_4',
        [
            Waypoint(7.4, -7.88, 0),
            Waypoint(7.4, 0.0, 10),
            Waypoint(12.0, 0.0, 4),
            Waypoint(12.0, -7.88, 10),
            Waypoint(7.4, -7.88, 5)
        ]
    )

    obstacle5 = Obstacle(
        'obstacle_5',
        [
            Waypoint(12.0, -15.0, 0),
            Waypoint(12.0, -10.3, 7),
            Waypoint(14.6, -10.3, 5),
            Waypoint(14.6, -5.5, 4),
            Waypoint(17.5, -5.5, 2),
            Waypoint(17.5, -15.0, 9)
        ]
    )

    obstacle6 = Obstacle(
        'obstacle_6',
        [
            Waypoint(-1.2, 0.0, 0),
            Waypoint(-10.0, 0.0, 9),
            Waypoint(-10.0, 4.0, 3),
            Waypoint(-1.2, 7.0, 10),
            Waypoint(-1.2, 0.0, 7)
        ]
    )

    obstacle7 = Obstacle(
        'obstacle_7',
        [
            Waypoint(-10.0, 7.8, 0),
            Waypoint(-10.0, 15.0, 7),
            Waypoint(-1.2, 15.0, 9),
            Waypoint(-1.2, 10.8, 3),
            Waypoint(-10.0, 7.8, 10)
        ]
    )

    mtwp = MoveToWaypoint([
        obstacle1, obstacle2, obstacle3, obstacle4,
        obstacle5, obstacle6, obstacle7,
    ])
    mtwp.move_obstacles()

