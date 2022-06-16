#!/usr/bin/env python3
# -*- coding: utf-8 -*-
from cmath import sqrt
from turtle import distance
import rospy 
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
    def __init__(self, model_name):
        self.model_name = model_name
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

    def move_step(self):
        path = self.waypoints
        time = rospy.get_rostime()
        now = time.to_sec()
        if self.stop_case(self.position):
            if not self.pause_start:
                self.pause_percentage = self.percentage
                self.pause_start = now
            return
        if self.pause_start:
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
        state_msg = ModelState()
        state_msg.model_name = self.model_name
        self.position.x = self.last_wp.x + distance_x
        self.position.y = self.last_wp.y + distance_y
        self.position.z = 0.9
        state_msg.pose.position = self.position
        rospy.wait_for_service('/gazebo/set_model_state')
        set_state = rospy.ServiceProxy(
            '/gazebo/set_model_state', SetModelState)
        resp = set_state(state_msg )

    def stop_case(self, position):
        state_msg = "mir"
        rospy.wait_for_service('/gazebo/get_model_state')
        get_state = rospy.ServiceProxy(
            '/gazebo/get_model_state', GetModelState)
        resp = get_state(model_name = "mir")
        dist_pos = math.sqrt((position.x - resp.pose.position.x)**2 + (position.y - resp.pose.position.y)**2)
        # rospy.loginfo(dist_pos)
        return dist_pos < 2.0



class MoveToWaypoint:
    def move_obstacles(self):
        for ob in self.obstacles:
            ob.move_init()
        while not rospy.is_shutdown():
            for ob in self.obstacles:
                ob.move_step()



if __name__ == u'__main__':
    rospy.init_node('move_obstacle', anonymous=True)

    obstacle1 = Obstacle('first_obstacle')
    obstacle1.waypoints = [
        Waypoint(0.0, 0, 0),
        Waypoint(18.0, 0, 20),
        Waypoint(0.0, 0, 20),
    ]
    obstacle1.spawn(6.0, 0)

    obstacle2 = Obstacle('second_obstacle')
    obstacle2.waypoints = [
        Waypoint(-1.0, 0, 0),
        Waypoint(-11.0, 0, 14),
        Waypoint(-1.0, 0, 14)
    ]
    obstacle2.spawn(-1.0, 0)

    mtwp = MoveToWaypoint()
    mtwp.obstacles = [obstacle1, obstacle2]
    mtwp.move_obstacles()

