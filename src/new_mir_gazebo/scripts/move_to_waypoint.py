#!/usr/bin/env python3
# -*- coding: utf-8 -*-
from cmath import sqrt
from turtle import distance
import rospy 
import math
from geometry_msgs.msg import Pose
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
          <cylinder>
            <radius>0.3</radius>
            <length>1.8</length>
          </cylinder>
        </geometry>
      </collision>
      <visual name='NAME_visual'>
        <geometry>
          <cylinder>
            <radius>0.3</radius>
            <length>1.8</length>
          </cylinder>
        </geometry>
        <material>
          <script>
            <uri>file://media/materials/scripts/gazebo.material</uri>
            <name>Gazebo/Purple</name>
          </script>
          <ambient>1 1 1 1</ambient>
        </material>
        <meta>
          <layer>0</layer>
        </meta>
      </visual>
    </link>
    <static>1</static>
  </model>
</sdf>
'''

class Obstacle:
    def __init__(self, model_name):
        self.model_name = model_name

    def spawn(self, x, y):
        rospy.wait_for_service('gazebo/spawn_sdf_model', 5.0)
        spawn_model_prox = rospy.ServiceProxy(
            'gazebo/spawn_sdf_model', SpawnModel)
        spawn_pose = Pose()
        spawn_pose.position.x = x
        spawn_pose.position.y = y
        model_xml = OBS.replace('NAME', self.model_name)

        res = spawn_model_prox(
            self.model_name, model_xml, '', spawn_pose, 'world')

    def move_init(self):
        self.waypoint_idx = 0
        path = self.waypoints
        self.last_wp = path[self.waypoint_idx]
        self.next_wp = path[self.waypoint_idx+1]
        self.start = 0
        self.end = self.next_wp.time

    def move_step(self):
        path = self.waypoints
        time = rospy.get_rostime()
        now = time.to_sec()
        # check if we need to get the next waypoint
        if now > self.end:
            self.waypoint_idx += 1
            if self.waypoint_idx + 1 >= len(path):
                # we are at the last waypoint, loop!
                self.waypoint_idx = 0
            self.last_wp = path[self.waypoint_idx]
            self.next_wp = path[self.waypoint_idx+1]
            self.start = now
            self.end = now + self.next_wp.time

        # calculate how far we should have walked during this time (in percentage)
        percentage = (now - self.start) / (self.end - self.start)
        distance_x = (self.next_wp.x - self.last_wp.x) * percentage
        distance_y = (self.next_wp.y - self.last_wp.y) * percentage

        # set model pose, add distance x and y to last position, copied from https://answers.gazebosim.org/question/22125/how-to-set-a-models-position-using-gazeboset_model_state-service-in-python/
        state_msg = ModelState()
        state_msg.model_name = self.model_name
        state_msg.pose.position.x = self.last_wp.x + distance_x
        state_msg.pose.position.y = self.last_wp.y + distance_y
        state_msg.pose.position.z = 0.9
        if self.stop_case(state_msg.pose.position):
          return
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
        resp.pose.position.x
        resp.pose.position.y
        dist_pos = math.sqrt((position.x - resp.pose.position.x)**2 + (position.y - resp.pose.position.y)**2)
        # rospy.loginfo(dist_pos)
        return dist_pos < 0.8



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
        Waypoint(9.0, -2.0, 1.0),
        Waypoint(11.0, -2.0, 1.0),
        Waypoint(9.0, -2.0, 1.0)
    ]
    obstacle1.spawn(9, -2)

    obstacle2 = Obstacle('second_obstacle')
    obstacle2.waypoints = [
        Waypoint(33.0, -19.0, 3.0),
        Waypoint(38.0, -19.0, 3.0),
        Waypoint(33.0, -19.0, 3.0)
    ]
    obstacle2.spawn(33, -19)

    mtwp = MoveToWaypoint()
    mtwp.obstacles = [obstacle1, obstacle2]
    mtwp.move_obstacles()

