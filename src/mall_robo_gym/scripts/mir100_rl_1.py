#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import rospy 
import numpy as np
from dataclasses import dataclass
# from tqdm import tqdm_notebook
import matplotlib.pyplot as plt
from std_msgs.msg import Header
from scipy.spatial.distance import cdist
# from robo_gym.utils import utils, mir100_utils
# from robo_gym.envs.mir100.mir100 import Mir100Env
#from robo_gym.envs.simulation_wrapper import Simulation
from gazebo_msgs.srv import GetModelState, SetModelState, SpawnModel, DeleteModel
from gazebo_msgs.msg import ModelState 
from geometry_msgs.msg import Pose, PoseStamped, Quaternion, Point, PoseWithCovarianceStamped
from tf.transformations import quaternion_from_euler
from agents.q_agent import QAgent

# Brings in the SimpleActionClient
import actionlib
import move_base_msgs.msg
import geometry_msgs.msg

import sys
sys.path.append("../")


WAYPOINT_POSITIONS = [-20,-29,0,-20.5,-10,0,-7,13.5,0,13.5,-6.5,0,21.5,-7,0,19,-32,0]
#                    |---------|-----------|---------|-----------|---------|--------|
#                         A          B          C          D          E         F
#                         0          1          2          3          4         5

WAYPOINT_YAWS = [90, 90, 90, 270, 30, 0]

SPEED = 1


@dataclass
class Waypoint:
    position: Point
    yaw: int
    # By considering a long-term decision making strategy
    ## We initialize the Q matrix with the time(distance/speed) matrix between (waypoints) and (waypoints + starting point) 
    def duration(self, wp):
        # time it takes to move from one waypoint to another
        # should be replaced with a table later on
        return cdist(
            [[self.position.x, self.position.y]],
            [[wp.position.x, wp.position.y]]
        )[0][0] / SPEED

# Robot starts at 0-position (the index of starting point is 0)
START_POINT = Waypoint(Point(1, -38.5, 0), 0)

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
    <pose>0 0 3.0 0 0 0</pose>
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
        self.rate = rospy.Rate(10)

    def delete(self):
        rospy.wait_for_service('gazebo/delete_model', 5.0)
        self.rate.sleep()
        spawn_model_prox = rospy.ServiceProxy(
            'gazebo/delete_model', DeleteModel)
        res = spawn_model_prox(self.model_name)
        self.spawned = False

    def _spawn(self, x, y, color):
        if self.spawned:
            self.delete()
        rospy.wait_for_service('gazebo/spawn_sdf_model', 5.0)
        spawn_model_prox = rospy.ServiceProxy(
            'gazebo/spawn_sdf_model', SpawnModel)
        spawn_pose = Pose()
        spawn_pose.position.x = x
        spawn_pose.position.y = y
        spawn_pose.position.z = 2.0 
        model_xml = WP_Label.replace('NAME', self.model_name)
        model_xml = model_xml.replace('COLOR', color)
        self.rate.sleep()
        res = spawn_model_prox(
            self.model_name, model_xml, '', spawn_pose, 'world')
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
        rospy.loginfo(f'[{self._name}] starting node DynamicObstacleNavigationMir100Sim')
        self.waypoints = []
        self.waypoints += [
            Waypoint(Point(*WAYPOINT_POSITIONS[i*3:(i*3)+3]),
            WAYPOINT_YAWS[i]
            )
            for i in range(len(WAYPOINT_YAWS))]

        # Create a dictionary to store the spawn status of waypoint labels
        self.waypoint_labels = {}

        # Spawn blue sphere to waypoint by default
        for j in range(len(WAYPOINT_YAWS)):
            wp_name = f"Waypoint_{j}"
            sphere = Waypoint_Label(wp_name)
            self.waypoint_labels[wp_name] = sphere
            sphere.spawn_blue(self.waypoints[j].position.x, self.waypoints[j].position.y)

        # Set the robot initial position to starting point(1.5, -38.5, 0)
        self.all_points = [START_POINT]
        self.all_points += self.waypoints
        # self.all_points.append(START_POINT)

        self.StartPoint_x = START_POINT.position.x
        self.StartPoint_y = START_POINT.position.y
        self.StartPoint_z = START_POINT.position.z
        self.StartPoint_ori = quaternion_from_euler(0, 0, START_POINT.yaw * math.pi / 180)

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
        self.max_time = 900        # Maximum time for each episode: 0.25 h = 15 min = 900 s
        self.initialize_Q_table_by_time = False

        # The Multiplier value for additional reward. This needs to be adjusted!
        self.addi_reward_multiply_value = 10   # We are more concerned with reducing the overall time rather than the time it takes to get to a certain waypoint.

        # Initialize basic reward in Q-table based on the time taken between (the starting point + waypoints) and (waypoints) in an obstacle-free environment
        if self.initialize_Q_table_by_time:
            self._generate_q_values()

        # Restart the environment on a new episode. Return the start_point_index
        # self.reset()


    def reset(self):
        rospy.loginfo(f"[{self._name}] reset!")
        # visited points index placeholder (visited_points is an array of the index of visited points)
        ## Equal to the state memory
        start_point_index = 0
        self.visited_points = [start_point_index]

        # Reset the waypoint status (1 is reached, 0 is not reached)
        self.waypoints_status = [0] * self.num_waypoints
        
        # Reset time since waypoints have been reached to 0
        self.waypoints_time = [0] * self.num_waypoints

        # teleport robot to the starting point
        self.teleport(self.StartPoint_x, self.StartPoint_y)

        # Get the time robot start path planning
        time = rospy.get_rostime()
        self.time_start = time.to_sec()
        rospy.loginfo(f"[{self._name}] The time to start timing is [{self.time_start}]")

        # Reset the overall time to 0
        self.overall_time = 0

        # Return the index of the first point robot starts (state)
        return start_point_index

    
    def render(self):
        pass


    # The action is next waypoint to go (next_waypoint)
    def step(self, next_waypoint):
        rospy.loginfo(f"[{self._name}] executes step(action), action: index of next waypoint to go is [{next_waypoint}]!")
        print('action', next_waypoint)
        info = {}
        done = False
        action =  next_waypoint

        # Get current state (return the index of last point visited, initially, is the starting point)
        ## Actually, visited_points = state_memory
        state = self.visited_points[-1]

        # Update the new state
        new_state = action + 1
        
        # Get reward from previous state and action
        reward_before_action = self._get_reward()
        rospy.loginfo(f"[{self._name}] gets reward before action!")

        # Update the waypoints_status and the time elapsed
        self._play_action(action)
        
        # Update visited_points
        visited_point_index = new_state
        # + 1 beacuse the index of the starting point is "0" 
        ## e.g. if we have 6 waypoints, then point index = 1 2 3 4 5 6
        if visited_point_index not in self.visited_points:
            self.visited_points.append(visited_point_index)

        # Calculate initial reward based on time in a known environment without obstacles
        if self.initialize_Q_table_by_time:
            # The longer the the time, the smaller the basic reward
            # We need to know how to involve the initial reward value into the Q value update, 
            # Do we want the initial reward to have a large impact on the total reward?
            ## what is the relationship between the initial reward value and time? 
            ### Inverse proportional function: y = 10/x (self.max_time / self.q_visited_points[state, action])
            ### Exponential function: y = 0.5^x
            ### Linear function: y = -1*x
            basic_reward = -1 * self.q_visited_points[state, action] # try all three options to compare which way is better.
        else:
            basic_reward = 0

        # Get reward for new action
        reward = self._get_reward() - reward_before_action + basic_reward
        rospy.loginfo(f"[{self._name}] gets reward for new action [{action}]!")

        # End episode when all waypoints have been visited
        if np.sum(self.waypoints_status) == self.num_waypoints:
            done = True
            # If robot completed the route give additional reward
            ## Additional reward inversely proportional to time since start of episode.
            reward += (self.max_time/(self.overall_time + 0.0001)) * self.addi_reward_multiply_value

        # Episode end if maximum time is reached
        if self.overall_time >= self.max_time:
            done = True
            info['final_status'] = 'max_time_exceeded'
            rospy.loginfo(f"[{self._name}] max_time_exceeded!")
        
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

        # Move robot to next_waypoint
        wp = self.waypoints[action]
        wp_x = wp.position.x
        wp_y = wp.position.y
        wp_ori = wp.yaw
        self.move_base_action_client(wp_x, wp_y, wp_ori)

        # If the robot reaches the waypoint, then get current robot position
        self._get_robot_position()
        
        # Update the waypoint_status
        for i in range(self.num_waypoints):
            if self.waypoints_status[i] == 0:
                # When the robot reaches the waypoint
                if abs(self.mir100_pos_x - self.waypoints[action].position.x) < 0.1 and abs(self.mir100_pos_y - self.waypoints[action].position.y) < 0.1: 
                    # Will there be a situation where the coordinates cannot be exactly the same?
                    self.waypoints_status[i] = 1
        rospy.loginfo(f"[{self._name}] updates waypoints_status, done!")
        
        # Update the color of waypoints that have been visited by the robot to green
        for j in range(len(WAYPOINT_YAWS)):
            if self.waypoints_status[j] == 1:
                wp_name = f"Waypoint_{j}"
                sphere = self.waypoint_labels[wp_name]
                sphere.spawn_green(self.waypoints[j].position.x, self.waypoints[j].position.y)
        

        # Update the elapsed time for reaching each waypoint
        for i in range(self.num_waypoints):
            if self.waypoints_status[i] == 0:
                time = rospy.get_rostime()
                time_s = time.to_sec()
                self.waypoints_time[i] == time_s - self.time_start
        rospy.loginfo(f"[{self._name}] updates waypoints_time, done!")

        # Update the oveall time since agent left START_POINT
        time = rospy.get_rostime()
        tim_s = time.to_sec()
        self.overall_time = tim_s - self.time_start
        rospy.loginfo(f"[{self._name}] updates overall_time, done!")


    def move_to_wp(self, wp_x, wp_y, wp_ori):
        pub = rospy.Publisher('move_base_simple/goal', PoseStamped, queue_size=10)
        pub.publish(PoseStamped(header=Header(time=rospy.Time.now(), frame_id='map'), pose=Pose(position=Point(x=wp_x, y=wp_y, z=0), orientation=Quaternion(*wp_ori))))


    def move_base_action_client(self, goalx=0,goaly=0,goaltheta=0):
        rospy.loginfo(f"[{self._name}] starts moving to the next waypoint!")

        client=actionlib.SimpleActionClient('move_base',move_base_msgs.msg.MoveBaseAction)
        client.wait_for_server(rospy.Duration(20))
        goal=move_base_msgs.msg.MoveBaseGoal()
        goal.target_pose.header.frame_id='map'
        goal.target_pose.header.stamp=rospy.Time.now()
        goal.target_pose.pose.position.x=goalx
        goal.target_pose.pose.position.y=goaly
        goal.target_pose.pose.position.z=0.0
        q_angle = quaternion_from_euler(0.0,0.0,goaltheta)
        q = geometry_msgs.msg.Quaternion(*q_angle)
        goal.target_pose.pose.orientation=q
        rospy.loginfo(f"[{self._name}] sending next wayppoint position {goalx}:{goaly}")
        client.send_goal(goal)
        client.wait_for_result()
        res = client.get_result()
        rospy.loginfo(f"RESULT: {res}")
        return


    def teleport(self, x, y):
        rospy.loginfo(f"Teleport [{self._name}] robot to the starting point")

        state_msg = ModelState()
        state_msg.model_name = "mir"
        state_msg.pose.position.x = x
        state_msg.pose.position.y = y

        rospy.wait_for_service('/gazebo/set_model_state')
        set_state = rospy.ServiceProxy(
            '/gazebo/set_model_state', SetModelState)
        resp = set_state(state_msg )
        rospy.loginfo(f'[{self._name}] is teleported to ({x},{y})')

        """
        pose_with_stamp_msg = PoseWithCovarianceStamped()
        pose_with_stamp_msg.pose.pose.position.x = x
        pose_with_stamp_msg.pose.pose.position.y = y
        rospy.wait_for_service('/initialpose')
        pose_with_stamp = rospy.ServiceProxy(
            '/initialpose', PoseWithCovarianceStamped)
        resp = pose_with_stamp(pose_with_stamp_msg )
        """


    def _get_robot_position(self):
        rospy.wait_for_service('/gazebo/get_model_state')
        get_state = rospy.ServiceProxy(
            '/gazebo/get_model_state', GetModelState)
        resp = get_state(model_name = "mir")
        self.mir100_pos_x = resp.pose.position.x
        self.mir100_pos_y = resp.pose.position.y
        rospy.loginfo(f"[{self._name}] get robot current position: {self.mir100_pos_x} {self.mir100_pos_y}!")
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
    def _get_reward(self):
        """
        agent gets a reward of -1 for each time step
        
        If a waypoint is delivered in that timestep, 
        it gets a positive reward inversely proportional to the time taken to deliver.
        
        If all the waypoints are delivered and the agent is back to the start point, 
        it gets an additional reward inversely proportional to time since start of episode.
        """
        common_reward = np.sum(np.asarray(self.waypoints_status) * self.max_time / (np.asarray(self.waypoints_time) + 0.0001)) - self.overall_time
        
        return common_reward


#----------------------------------------------------------------------------------------#

def run_episode(env,agent,verbose = 1):
    
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
        rospy.loginfo(f"[mir100_rl_1] picks an action <{a}>!")
        
        # Take the action, and get the reward from environment
        s_next,r,done,info = env.step(a)

        # Print out: next waypoint to go, reward of current step, and whether this episode done or not
        if verbose: 
            print(s_next,r,done,info)
        
        # For each action, use the Bellman equation to update our knowledge in the existing Q-table
        agent.train(s,a,r,s_next)
        
        # Update the caches
        episode_reward += r # The total reward of each episode
        s = s_next
        rospy.loginfo(f"[mir100_rl_1]'s step <{i}> is done!")
        
        # If the episode is terminated
        i += 1
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



def run_num_episodes(env,agent,num_episodes=500):
    # Store the rewards
    rewards = []
    overall_times = []

    # Experience replay
    for i in range(num_episodes):

        # Run the episode
        rospy.loginfo(f"[mir100_rl_1] episode <{i}>!")
        env,agent,episode_reward = run_episode(env,agent,verbose = 0)

        overall_times.append(env.overall_time)
        rewards.append(episode_reward)
        
    # Show rewards and overall time
    fig, ax = plt.subplots(2, 1, figsize=(15,7.5))
    ax[0].plot(rewards)
    ax[1].plot(overall_times)
    ax[0].set_title("Rewards over Training")
    ax[1].set_title("Overall Time Taken over Training")
    ax[0].set_xlabel("Episode")
    ax[0].set_ylabel("Rewards")
    ax[1].set_xlabel("Episode")
    ax[1].set_ylabel("Overall Time (Unit: second)")
    fig.tight_layout()
    plt.savefig('../Results_Plot/Rewards_and_OverallTime.png', dpi = 120)
    plt.show()

    return env,agent


#----------------------------------------------------------------------------------------#


if __name__ == u'__main__':
    rospy.init_node('mir100_rl_1', anonymous=True)

    env = DynamicObstacleNavigationMir100Sim()

    agent = DeliveryQAgent(env.state_space,env.action_space,epsilon = 1.0,epsilon_min = 0.01,epsilon_decay = 0.999,gamma = 0.95,lr = 0.8)

    run_num_episodes(env,agent,num_episodes = 200)
