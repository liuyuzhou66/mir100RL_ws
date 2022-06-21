#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import time
import rospy
import numpy as np
from pathlib import Path
from std_srvs.srv import Empty, EmptyRequest



BASE_PATH = Path(os.path.dirname(__file__))
results_path = BASE_PATH.parent.parent.parent / 'Results_Plot'
if not results_path.exists():
    results_path.mkdir()


# Load the Q table
Q = np.load(results_path / 'RL_Qtable.npy')

print("----type----")
print(type(Q))
print("----shape----")
print(Q.shape)
print("----data----")
print(Q)


class DynamicObstacleNavigationMir100Test:
    def __init__(self, **kwargs):
        fasdfasd

    def 



def run_episode(env,agent):
    
    rospy.loginfo("[mir100_rl_1] starts a new episode!")
    
    # The initial state of the robot is start_point_index
    s = 0

    # Max steps per episode (6 steps for 6 waypoints)
    max_step = 6
    
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
        s_next = a
        rospy.loginfo(f"for action ({a}), [mir100_rl_1]'s new state: {s_next}; reward: {r}; episode done: {done}; info: {info}!")
        
        # Update the caches
        s = s_next
        rospy.loginfo(f"[mir100_rl_1]'s step num.<{i}> is finished!")
        
        # If the episode is terminated
        i += 1
        if done:
            break
            
    return env,agent

class QAgent:
    def __init__(self,**kwargs):
        self.wp_memory = []

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

    run_episode(env,agent)

    pause_physics_client = rospy.ServiceProxy('/gazebo/pause_physics', Empty)
    pause_physics_client(EmptyRequest())