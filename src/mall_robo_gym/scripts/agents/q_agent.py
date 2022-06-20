#!/usr/bin/env python3
# -*- coding: utf-8 -*- 


"""--------------------------------------------------------------------
REINFORCEMENT LEARNING

Started on the 25/08/2017


theo.alves.da.costa@gmail.com
https://github.com/theolvs
------------------------------------------------------------------------
"""

import numpy as np
import rospy 

# from mall_robo_gym import utils
from .memory import Memory
from .base_agent import Agent

np.set_printoptions(linewidth=120)


class QAgent(Agent):
    def __init__(self,states_size,actions_size,epsilon = 1.0,epsilon_min = 0.01,epsilon_decay = 0.999,gamma = 0.95,lr = 0.8):
        self.states_size = states_size
        self.actions_size = actions_size
        self.exploration_rate = []
        self.gamma = gamma                  # Discounting rate
        self.lr = lr                        # Learning rate
        
        # Exploration parameters
        self.epsilon = epsilon              # Exploration rate
        self.epsilon_min = epsilon_min      # Minimum exploration rate
        self.epsilon_decay = epsilon_decay  # Exponential decay rate 
        self.exploration_rate.append(self.epsilon)

        # Initialize Q-table to 0
        self.Q = self.build_model(states_size,actions_size)


    def build_model(self,states_size,actions_size):
        Q = np.zeros([states_size,actions_size])
        return Q


    def train(self,s,a,r,s_next):
        # Bellman equation to update the Q value
        self.Q[s,a] = self.Q[s,a] + self.lr * (r + self.gamma*np.max(self.Q[s_next,a]) - self.Q[s,a])
        rospy.loginfo(f"[mir100_rl_1] updates Q table: \n{self.Q}")

        # Reduce epsilon (because we need less and less exploration)
        if self.epsilon > self.epsilon_min:
            # for steps in max_steps:
            #       epsilon = epsilon_decay^steps
            self.epsilon *= self.epsilon_decay
        self.exploration_rate.append(self.epsilon)


    def act(self,s):

        q = self.Q[s,:]

        if np.random.rand() > self.epsilon:
            a = np.argmax(q)
        else:
            a = np.random.randint(self.actions_size)

        return a
