#! /usr/bin/env python3

import torch
import rospy
import threading
import numpy as np

import environment

rospy.init_node('pioneer_dragging')

env = environment.Env()
env.initial()

#############
# Parameter #
num_episodes = 3
num_step     = 100

for i_episode in range (num_episodes):
    print('episode: ',i_episode)
    state = env.reset()

    while not rospy.is_shutdown():

        action = None
        new_state, reward, done, info = env.step(action)
        if done:
            break

env.close()
rospy.loginfo('[RL] Exit ...')