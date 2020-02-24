#! /usr/bin/env python3
import os
import yaml
import time
import torch
import rospy
import rospkg
import random
import getpass
import threading
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

from time import sleep
from environment import Env
from std_msgs.msg import Bool
from deepQlearn import DQN, ExperienceReplay

class Testing:
    def __init__(self):
        self.n_episode = []
        self.n_epsilon = []
        self.n_dist    = []
        self.avg_err   = []

        # Parameters
        self.n_episodes  = rospy.get_param("/n_episodes") 
        self.avg_err_fre = rospy.get_param('/avg_err_fre')

        # create environment
        self.env       = Env()
        self.n_states  = self.env.observation_space
        self.n_actions = self.env.action_space.n

        # create Deep Q-Network
        self.dqn       = DQN(self.n_states, self.n_actions)

        # load DQN weight
        rospack        = rospkg.RosPack()
        data_path      = rospack.get_path("pioneer_dragging") + "/data"
        username       = rospy.get_param("/username") 
        n_folder       = rospy.get_param("/n_folder") 
        self.dqn.file_models = "{0}/{1}-{2}/{2}-pytorch-RL.tar".format(data_path, username, n_folder)
        self.dqn.load_model()

        # plot
        self.color1    = 'tab:green'
        self.color2    = 'tab:blue'
        self.color3    = 'tab:orange'
        self.color4    = 'tab:red'

        self.style_plot = random.choice(plt.style.available)
        plt.style.use(self.style_plot)
        plt.ion()

        ###########
        # Figure 1 - Rewards
        self.fig1 = plt.figure(1)
        self.ax1  = self.fig1.add_subplot(1,1,1)
        self.ax2  = self.ax1.twinx()

        title_1 = 'Rewards - (Mode: Testing)'
        self.ax1.set_title(title_1)
        self.ax1.set_xlabel('Episode')
        self.ax1.set_ylabel('Reward',  color=self.color1)
        self.ax2.set_ylabel('Epsilon', color=self.color2)
        self.ax1.tick_params(axis='y', labelcolor=self.color1)
        self.ax2.tick_params(axis='y', labelcolor=self.color2)

        ###########
        # Figure 2 - Error
        self.fig2 = plt.figure(2)
        self.ax3  = self.fig2.add_subplot(1,1,1)

        title_2 = 'Error Distance - (Mode: Testing)'
        self.ax3.set_title(title_2)
        self.ax3.set_xlabel('Episode')
        self.ax3.set_ylabel('Meter')

    def moving_average(self, x, w):
        return np.convolve(x, np.ones(w), 'valid') / w

    def plot_result(self, i_episode, cumulated_reward, error_dist):
        ### Figure 1
        # plot bar (cumulated reward)
        self.ax1.bar(i_episode, cumulated_reward, color=self.color1)

        ### Figure 2
        # plot bar (error distance)
        self.ax3.bar(i_episode, error_dist, color=self.color3)
        self.n_dist.append(error_dist)

        # plot line (average error distance)
        if len(self.n_dist) % self.avg_err_fre == 0:
            avg_err = self.moving_average( np.array(self.n_dist), self.avg_err_fre)
            self.ax3.plot(avg_err, color=self.color4)

        plt.draw()
        plt.pause(0.1)

    def run(self):
        start_time = time.time()

        #########################################
        ###### Reinfrocement Training loop ######
        for i_episode in range (self.n_episodes):
            state = self.env.reset(i_episode)
            cumulated_reward = 0

            steps     = 0
            step_time = time.time()

            while not rospy.is_shutdown():
                steps += 1
                action = self.dqn.test_action(state)

                # action = env.action_space.sample()
                rospy.loginfo('[RL] action: {}'.format(action))

                next_state, reward, done, _ = self.env.step(action)
                cumulated_reward += reward

                if not done:
                    state = next_state
                else:
                    break
            
            ################
            # Plotting
            error_dist = self.env.calc_dist()
            self.plot_result(i_episode, cumulated_reward, error_dist)

            ################
            # Timing
            elapsed_time = time.time() - step_time
            total_time   = time.time() - start_time
            print('\n********')
            print("Elapsed time: {}".format( time.strftime("%H:%M:%S", time.gmtime(elapsed_time)) ))
            print("Total time: {}"  .format( time.strftime("%H:%M:%S", time.gmtime(total_time)) ))

        ################
        # Finish Testing

        self.env.close()
        print()
        rospy.loginfo('[RL] Exit ...')

        total_time = time.time() - start_time
        print('\n*********************')
        print("Total time: ", time.strftime("%H:%M:%S", time.gmtime(total_time)))

        rospy.loginfo('[RL] Style plot: {}'.format(self.style_plot))
        plt.show(block=True)

if __name__ == "__main__":
    rospy.init_node('pioneer_dragging_test_RL') # init node
    testing = Testing()
    testing.run()