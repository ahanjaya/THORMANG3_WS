#! /usr/bin/env python3

import yaml
import rospy
import rospkg
import random
import getpass
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

class Utility:
    def __init__(self):
        rospack          = rospkg.RosPack()
        n_folder         = 16
        data_path        = rospack.get_path("pioneer_dragging") + "/data"
        # username         = getpass.getuser()
        username         = 'pioneer' #'barelangfc'
       
        self.data_path   = "{}/{}-{}".format(data_path, username, n_folder)
        self.history_log = '{}/{}-log.txt'.format(self.data_path, n_folder)
        self.config_log  = '{}/{}-params.yaml'.format(self.data_path, n_folder)

        with open(self.config_log,'r') as yamlfile:
            self.config_yaml = yaml.safe_load(yamlfile) # Note the safe_load

        # plot
        self.color1    = 'tab:green'
        self.color2    = 'tab:blue'
        self.color3    = 'tab:orange'
        self.color4    = 'tab:red'

        # plt.style.use(self.config_yaml['plot_style'])

        self.style_plot = random.choice(plt.style.available)
        # self.style_plot = 'seaborn-pastel'
        self.style_plot = 'fast'
        plt.style.use(self.style_plot)
        plt.rcParams.update({'font.size': 22})

        print(self.style_plot)
        # plt.style.use('seaborn-bright')
        # plt.ion()

        fig1 = plt.figure(1, figsize=(12,8))
        # fig1 = plt.figure(1)
        self.ax1 = fig1.add_subplot(1,1,1)
        self.ax2 = self.ax1.twinx()

        # fig2 = plt.figure(2)
        fig2 = plt.figure(2, figsize=(12,8))
        self.ax3 = fig2.add_subplot(1,1,1)

        # if self.config_yaml['testing']:
        #     mode = 'Testing'
        # else:
        #     mode = 'Training'

        self.mode_action = self.config_yaml['mode_action']
        # title_1 = 'Rewards - {} (Mode: {})'.format(self.mode_action, mode)
        # title_2 = 'Euclidean Distance - {} (Mode: {})'.format(self.mode_action, mode)

        # self.ax1.set_title(title_1)
        self.ax1.set_title('Maximized Rewards')
        self.ax1.set_xlabel('Episode')
        self.ax1.set_ylabel('Reward',  color=self.color1)
        self.ax2.set_ylabel('Epsilon', color=self.color2)
        self.ax1.tick_params(axis='y', labelcolor=self.color1)
        self.ax2.tick_params(axis='y', labelcolor=self.color2)

        # self.ax3.set_title(title_2)
        self.ax3.set_title('Error Distance')
        self.ax3.set_xlabel('Episode')
        self.ax3.set_ylabel('Meter')

    def moving_average(self, x, w):
        return np.convolve(x, np.ones(w), 'valid') / w

    def plot_result(self, i_episode, cumulated_reward, epsilon, error_dist):
        ### Figure 1
        # plot bar (cumulated reward)
        self.ax1.bar(i_episode, cumulated_reward, color=self.color1, label='Cumulative Reward')
        
        avg_filter = 100

        avg_reward = self.moving_average( np.array(cumulated_reward), avg_filter)
        self.ax1.plot(avg_reward, color=self.color4, label='Average Reward')

        # plot line (epsilon decay )
        self.ax2.plot(i_episode ,epsilon, color=self.color2, label='Epsilon')

        plt.legend()
        self.ax1.legend()
        self.ax2.legend()

        ### Figure 2
        # plot bar (error distance)

        # error_dist -= 0.25
        # error_dist = abs(error_dist)

        # print(error_dist)

        self.ax3.bar(i_episode, error_dist, color=self.color3, label='Euclidean Distance')

        # plot line (average error distance)
        # self.avg_err_fre = self.config_yaml['avg_err_fre']

        avg_err = self.moving_average( np.array(error_dist), avg_filter)
        self.ax3.plot(avg_err, color=self.color4, label='Average Distance')
        self.ax3.legend()

        # plt.draw()
        # plt.pause(0.1)

    def run(self):
        history = pd.read_csv(self.history_log, sep=",")#, header=None)

        i_episode        = history['i_episode']
        epsilon          = history['epsilon']
        error_dist       = history['error_dist']

        mask             = (history['error_dist'] == 1.5)
        history.loc[mask, 'cumulated_reward'] = 20  # replace column values by other column references

        cumulated_reward = history['cumulated_reward']
        error_dist       = error_dist.replace(1.5, 0)

        self.plot_result(i_episode, cumulated_reward, epsilon, error_dist)

        plt.show(block=False)
        input('Close all ...')
        plt.close('all')

if __name__ == "__main__":
    rospy.init_node('pioneer_drag_utils')
    drag_util = Utility()
    drag_util.run()


'''
1.
seminar slide / suitcase
n_folder = 2
username = 'barelangfc'
error_dist -= 0.25
error_dist = abs(error_dist)

2. 
best training so far, added foot_step with DQN
n_folder = 14
username = 'pioneer'

'''