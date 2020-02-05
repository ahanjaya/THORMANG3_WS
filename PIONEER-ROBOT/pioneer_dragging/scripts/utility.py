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
        n_folder         = 1
        data_path        = rospack.get_path("pioneer_dragging") + "/data"
        username         = getpass.getuser()
       
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

        plt.style.use(self.config_yaml['plot_style'])
        plt.ion()

        # fig = plt.figure(figsize=(12,5))
        fig1 = plt.figure(1)
        self.ax1 = fig1.add_subplot(1,1,1)
        self.ax2 = self.ax1.twinx()

        fig2 = plt.figure(2)
        self.ax3 = fig2.add_subplot(1,1,1)

        if self.config_yaml['testing']:
            mode = 'Testing'
        else:
            mode = 'Training'

        self.mode_action = self.config_yaml['mode_action']
        title_1 = 'Rewards - {} (Mode: {})'.format(self.mode_action, mode)
        title_2 = 'Euclidean Distance - {} (Mode: {})'.format(self.mode_action, mode)

        self.ax1.set_title(title_1)
        self.ax1.set_xlabel('Episode')
        self.ax1.set_ylabel('Reward',  color=self.color1)
        self.ax2.set_ylabel('Epsilon', color=self.color2)
        self.ax1.tick_params(axis='y', labelcolor=self.color1)
        self.ax2.tick_params(axis='y', labelcolor=self.color2)

        self.ax3.set_title(title_2)
        self.ax3.set_xlabel('Episode')
        self.ax3.set_ylabel('Meter')

    def moving_average(self, x, w):
        return np.convolve(x, np.ones(w), 'valid') / w

    def plot_result(self, i_episode, cumulated_reward, epsilon, error_dist):
        ### Figure 1
        # plot bar (cumulated reward)
        self.ax1.bar(i_episode, cumulated_reward, color=self.color1)

        # plot line (epsilon decay )
        self.ax2.plot(i_episode ,epsilon, color=self.color2)

        ### Figure 2
        # plot bar (error distance)
        self.ax3.bar(i_episode, error_dist, color=self.color3)

        # plot line (average error distance)

        # self.avg_err_fre = self.config_yaml['avg_err_fre']
        self.avg_err_fre = 4

        avg_err = self.moving_average( np.array(error_dist), self.avg_err_fre)
        self.ax3.plot(avg_err, color=self.color4)

        plt.draw()
        plt.pause(0.1)

    def run(self):
        history_data = pd.read_csv(self.history_log, sep=",")#, header=None)

        i_episode        = history_data['i_episode']
        cumulated_reward = history_data['cumulated_reward']
        epsilon          = history_data['epsilon']
        error_dist       = history_data['error_dist']

        self.plot_result(i_episode, cumulated_reward, epsilon, error_dist)

        plt.show(block=False)
        input('Close all ...')
        plt.close('all')

if __name__ == "__main__":
    rospy.init_node('pioneer_drag_utils')
    drag_util = Utility()
    drag_util.run()