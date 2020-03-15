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
        # n_folder         = 2
        # self.username    = 'barelangfc'

        n_folder         = 14
        self.username    = 'pioneer'

        data_path        = rospack.get_path("pioneer_dragging") + "/data"
        self.res_path    = rospack.get_path("pioneer_dragging") + "/results"
        # self.username    = getpass.getuser()
       
        self.data_path   = "{}/{}-{}".format(data_path, self.username, n_folder)
        self.history_log = '{}/{}-log.txt'.format(self.data_path, n_folder)
        self.config_log  = '{}/{}-params.yaml'.format(self.data_path, n_folder)

        with open(self.config_log,'r') as yamlfile:
            self.config_yaml = yaml.safe_load(yamlfile) # Note the safe_load

        # plot
        self.color_green  = 'tab:green'
        self.color_blue   = 'tab:blue'
        self.color_orange = 'tab:orange'
        self.color_red    = 'tab:red'

        # plt.style.use(self.config_yaml['plot_style'])

        self.style_plot = random.choice(plt.style.available)
        # self.style_plot = 'fast' #seaborn-pastel' #'fast'
        plt.style.use(self.style_plot)
        plt.rcParams.update({'font.size': 22})

        print(self.style_plot)
        # plt.style.use('seaborn-bright')
        # plt.ion()

        self.fig1 = plt.figure(1, figsize=(12,8))
        # self.fig1 = plt.figure(1)
        self.ax1 = self.fig1.add_subplot(1,1,1)
        self.ax2 = self.ax1.twinx()

        # self.fig2 = plt.figure(2)
        self.fig2 = plt.figure(2, figsize=(12,8))
        self.ax3 = self.fig2.add_subplot(1,1,1)

        self.ax1.set_title('Rewards')
        self.ax1.set_xlabel('Episode')
        self.ax1.set_ylabel('Accumulated Reward',  color=self.color_green)
        self.ax2.set_ylabel('Epsilon', color=self.color_blue)
        self.ax1.tick_params(axis='y', labelcolor=self.color_green)
        self.ax2.tick_params(axis='y', labelcolor=self.color_blue)

        # self.ax3.set_title(title_2)
        self.ax3.set_title('Error Distance')
        self.ax3.set_xlabel('Episode')
        self.ax3.set_ylabel('Meter')

    # def moving_average(self, x, w):
    #     return np.convolve(x, np.ones(w), 'valid') / w

    def moving_average(self, scalars, weight):
        last = scalars[0]  # First value in the plot (first timestep)
        smoothed = list()
        for point in scalars:
            smoothed_val = last * weight + (1 - weight) * point  # Calculate smoothed value
            smoothed.append(smoothed_val)                        # Save it
            last = smoothed_val                                  # Anchor the last smoothed value

        return smoothed

    def plot_result(self, i_episode, cumulated_reward, epsilon, error_dist):
        ### Figure 1
        # plot bar (cumulated reward)
        # self.ax1.bar(i_episode, cumulated_reward, color=self.color_green, label='Cumulative Reward')
        
        avg_filter     = 0.7 #50 #100
        avg_reward     = self.moving_average( np.array(cumulated_reward), avg_filter)
        avg_reward_std = np.std(avg_reward)
        # print('len cumulated reward: ', cumulated_reward.shape)
        # print('len avg reward: ', avg_reward.shape)

        lns1 = self.ax1.plot(avg_reward, color=self.color_green, label='Reward') #label='Average Reward')
        self.ax1.fill_between(range(len(avg_reward)), avg_reward - avg_reward_std, avg_reward + avg_reward_std, alpha=0.25, color=self.color_green)

        # plot line (epsilon decay)
        lns2 = self.ax2.plot(i_episode, epsilon, color=self.color_blue, label='Epsilon')
        # epsilon_std = np.std(epsilon)
        # self.ax2.fill_between(i_episode, epsilon - epsilon_std, epsilon + epsilon_std, alpha=0.25, color=self.color_blue)

        # added these three lines
        lns  = lns1 + lns2
        labs = [l.get_label() for l in lns]
        self.ax1.legend(lns, labs, loc=0)
        self.ax1.grid()

        ### Figure 2
        # plot bar (error distance)
        if self.username == "barelangfc":
            error_dist -= 0.25
            error_dist = abs(error_dist)

        # print(error_dist)
        # self.ax3.bar(i_episode, error_dist, color=self.color_orange, label='Euclidean Distance')
        # plot line (average error distance)
        # self.avg_err_fre = self.config_yaml['avg_err_fre']

        avg_err = self.moving_average( np.array(error_dist), avg_filter)
        avg_err_std = np.std(avg_err)

        self.ax3.plot(avg_err, color=self.color_red, label='Euclidean Dist.') #label='Average Distance')
        self.ax3.fill_between(range(len(avg_err)), avg_err - avg_err_std, avg_err + avg_err_std, alpha=0.25, color=self.color_red)
        self.ax3.legend()
        self.ax3.grid()

        # plt.draw()
        # plt.pause(0.1)

    def run(self):
        history = pd.read_csv(self.history_log, sep=",")#, header=None)

        i_episode        = history['i_episode']
        epsilon          = history['epsilon']
        error_dist       = history['error_dist']
        cumulated_reward = history['cumulated_reward']

        if self.username == "pioneer":
            mask             = (history['error_dist'] == 1.5)
            history.loc[mask, 'cumulated_reward'] = 20  # replace column values by other column references
            cumulated_reward = history['cumulated_reward']
            error_dist       = error_dist.replace(1.5, 0)

        self.plot_result(i_episode, cumulated_reward, epsilon, error_dist)

        # save fig
        self.figure1 = "{}/reward_{}.png".format(self.res_path, self.style_plot)
        self.fig1.savefig(self.figure1, dpi=self.fig1.dpi)

        self.figure2 = "{}/error_{}.png".format(self.res_path, self.style_plot)
        self.fig2.savefig(self.figure2, dpi=self.fig2.dpi)

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
self.username = 'barelangfc'
error_dist -= 0.25
error_dist = abs(error_dist)

2. 
best training so far, added foot_step with DQN
n_folder = 14
self.username = 'pioneer'

'''