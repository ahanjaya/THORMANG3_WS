#! /usr/bin/env python3
import torch
import time
import rospy
import random
import threading
import numpy as np
import matplotlib.pyplot as plt

from time import sleep
from environment import Env
from std_msgs.msg import Bool
from deepQlearn import DQN, ExperienceReplay

class Mains:
    def __init__(self):
        self.n_episode = []
        self.n_epsilon = []
        self.n_dist    = []
        self.avg_err   = []

        # Parameters
        self.n_episodes    = rospy.get_param("/n_episodes") 
        self.n_step        = rospy.get_param("/n_steps") 
        self.plotting      = rospy.get_param('/plotting')
        self.mode_action   = rospy.get_param('/mode_action')
        self.mem_size      = rospy.get_param('/mem_size')
        self.batch_size    = rospy.get_param('/batch_size')
        self.mode_optimize = rospy.get_param('/mode_optimize')
        self.testing       = rospy.get_param('/testing')
        self.avg_err_fre   = rospy.get_param('/avg_err_fre')

        # create environment
        self.env       = Env()
        self.n_states  = self.env.observation_space
        self.n_actions = self.env.action_space.n

        # create Deep Q-Network
        self.dqn       = DQN(self.n_states, self.n_actions)
        self.memory    = ExperienceReplay(self.mem_size)

        # plot
        self.color1    = 'tab:green'
        self.color2    = 'tab:blue'
        self.color3    = 'tab:orange'
        self.color4    = 'tab:red'

        self.style_plot = random.choice(plt.style.available)
        plt.style.use(self.style_plot)
        plt.ion()

        # fig = plt.figure(figsize=(12,5))
        fig1 = plt.figure(1)
        self.ax1 = fig1.add_subplot(1,1,1)
        self.ax2 = self.ax1.twinx()

        fig2 = plt.figure(2)
        self.ax3 = fig2.add_subplot(1,1,1)

        if self.testing:
            mode = 'Testing'
        else:
            mode = 'Training'

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

    def plot_result(self, i_episode, cumulated_reward, epsilon):
        ### Figure 1
        # plot bar (cumulated reward)
        self.ax1.bar(i_episode, cumulated_reward, color=self.color1)

        # plot line (epsilon decay )
        self.n_episode.append(i_episode)
        self.n_epsilon.append(epsilon)
        self.ax2.plot(self.n_episode, self.n_epsilon, color=self.color2)

        ### Figure 2
        error_dist = self.env.calc_dist()
        self.n_dist.append(error_dist)

        # plot bar (error distance)
        self.ax3.bar(i_episode, error_dist, color=self.color3)

        # window_err = np.array(self.n_dist)
        # window_err = np.mean(window_err)
        # self.avg_err.append(window_err)
        # self.ax3.plot(self.n_episode, self.avg_err, color=self.color4)

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

            steps = 0
            step_time = time.time()

            while not rospy.is_shutdown():
                steps += 1
                action, epsilon = self.dqn.select_action(state)
                # print('num_steps: {}, epsilon: {}, steps_done: {}'.format(steps, epsilon, dqn.steps_done))

                # action = env.action_space.sample()
                rospy.loginfo('[RL] action: {}'.format(action))

                next_state, reward, done, info = self.env.step(action)
                self.memory.push(state, action, next_state, reward, done)
                cumulated_reward += reward

                ################################
                ######### optimize #############

                if self.mode_optimize == 'normal_dqn':
                    # without experience replay memory
                    self.dqn.optimize(state, action, next_state, reward, done)

                elif self.mode_optimize == 'dqn_replay_memory':
                    # with experience replay memory
                    if len(self.memory) > self.batch_size:
                        state_mem, action_mem, next_state_mem, reward_mem, done_mem = self.memory.sample(self.batch_size)
                        self.dqn.optimize_with_replay_memory(state_mem, action_mem, next_state_mem, reward_mem, done_mem)

                elif self.mode_optimize == 'dqn_taget_net':
                    # with experience target net
                    if len(self.memory) > self.batch_size:
                        state_mem, action_mem, next_state_mem, reward_mem, done_mem = self.memory.sample(self.batch_size)
                        self.dqn.optimize_with_target_net(state_mem, action_mem, next_state_mem, reward_mem, done_mem)

                if not done:
                    state = next_state
                else:
                    break

            if self.plotting:
                self.plot_result(i_episode, cumulated_reward, epsilon)

            elapsed_time = time.time() - step_time
            total_time   = time.time() - start_time
            print('\n********')
            print("Elapsed time: {}".format( time.strftime("%H:%M:%S", time.gmtime(elapsed_time)) ))
            print("Total time: {}"  .format( time.strftime("%H:%M:%S", time.gmtime(total_time)) ))

        self.env.close()
        print()
        rospy.loginfo('[RL] Exit ...')

        total_time = time.time() - start_time
        print('\n*********************')
        print("Total time: ", time.strftime("%H:%M:%S", time.gmtime(total_time)))

        if self.plotting:
            rospy.loginfo('[RL] Style plot: {}'.format(self.style_plot))
            plt.show(block=True)


if __name__ == "__main__":
    rospy.init_node('pioneer_RL_dragging') # init node

    main_drag = Mains()
    main_drag.run()