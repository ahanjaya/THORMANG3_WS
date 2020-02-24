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

from glob import glob  
from time import sleep
from environment import Env
from std_msgs.msg import Bool
from deepQlearn import DQN, ExperienceReplay

class Training:
    def __init__(self):
        self.n_episode = []
        self.n_epsilon = []
        self.n_dist    = []
        self.avg_err   = []
        self.logging_data = []

        # Parameters
        self.n_episodes      = rospy.get_param("/n_episodes") 
        self.n_step          = rospy.get_param("/n_steps") 
        self.mode_action     = rospy.get_param('/mode_action')
        self.mem_size        = rospy.get_param('/mem_size')
        self.batch_size      = rospy.get_param('/batch_size')
        self.mode_optimize   = rospy.get_param('/mode_optimize')
        self.avg_err_fre     = rospy.get_param('/avg_err_fre')
        self.save_fre        = rospy.get_param("/save_fre")
        self.load_checkpoint = rospy.get_param("/load_checkpoint")

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

        ###########
        # Figure 1 - Rewards
        self.fig1 = plt.figure(1)
        # fig = plt.figure(figsize=(12,5))
        self.ax1  = self.fig1.add_subplot(1,1,1)
        self.ax2  = self.ax1.twinx()

        title_1 = 'Rewards - (Mode: Training)'
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

        title_2 = 'Error Distance - (Mode: Training)'
        self.ax3.set_title(title_2)
        self.ax3.set_xlabel('Episode')
        self.ax3.set_ylabel('Meter')

        self.init_file()

    def moving_average(self, x, w):
        return np.convolve(x, np.ones(w), 'valid') / w

    def init_file(self):
        rospack   = rospkg.RosPack()
        data_path = rospack.get_path("pioneer_dragging") + "/data"
        username  = getpass.getuser()
        # n_folder  = len(os.walk(data_path).__next__()[1])
        n_folder  = glob( "{}/{}*".format(data_path, username) )
        n_folder  = len(n_folder) + 1

        if self.load_checkpoint:
            n_folder -= 1

        self.data_path = "{}/{}-{}".format(data_path, username, n_folder)
        if not os.path.exists(self.data_path):
            os.mkdir(self.data_path)

        # config file
        if not self.load_checkpoint:
            config_path = rospack.get_path("pioneer_dragging") + "/config/dragging_params.yaml"
            config_log  = '{}/{}-params.yaml'.format(self.data_path, n_folder)
            os.system('cp {} {}'.format(config_path, config_log))

            plot_style = {'plot_style': self.style_plot}
            with open(config_log,'r') as yamlfile:
                cur_yaml = yaml.safe_load(yamlfile) # Note the safe_load
                cur_yaml.update(plot_style)

            if cur_yaml:
                with open(config_log,'w') as yamlfile:
                    yaml.safe_dump(cur_yaml, yamlfile) # Also note the safe_dump

        # history file
        self.history_log     = '{}/{}-log.txt'.format(self.data_path, n_folder)
        
        # model file
        self.dqn.file_models = '{}/{}-pytorch-RL.tar'.format(self.data_path, n_folder)

        # memory file
        self.memory.file_mem = '{}/{}-memory.data'.format(self.data_path, n_folder)

        # figures file
        self.figure1 = '{}/{}-Rewards(Training).png'.format(self.data_path, n_folder)
        self.figure2 = '{}/{}-Error(Training).png'.format(self.data_path, n_folder)

    def plot_result(self, i_episode, cumulated_reward, epsilon, error_dist, loaded=False):
        ### Figure 1
        # plot bar (cumulated reward)
        self.ax1.bar(i_episode, cumulated_reward, color=self.color1)

        # plot line (epsilon decay )
        if loaded:
            self.ax2.plot(i_episode ,epsilon, color=self.color2)

            self.n_episode = i_episode.tolist()
            self.n_epsilon = epsilon.tolist()
            self.n_dist    = error_dist.tolist()
        else:
            self.n_episode.append(i_episode)
            self.n_epsilon.append(epsilon)
            self.ax2.plot(self.n_episode, self.n_epsilon, color=self.color2)

            self.n_dist.append(error_dist)

        ### Figure 2
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

        if self.load_checkpoint:
            self.memory.load()
            self.dqn.load_model()

            # history log loaded
            self.logging_data = [line.rstrip('\n') for line in open(self.history_log)]
            
            hist_data = pd.read_csv(self.history_log, sep=",")
            i_episode        = hist_data['i_episode']
            cumulated_reward = hist_data['cumulated_reward']
            epsilon          = hist_data['epsilon']
            error_dist       = hist_data['error_dist']
    
            self.plot_result(i_episode, cumulated_reward, epsilon, error_dist, loaded=True)
            i_episode        = hist_data['i_episode'].iloc[-1] + 1
            self.dqn.epsilon = hist_data['epsilon'].iloc[-1]
            rospy.loginfo('[RL] Loaded checkpoint')
        else:
            i_episode = 0
            
        #########################################
        ###### Reinfrocement Training loop ######
        for i_episode in range (i_episode, self.n_episodes):
            state = self.env.reset(i_episode)
            cumulated_reward = 0

            steps     = 0
            step_time = time.time()

            while not rospy.is_shutdown():
                steps += 1
                action, epsilon = self.dqn.select_action(state, i_episode)
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
                        self.dqn.optimize_with_DQN(state_mem, action_mem, next_state_mem, reward_mem, done_mem)

                elif self.mode_optimize == 'dueling_dqn':
                    # with double DQN
                    if len(self.memory) > self.batch_size:
                        state_mem, action_mem, next_state_mem, reward_mem, done_mem = self.memory.sample(self.batch_size)
                        self.dqn.optimize_with_dueling_DQN(state_mem, action_mem, next_state_mem, reward_mem, done_mem)

                if not done:
                    state = next_state
                else:
                    break

            # DQN update param 
            self.dqn.update_param(i_episode)

            # Plotting
            error_dist = self.env.calc_dist()
            self.plot_result(i_episode, cumulated_reward, epsilon, error_dist)
           
            # Save Checkpoint
            temp_data = "{},{},{},{}".format(i_episode, cumulated_reward, epsilon, error_dist)
            self.logging_data.append(temp_data)
            
            if i_episode % self.save_fre == 0:
                rospy.loginfo('[RL] Save checkpoint: {}'.format(i_episode))
                
                self.dqn.save_model()   # save models
                self.memory.save()      # save replay memory

                # logging file
                with open(self.history_log, 'w') as f:
                    if not self.load_checkpoint:
                        f.write("i_episode,cumulated_reward,epsilon,error_dist\n")

                    for item in self.logging_data:
                        f.write("%s\n" % item)

                # save figures
                self.fig1.savefig(self.figure1, dpi=self.fig1.dpi)
                self.fig2.savefig(self.figure2, dpi=self.fig2.dpi)
                rospy.loginfo('[RL] Save figure1: {}'.format(self.figure1))
                rospy.loginfo('[RL] Save figure2: {}'.format(self.figure2))
            
            # Timing
            elapsed_time = time.time() - step_time
            total_time   = time.time() - start_time
            print('\n********')
            print("Elapsed time: {}".format( time.strftime("%H:%M:%S", time.gmtime(elapsed_time)) ))
            print("Total time: {}"  .format( time.strftime("%H:%M:%S", time.gmtime(total_time)) ))

        # Finish Training
        self.env.close()
        print()
        rospy.loginfo('[RL] Exit ...')

        total_time = time.time() - start_time
        print('\n*********************')
        print("Total time: ", time.strftime("%H:%M:%S", time.gmtime(total_time)))

        rospy.loginfo('[RL] Style plot: {}'.format(self.style_plot))
        plt.show(block=True)

if __name__ == "__main__":
    rospy.init_node('pioneer_dragging_RL') # init node
    training = Training()
    training.run()