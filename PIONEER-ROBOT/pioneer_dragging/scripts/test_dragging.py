#! /usr/bin/env python3
import os
import time
import torch
import rospy
import rospkg
import random
import numpy as np

# import matplotlib
# matplotlib.use('Agg')
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

        # self.style_plot = random.choice(plt.style.available)
        self.style_plot = 'fast'

        plt.style.use(self.style_plot)
        plt.ion()

        ###########
        # Figure 1 - Rewards
        self.fig1, self.ax1 = self.create_figure(figure_n=1, title='Rewards - (Mode: Testing)',\
                                            x_label='Episode', y_label='Reward')
        self.ax2  = self.ax1.twinx()
        self.ax2.set_ylabel('Epsilon', color=self.color2)
        self.ax2.tick_params(axis='y', labelcolor=self.color2)

        ###########
        # Figure 2 - Error
        self.fig2, self.ax3 = self.create_figure(figure_n=2, title='Error Distance - (Mode: Testing)',\
                                            x_label='Episode', y_label='Meter')


    def moving_average(self, x, w):
        return np.convolve(x, np.ones(w), 'valid') / w

    def create_figure(self, figure_n, title, x_label, y_label):
        fig = plt.figure(figure_n)
        ax  = fig.add_subplot(1,1,1)

        ax.set_title(title)
        ax.set_xlabel(x_label)
        ax.set_ylabel(y_label)
        # ax.set_ylabel(y_label, color=self.color1)
        # ax.tick_params(axis='y', labelcolor=self.color1)
        return fig, ax

    def clear_axis(self, axes):
        for ax in axes:
            if ax is not None:
                ax.cla()

    def legend_axis(self, axes):
        for ax in axes:
            if ax is not None:
                ax.legend()

    def close_fig(self, figs):
        for fig in figs:
            if fig is not None:
                plt.close(fig=fig)

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

            ###########
            # Figure 3 - CoM
            fig3, ax4 = self.create_figure(figure_n=3, title='CoM X',\
                                            x_label='Step', y_label='Offset Value')
            def_cob_x  = rospy.get_param('/cob_x')
            list_cob_x = [ def_cob_x ]

            # ###########
            # # Figure 4 - IMU
            fig4, ax5 = self.create_figure(figure_n=4, title='IMU',\
                                            x_label='Step', y_label='Degree')
            imu_pitch = []
            imu_roll  = []

            # ###########
            # # Figure 5 - F/T Sensor
            fig5, ax6 = self.create_figure(figure_n=5, title='F/T Sensor',\
                                            x_label='Step', y_label='Binary')
            l_foot = []
            r_foot = []
            
            while not rospy.is_shutdown():
                steps += 1
                action = self.dqn.test_action(state)
                rospy.loginfo('[RL] action: {}'.format(action))

                next_state, reward, done, cob_x = self.env.step(action)
                cumulated_reward += reward

                list_cob_x.append(cob_x)
                imu_pitch. append(next_state[0])
                imu_roll.  append(next_state[1])
                l_foot.    append(next_state[2])
                r_foot.    append(next_state[3])

                # self.clear_axis([ax4, ax5, ax6])
                # ax4.plot(list_cob_x, '-o', color=self.color1, label='offset')
                # ax5.plot(imu_pitch,  '-o', color=self.color2, label='pitch')
                # ax5.plot(imu_roll,   '-o', color=self.color3, label='roll')
                # ax6.plot(l_foot,     '-o', color=self.color4, label='left_foot')
                # ax6.plot(r_foot,     '-o', color=self.color1, label='right_foot')
                # self.legend_axis([ax4, ax5, ax6])
                # plt.draw()
                # plt.pause(0.1)

                if not done:
                    state = next_state
                else:
                    break
          
            ################
            # Plotting
            ax4.plot(list_cob_x, '-o', color=self.color1, label='offset')
            ax5.plot(imu_pitch,  '-o', color=self.color2, label='pitch')
            ax5.plot(imu_roll,   '-o', color=self.color3, label='roll')
            ax6.plot(l_foot,     '-o', color=self.color4, label='left_foot')
            ax6.plot(r_foot,     '-o', color=self.color1, label='right_foot')
            self.legend_axis([ax4, ax5, ax6])
            error_dist = self.env.calc_dist()

            self.plot_result(i_episode, cumulated_reward, error_dist)
            input('Press enter')
            self.close_fig([fig3, fig4, fig5])

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