#! /usr/bin/env python3
import os
import time
import torch
import rospy
import rospkg
import random
import pickle
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
        self.color_green  = 'tab:green'
        self.color_blue   = 'tab:blue'
        self.color_orange = 'tab:orange'
        self.color_red    = 'tab:red'

        # self.style_plot = random.choice(plt.style.available)
        self.style_plot = 'seaborn-deep'#'fast'

        plt.style.use(self.style_plot)
        plt.rcParams.update({'font.size': 22})
        plt.ion()

        # pickle file name
        self.result_path = rospack.get_path("pioneer_dragging") + "/results"

        self.dis_plot_rewards = True
        if not self.dis_plot_rewards:
            ###########
            # Figure 1 - Rewards
            self.fig1, self.ax1 = self.create_figure(figure_n=1, title='Rewards - (Mode: Testing)',\
                                                x_label='Episode', y_label='Reward')
            self.ax2  = self.ax1.twinx()
            self.ax2.set_ylabel('Epsilon', color=self.color_blue)
            self.ax2.tick_params(axis='y', labelcolor=self.color_blue)

            ###########
            # Figure 2 - Error
            self.fig2, self.ax3 = self.create_figure(figure_n=2, title='Error Distance - (Mode: Testing)',\
                                                x_label='Episode', y_label='Meter')


    def moving_average(self, x, w):
        return np.convolve(x, np.ones(w), 'valid') / w

    def create_figure(self, figure_n, title, x_label, y_label):
        fig = plt.figure(figure_n, figsize=(12,8))
        ax  = fig.add_subplot(1,1,1)

        # ax.set_title(title)
        ax.set_xlabel(x_label)
        ax.set_ylabel(y_label)
        # ax.set_ylabel(y_label, color=self.color_green)
        # ax.tick_params(axis='y', labelcolor=self.color_green)
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
        self.ax1.bar(i_episode, cumulated_reward, color=self.color_green)

        ### Figure 2
        # plot bar (error distance)
        self.ax3.bar(i_episode, error_dist, color=self.color_orange)
        self.n_dist.append(error_dist)

        # plot line (average error distance)
        if len(self.n_dist) % self.avg_err_fre == 0:
            avg_err = self.moving_average( np.array(self.n_dist), self.avg_err_fre)
            self.ax3.plot(avg_err, color=self.color_red)

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
                                            x_label='Step', y_label='Sate')
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
                # ax4.plot(list_cob_x, '-o', color=self.color_green, label='offset')
                # ax5.plot(imu_pitch,  '-o', color=self.color_blue, label='pitch')
                # ax5.plot(imu_roll,   '-o', color=self.color_orange, label='roll')
                # ax6.plot(l_foot,     '-o', color=self.color_red, label='left_foot')
                # ax6.plot(r_foot,     '-o', color=self.color_green, label='right_foot')
                # self.legend_axis([ax4, ax5, ax6])
                # plt.draw()
                # plt.pause(0.1)

                if not done:
                    state = next_state
                else:
                    break
          
            ################
            # Plotting
            list_cob_x = np.array(list_cob_x)
            imu_pitch  = np.array(imu_pitch)
            imu_roll   = np.array(imu_roll)
            l_foot     = np.array(l_foot)
            r_foot     = np.array(r_foot)

            # dump pickle
            n_folder   = len(os.walk(self.result_path).__next__()[1])
            res_folder = "{}/{}".format(self.result_path, n_folder)
            
            if not os.path.exists(res_folder):
                os.mkdir(res_folder)

            pickle_name = "{}/{}_result.p".format(res_folder, n_folder)
            with open(pickle_name, 'wb') as filehandle:
                pickle.dump([list_cob_x, imu_pitch, imu_roll, l_foot, r_foot], filehandle)

            ax4.plot(list_cob_x, '-o', color=self.color_green,  label='CoB X')
            # std_cob_x = np.std(list_cob_x)
            # ax4.fill_between(range(len(list_cob_x)), list_cob_x - std_cob_x, list_cob_x + std_cob_x, alpha=0.25, color=self.color_green)

            ax5.plot(imu_pitch, '-o', color=self.color_blue,   label='Pitch')
            ax5.plot(imu_roll,  '-o', color=self.color_orange, label='Roll')
            ax6.plot(l_foot,    '-o', color=self.color_red,    label='Left Foot')
            ax6.plot(r_foot,    '-o', color=self.color_green,  label='Right Foot')
            self.legend_axis([ax4, ax5, ax6])
            error_dist = self.env.calc_dist()

            ax4.grid()
            ax5.grid()
            ax6.grid()

            # save figures
            fig3_name = "{}/cob_x.png".format(res_folder)
            fig3.savefig(fig3_name, dpi=fig3.dpi)

            fig4_name = "{}/IMU.png".format(res_folder)
            fig4.savefig(fig4_name, dpi=fig4.dpi)

            fig5_name = "{}/F_T.png".format(res_folder)
            fig5.savefig(fig5_name, dpi=fig5.dpi)

            if not self.dis_plot_rewards:
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
        plt.tight_layout()
        plt.show(block=True)

if __name__ == "__main__":
    rospy.init_node('pioneer_dragging_test_RL') # init node
    testing = Testing()
    testing.run()