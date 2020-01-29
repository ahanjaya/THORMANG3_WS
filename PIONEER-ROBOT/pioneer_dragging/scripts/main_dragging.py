#! /usr/bin/env python3

import rospy
import random
import threading
import numpy as np
import matplotlib.pyplot as plt

from time import sleep
from environment import Env
from deepQlearn import DQN, ExperienceReplay

# global variable
n_episode = []
n_epsilon = []
color1    = 'tab:green'
color2    = 'tab:blue'

def plot_results(ax1, ax2, i_episode, cumulated_reward, epsilon):
    ax1.bar(i_episode, cumulated_reward, color=color1)
    n_episode.append(i_episode)
    n_epsilon.append(epsilon)
    ax2.plot(n_episode, n_epsilon, color=color2)

    plt.draw()
    plt.pause(0.1)

###################
###### Main #######

def main():
    # init node
    rospy.init_node('pioneer_dragging')

    # Parameters
    n_episodes    = rospy.get_param("/n_episodes") 
    n_step        = rospy.get_param("/n_steps") 
    plotting      = rospy.get_param('/plotting')
    mode_action   = rospy.get_param('/mode_action')
    mem_size      = rospy.get_param('/mem_size')
    batch_size    = rospy.get_param('/batch_size')
    mode_optimize = rospy.get_param('/mode_optimize')

    # create environment
    env       = Env()
    n_states  = env.observation_space
    n_actions = env.action_space.n

    # create Deep Q-Network
    dqn       = DQN(n_states, n_actions)
    memory    = ExperienceReplay(mem_size)


    # plotting
    style_plot = random.choice(plt.style.available)
    plt.style.use(style_plot)
    plt.ion()

    fig = plt.figure(figsize=(12,5))
    ax1 = fig.add_subplot(1,1,1)
    ax2 = ax1.twinx() 

    plt.title('{}'.format(mode_action))
    ax1.set_xlabel('Episode')
    ax1.set_ylabel('Reward',  color=color1)
    ax2.set_ylabel('Epsilon', color=color2)
    ax1.tick_params(axis='y', labelcolor=color1)
    ax2.tick_params(axis='y', labelcolor=color2)
    
    #########################################
    ###### Reinfrocement Training loop ######

    for i_episode in range (n_episodes):
        state = env.reset(i_episode)
        cumulated_reward = 0

        while not rospy.is_shutdown():
            action, epsilon = dqn.select_action(state)

            # action = env.action_space.sample()
            rospy.loginfo('[RL] action: {}'.format(action))

            next_state, reward, done, info = env.step(action)
            memory.push(state, action, next_state, reward, done)
            cumulated_reward += reward

            ################################
            ######### optimize #############

            if mode_optimize == 'normal_dqn':
                # without experience replay memory
                dqn.optimize(state, action, next_state, reward, done)

            elif mode_optimize == 'dqn_replay_memory':
                # with experience replay memory
                if len(memory) > batch_size:
                    state_mem, action_mem, next_state_mem, reward_mem, done_mem = memory.sample(batch_size)
                    dqn.optimize_with_replay_memory(state_mem, action_mem, next_state_mem, reward_mem, done_mem)

            elif mode_optimize == 'dqn_taget_net':
                # with experience target net
                if len(memory) > batch_size:
                    state_mem, action_mem, next_state_mem, reward_mem, done_mem = memory.sample(batch_size)
                    dqn.optimize_with_target_net(state_mem, action_mem, next_state_mem, reward_mem, done_mem)

            if not done:
                state = next_state
            else:
                break

        if plotting:
            plot_results(ax1, ax2, i_episode, cumulated_reward, epsilon)

    env.close()
    print()
    rospy.loginfo('[RL] Exit ...')
    if plotting:
        rospy.loginfo('[RL] Style plot: {}'.format(style_plot))
        plt.show(block=True)


if __name__ == "__main__":
    main()