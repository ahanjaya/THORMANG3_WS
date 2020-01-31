#!/usr/bin/env python3

import math
import random
import matplotlib.pyplot as plt

# # **************************
# # plotting

# style_plot = random.choice(plt.style.available)
# plt.style.use(style_plot)
# plt.ion()

# fig    = plt.figure(figsize=(12,5))
# ax1    = fig.add_subplot(1,1,1)
# ax2    = ax1.twinx() 
# color1 = 'tab:green'
# color2 = 'tab:blue'

# plt.title('Hanjaya')
# ax1.set_xlabel('Episode')
# ax1.set_ylabel('Reward',  color=color1)
# ax2.set_ylabel('Epsilon', color=color2)  # we already handled the x-label with ax1

# ax1.tick_params(axis='y', labelcolor=color1)
# ax2.tick_params(axis='y', labelcolor=color2)

# n_episode = []
# n_epsilon = []

# def plot_results(i_episode, cumulated_reward, epsilon):
#     ax1.bar(i_episode, cumulated_reward, color=color1)

#     n_episode.append(i_episode)
#     n_epsilon.append(epsilon)
#     ax2.plot(n_episode, n_epsilon, color=color2)

#     plt.draw()
#     plt.pause(0.1)

# for i_episode in range (100):
#     epsilon = random.random()
#     cumulated_reward = random.random() * 10
#     print(epsilon, cumulated_reward)

#     plot_results(i_episode, cumulated_reward, epsilon)


#     # plt.gca().cla() # optionally clear axes
#     # plt.bar(i_episode, cumulative_reward, alpha=0.9, color='blue')
    

# plt.show(block=True)
# plt.ioff()

steps_done = 0
epsilon_final = 0.05
epsilon = 0.9
n_episodes = 500
epsilon_decay = 600000
# epsilon_decay = n_episodes * 7 # lower value faster converge

print(epsilon_decay)

def calculate_epsilon(epsilon):
    global steps_done
    steps_done += 1
    epsilon = epsilon_final + (epsilon - epsilon_final) * \
                math.exp(-1. * steps_done / epsilon_decay)
    return epsilon

n_epsilon = []
for i in range(n_episodes):
    for j in range(8):
        epsilon = calculate_epsilon(epsilon)

    n_epsilon.append(epsilon)
    # print(epsilon)

plt.plot(n_epsilon)

plt.show()