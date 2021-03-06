#!/usr/bin/env python3

# import math
# import random
# import numpy as np
# from time import sleep
# import matplotlib.pyplot as plt
# from scipy.interpolate import interp1d

# def calc_dist(x, y):
#     target_pos     = np.array([ [-1.5, 0.0] ])
#     current_pos    = np.array([ [x, y] ])
#     euclidean_dist = np.linalg.norm(target_pos - current_pos, axis=1)
#     return np.asscalar(euclidean_dist)

# # print(calc_dist(-1.41959809084, -0.526477510985))
# print(calc_dist(-1.7991616, 0.00666374))

# # reward = interp1d([1.5,0], [0, 1])

# # for i in range(16):
# #     i = - ( i /10 )

# #     err = calc_dist(i)

# #     han = reward(err)
# #     reduced_reward = 0.3 * han

# #     print( 'err: {}, rewards: {}, dist: {}'.format(err, han, reduced_reward) )



# import rospy
# from gazebo_msgs.srv import SpawnModel
# from geometry_msgs.msg import Pose
# from pioneer_utils.utils import *

# from pathlib import Path

# rospy.init_node('insert_object',log_level=rospy.INFO)

# initial_pose = Pose()
# initial_pose.position.x = 0.4
# initial_pose.position.y = -0.1
# initial_pose.position.z = 0

# # ori = euler_to_quaternion(roll=0, pitch=45, yaw=0)
# # initial_pose.orientation.x = ori[0]
# # initial_pose.orientation.y = ori[1]
# # initial_pose.orientation.z = ori[2]
# # initial_pose.orientation.w = ori[3]


# home = str(Path.home())
# # f = open(home + '/.gazebo/models/foot_chair/model.sdf','r')
# f = open(home + '/.gazebo/models/suitcase/model.sdf','r')
# sdff = f.read()

# rospy.wait_for_service('gazebo/spawn_sdf_model')
# spawn_model_prox = rospy.ServiceProxy('gazebo/spawn_sdf_model', SpawnModel)
# spawn_model_prox("some_robo_name", sdff, "robotos_name_space", initial_pose, "world")


# f=open("guru99.txt", "a+")

# for i in range(2):
#     # f.write("Appended line %d\r\n" % (i+1))
#     f.write('{}\n'.format(i))
# f.close()

# epsilon = 0.9
# epsilon_final = 0.05
# epsilon_decay = 1000000


# def calculate_epsilon(epsilon, i_episode):
#     epsilon = epsilon_final + (epsilon - epsilon_final) * \
#                 math.exp(-1. * i_episode / epsilon_decay)
#     return epsilon

# eps = []

# for i in range(2000):
#     for j in range(8):
#         epsilon = calculate_epsilon(epsilon, i)

#     eps.append(epsilon)

# plt.plot(eps)
# plt.show()



from openai_ros.gazebo_connection import GazeboConnection
import rospy
from std_msgs.msg import Float32


rospy.init_node('pioneer_RL_dragging') # init node
left_pub  = rospy.Publisher('/pioneer/dragging/left',  Float32,  queue_size=1)
right_pub = rospy.Publisher('/pioneer/dragging/right', Float32,  queue_size=1)

gc = GazeboConnection(start_init_physics_parameters=True, reset_world_or_sim='WORLD')

while not rospy.is_shutdown():
    left_foot = gc.getLinkState('thormang3::l_leg_an_r_link')
    right_foot = gc.getLinkState('thormang3::r_leg_an_r_link')

    left_pub.publish(left_foot)
    right_pub.publish(right_foot)




