#! /usr/bin/env python3

import rospy
import threading
import numpy as np
from time import sleep
from gym import spaces, logger
from scipy.interpolate import interp1d

from std_srvs.srv import Empty
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32
from gazebo_msgs.msg import ModelStates

from pioneer_utils.utils import *
from pioneer_walking.walking import Walking
from openai_ros.gazebo_connection import GazeboConnection
from openai_ros.controllers_connection import ControllersConnection

class Env:
    def __init__(self):
        self.walking            = Walking()
        self.gazebo             = GazeboConnection(start_init_physics_parameters=True, reset_world_or_sim='WORLD')
        controllers_list        = rospy.get_param('/controllers_list')
        self.controllers_object = ControllersConnection(namespace="thormang3", controllers_list=controllers_list)

        self.thread_rate    = rospy.Rate(30)
        self.thread2_flag   = False

        self.fall           = False
        self.walk_finished  = False
        self.first_run      = True

        self.fall_angle     = rospy.get_param('/fall_angle')
        self.cob_x          = rospy.get_param('/cob_x')
        self.step_size      = rospy.get_param('/step_size')
        self.angle_thresh   = rospy.get_param("/angle_thresh")
        self.distance       = rospy.get_param("/distance")
        self.mode_action    = rospy.get_param('/mode_action')
        
        self.dist_reward    = interp1d([abs(self.distance),0], [0,1])
        self.prev_imu_pitch = 0.0

        if self.mode_action == 'Discrete-Action':
            self.actions        = np.arange(start=-0.1, stop=0, step=0.01)
            self.action_space   = spaces.Discrete(self.actions.size)
        elif self.mode_action == 'Step-Action':
            self.action_space   = spaces.Discrete(3)
        
        self.observation_space = 2 # spaces.Box(2)

        # rqt_plot
        # self.imu_roll_pub  = rospy.Publisher('/pioneer/dragging/imu_roll',  Float32,  queue_size=1)
        # self.imu_pitch_pub = rospy.Publisher('/pioneer/dragging/imu_pitch', Float32,  queue_size=1)
        # self.imu_yaw_pub   = rospy.Publisher('/pioneer/dragging/imu_yaw',   Float32,  queue_size=1)

        # threading
        thread1    = threading.Thread(target = self.thread_check_robot, ) 
        thread1.start()
        self.mutex = threading.Lock()

    def thread_check_robot(self):
        rospy.Subscriber('/robotis/sensor/imu',  Imu,         self.imu_callback)
        rospy.Subscriber('/gazebo/model_states', ModelStates, self.model_callback, queue_size=1) #, buff_size=2**24)
        rospy.spin()

    def imu_callback(self, msg):
        self.mutex.acquire()
        euler_rad    = quaternion_to_euler( msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w )
        euler_deg    = np.degrees(euler_rad)
        self.imu_ori = { 'roll': euler_deg[0], 'pitch': euler_deg[1], 'yaw' : euler_deg[2] }

        try:
            if np.absolute(self.imu_ori['pitch']) > self.fall_angle and not self.fall:
                rospy.logwarn('[Env] Robot FALLING - angle: {}'.format(self.imu_ori['pitch']))
                self.fall = True

            # self.imu_roll_pub.publish(self.imu_ori['roll'])
            # self.imu_pitch_pub.publish(self.imu_ori['pitch'])
            # self.imu_yaw_pub.publish(self.imu_ori['yaw'])
        except:
            pass
        self.mutex.release()

    def model_callback(self, msg):
        self.mutex.acquire()
        models_name      = msg.name
        models_pose      = msg.pose
        thormang3_idx    = models_name.index('thormang3')
        thormang3_pose   = models_pose[thormang3_idx]
        self.thormang3_x = thormang3_pose.position.x
        self.thormang3_y = thormang3_pose.position.y

        # print('Distance X: {} \t Y: {}'.format(self.thormang3_x, self.thormang3_y))
        self.mutex.release()

    def wait_robot(self, obj, msg, msg1=None):
        if msg1 is None:
            while obj.status_msg != msg:
                if rospy.is_shutdown():
                    break
                else:
                    pass # do nothing
        else:
            while True:
                if obj.status_msg == msg or obj.status_msg == msg1:
                    break
                elif rospy.is_shutdown():
                    break
                else:
                    pass # do nothing
        # rospy.loginfo('[Env] Robot: {}'.format(msg))

    def initial(self):
        sleep(1)
        self.gazebo.unpauseSim()

        walking = self.walking
        rospy.loginfo('[Env] Init Pose')

        # set init pose
        walking.publisher_(walking.walking_pub, "ini_pose", latch=True)
        self.wait_robot(walking, "Finish Init Pose")
        rospy.loginfo('[Env] Finish Init Pose')

        # set walking mode
        walking.publisher_(walking.walking_pub, "set_mode")
        self.wait_robot(walking, "Walking_Module_is_enabled")
        rospy.loginfo('[Env] Set Mode')

        # turn on balance
        walking.publisher_(walking.walking_pub, "balance_on")
        self.wait_robot(self.walking, "Balance_Param_Setting_Finished", "Joint_FeedBack_Gain_Update_Finished")
        rospy.loginfo('[Env] Balance on')

        return

    def reset(self, i_episode):
        walking = self.walking
        self.thread2_flag  = True
        self.walk_finished = False
        self.cob_x         = rospy.get_param('/cob_x')

        # reset environment
        if self.fall:
            walking.publisher_(walking.walking_pub, "stop")
            self.wait_robot(walking, "Walking_Finished")

        if not self.first_run:
            # reset robot poses
            walking.publisher_(walking.walking_pub, "reset_pose")
            self.wait_robot(self.walking, "Finish Reset Pose")
            self.gazebo.resetSim()
            self.gazebo.change_gravity(0.0, 0.0, 0.0)
            self.controllers_object.reset_controllers()
            self.gazebo.change_gravity(0.0, 0.0, -9.81)
            self.gazebo.pauseSim()
            self.fall = False

        self.initial() # set robot initial pose

        # wait balance
        self.update_COM(-0.1)
        self.wait_robot(self.walking, "Balance_Param_Setting_Finished", "Joint_FeedBack_Gain_Update_Finished")
        rospy.loginfo('[Env] Update balance')

        print('\n*********************')
        print('Episode: {}'.format(i_episode))

        # start walking
        command          = 'backward'
        step_num         = rospy.get_param("/step_num") 
        step_time        = rospy.get_param("/step_time") 
        step_length      = rospy.get_param("/step_length") 
        side_step_length = rospy.get_param("/side_step_length") 
        step_angle_deg   = rospy.get_param("/step_angle_deg") 
        walking.walk_command(command, step_num, step_time, step_length, side_step_length, step_angle_deg)

        self.wait_robot(walking, "Walking_Started")
        self.thread2_flag = False
        thread2 = threading.Thread(target = self.thread_check_walk, args =(lambda : self.thread2_flag, )) 
        thread2.start()

        self.first_run = False
        state = self.get_state()
        return state

    def get_state(self):
        # IMU pitch
        return np.array([ self.imu_ori['pitch'], self.imu_ori['roll'] ])

    def update_COM(self, cob_x):
        # default_cob_x = -0.015
        # cob_x = -0.05 #-0.6 #-0.06 #-0.02 # -0.015 -0.1

        balance_dict = {
            "updating_duration"                     : 2.0*1.0,

            ####### cob_offset #######
            "cob_x_offset_m"                        : cob_x, #-0.015
            "cob_y_offset_m"                        : -0.00, #-0.00

            ####### FeedForward #####
            "hip_roll_swap_angle_rad"               : 0.00,
            
            ########## Gain ########
            # by gyro
            "foot_roll_gyro_p_gain"                 : 0.5,   #0.25,
            "foot_roll_gyro_d_gain"                 : 0.00,
            "foot_pitch_gyro_p_gain"                : 0.5,   #0.25,
            "foot_pitch_gyro_d_gain"                : 0.00,

            # by imu
            "foot_roll_angle_p_gain"                : 1.0,   #0.35,
            "foot_roll_angle_d_gain"                : 0.1,   #0.00,
            "foot_pitch_angle_p_gain"               : 1.0,   #0.25,
            "foot_pitch_angle_d_gain"               : 0.1,   #0.00,

            # by ft sensor
            "foot_x_force_p_gain"                   : 0.1,   #0.025,
            "foot_x_force_d_gain"                   : 0.00,
            "foot_y_force_p_gain"                   : 0.1,   #0.025,
            "foot_y_force_d_gain"                   : 0.00,
            "foot_z_force_p_gain"                   : 0.02,  #0.001,
            "foot_z_force_d_gain"                   : 0.00,
            
            "foot_roll_torque_p_gain"               : 0.0015, #0.0006,
            "foot_roll_torque_d_gain"               : 0.00,
            "foot_pitch_torque_p_gain"              : 0.0015, #0.0003,
            "foot_pitch_torque_d_gain"              : 0.00,

            ########## CUT OFF FREQUENCY ##########
            # by gyro
            "roll_gyro_cut_off_frequency"           : 50.0,   #40.0,
            "pitch_gyro_cut_off_frequency"          : 50.0,   #40.0,
            "roll_angle_cut_off_frequency"          : 50.0,   #40.0,
            "pitch_angle_cut_off_frequency"         : 50.0,   #40.0,
            "foot_x_force_cut_off_frequency"        : 40.0,   #20.0,
            "foot_y_force_cut_off_frequency"        : 40.0,   #20.0,
            "foot_z_force_cut_off_frequency"        : 40.0,   #20.0,
            "foot_roll_torque_cut_off_frequency"    : 40.0,   #20.0,
            "foot_pitch_torque_cut_off_frequency"   : 40.0    #20.0
        }

        rospy.loginfo('[Env] COM_X: {}'.format(cob_x))
        self.walking.set_balance_param(balance_dict)

    def thread_check_walk(self, stop_thread):
        while not rospy.is_shutdown():
            if self.walking.status_msg == "Walking_Finished":
                self.walk_finished = True
                rospy.loginfo('[Env] Walk finished')
                break
        
            if stop_thread():
                rospy.loginfo("[Env] Thread check robot killed")
                break
            self.thread_rate.sleep()

    def calc_dist(self):
        target_pos     = np.array([ [self.distance, 0.0] ])
        current_pos    = np.array([ [self.thormang3_x, self.thormang3_y] ])
        euclidean_dist = np.linalg.norm(target_pos - current_pos, axis=1)

        euclidean_dist = np.asscalar(euclidean_dist)

        if euclidean_dist >= abs(self.distance):
            euclidean_dist = self.distance

        return euclidean_dist

    def reward_function(self, imu_pitch, euclidean_dist):
        if self.fall:
            reward = -10 # robot fell down
        elif self.walk_finished:
            reward = 10 # robot succesfully finished
        else:
            dist_reward = self.dist_reward(euclidean_dist) * 0.3

            if abs(imu_pitch) <= self.angle_thresh:
                reward = 1 
            elif self.prev_imu_pitch == imu_pitch:
                reward = 0
            else:
                reward = 1.0 / ( abs(imu_pitch) + 1.0 - self.angle_thresh)

            reward *= 0.7
            reward = reward + dist_reward

            '''
            rewards based on IMU and distance
            balance reward  = 0.7 
            distance reward = 0.3
            reward = balance_reward + distance_reward
            '''
        return reward

    def select_action(self, action):
        ## actions (update COM)

        if self.mode_action == 'Discrete-Action':
            self.cob_x   = self.actions[action]

        elif self.mode_action == 'Step-Action':
            if action == 0: # increment
                self.cob_x += self.step_size
            elif action == 1: # decrement
                self.cob_x -= self.step_size
            else: # stop
                pass

        self.update_COM(self.cob_x)
        self.wait_robot(self.walking, "Balance_Param_Setting_Finished")

        return

    def step(self, action):
        # select action
        self.select_action(action)

        # state (IMU Pitch)
        state   = self.get_state()
        rospy.loginfo('[Env] IMU_Pitch: {}'.format(state))

        done    = False
        # if self.fall or self.walk_finished:
        if self.fall or self.walk_finished:
            done = True

        ## rewards
        reward = self.reward_function( self.imu_ori['pitch'], self.calc_dist() )
        rospy.loginfo('[Env] Reward: {}'.format(reward))
        print()

        self.prev_imu_pitch = self.imu_ori['pitch']
        info = None
        return state, reward, done, info 

    def close(self):
        walking = self.walking

        # reset environment
        walking.publisher_(walking.walking_pub, "stop")
        self.wait_robot(walking, "Walking_Finished")

        # wait until robot balance
        self.gazebo.resetWorld()
        self.fall = False
        self.thread2_flag = True