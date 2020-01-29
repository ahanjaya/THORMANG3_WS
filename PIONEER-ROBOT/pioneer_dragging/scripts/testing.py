#! /usr/bin/env python3

import rospy
import threading
import numpy as np
from time import sleep

from std_srvs.srv import Empty
from std_msgs.msg import Float32
from sensor_msgs.msg import Imu
from pioneer_utils.utils import *
from pioneer_walking.walking import Walking

class Testing:
    def __init__(self):
        rospy.wait_for_service('/gazebo/reset_world')
        rospy.wait_for_service('/gazebo/reset_simulation')
        rospy.wait_for_service('/gazebo/unpause_physics')

        self.reset_world      = rospy.ServiceProxy('/gazebo/reset_world',      Empty)
        self.reset_simulation = rospy.ServiceProxy('/gazebo/reset_simulation', Empty)
        self.play_simluation  = rospy.ServiceProxy('/gazebo/unpause_physics',  Empty)

        # rqt_plot
        self.imu_roll_pub  = rospy.Publisher('/pioneer/dragging/imu_roll',  Float32,  queue_size=1)
        self.imu_pitch_pub = rospy.Publisher('/pioneer/dragging/imu_pitch', Float32,  queue_size=1)
        self.imu_yaw_pub   = rospy.Publisher('/pioneer/dragging/imu_yaw',   Float32,  queue_size=1)

        self.play_simluation()
        self.walking     = Walking()

        # threading
        thread1    = threading.Thread(target = self.thread_check_robot, ) 
        thread1.start()
        self.mutex = threading.Lock()

    def thread_check_robot(self):
        rospy.Subscriber('/robotis/sensor/imu', Imu, self.imu_callback)
        rospy.spin()

    def imu_callback(self, msg):
        self.mutex.acquire()
        euler_rad    = quaternion_to_euler( msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w )
        euler_deg    = np.degrees(euler_rad)
        self.imu_ori = { 'roll': euler_deg[0], 'pitch': euler_deg[1], 'yaw' : euler_deg[2] }

        try:
            self.imu_roll_pub.publish(self.imu_ori['roll'])
            self.imu_pitch_pub.publish(self.imu_ori['pitch'])
            self.imu_yaw_pub.publish(self.imu_ori['yaw'])
        
        except:
            pass
        self.mutex.release()

    def wait_robot(self, obj, msg):
        while obj.status_msg != msg:
            pass # do nothing
        # rospy.loginfo('[Env] Robot: {}'.format(msg))

    def initial(self):
        walking = self.walking

        # set init pose
        walking.publisher_(walking.walking_pub, "ini_pose", latch=True)
        self.wait_robot(walking, "Finish Init Pose")

        # set walking mode
        walking.publisher_(walking.walking_pub, "set_mode")
        self.wait_robot(walking, "Walking_Module_is_enabled")

        # turn on balance
        walking.publisher_(walking.walking_pub, "balance_on")
        self.wait_robot(walking, "Balance_Param_Setting_Finished")
        self.wait_robot(walking, "Joint_FeedBack_Gain_Update_Finished")

        return

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

        self.walking.set_balance_param(balance_dict)

            
if __name__ == '__main__':
    rospy.init_node('pioneer_wolf_walk') #, disable_signals=True)

    wolf = Testing()
    # wolf.initial()

    # wolf.update_COM(0.1)