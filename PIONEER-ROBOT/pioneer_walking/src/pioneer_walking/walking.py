#!/usr/bin/env python3

import rospy
import threading
import numpy as np
from time import sleep
from std_msgs.msg import String
from pioneer_utils.utils import *
from geometry_msgs.msg import Quaternion
from robotis_controller_msgs.msg import StatusMsg
from thormang3_foot_step_generator.msg import FootStepCommand
from thormang3_walking_module_msgs.srv import SetBalanceParam
from thormang3_walking_module_msgs.msg import RobotPose, BalanceParam, PoseXYZRPY

class Walking:
    def __init__(self):
        # rospy.init_node('pioneer_walking', anonymous=False)

        self.thread1_flag   = False
        self.pub_rate       = rospy.Rate(10)
        self.thread_rate    = rospy.Rate(60)
        self.module_name    = None
        self.status_msg     = None
        self.mutex          = threading.Lock()
        self.des_pose_roll  = 0
        self.des_pose_pitch = 0

        ## Publisher
        self.walking_pub         = rospy.Publisher('/robotis/walking/command',    String,    queue_size=10) #, latch=True)
        self.robot_pose_pub      = rospy.Publisher('/robotis/walking/robot_pose', RobotPose, queue_size=10) #, latch=True)
        self.walking_command_pub = rospy.Publisher('/robotis/thormang3_foot_step_generator/walking_command', FootStepCommand, queue_size=10) #, latch=True)

        ## Subscriber
        rospy.Subscriber('/robotis/walking/des_balance',  PoseXYZRPY,  self.des_balance_callback)
        
        ## Service Client
        self.set_walking_balance_param = rospy.ServiceProxy('/robotis/walking/set_balance_param', SetBalanceParam)
        
        self.read_robot_status()

    def kill_threads(self):
        self.thread1_flag = True

    def des_balance_callback(self, msg):
        self.des_pose_roll  = msg.roll
        self.des_pose_pitch = msg.pitch
        # rospy.loginfo("[Walking] Des roll: {}, pitch: {}".format(msg.roll, msg.pitch))
        
    def thread_read_robot_status(self, stop_thread):
        rospy.Subscriber('/robotis/status', StatusMsg, self.robot_status_callback)
        rospy.spin()

        # while True:
        #     ## Subscriber
        #     rospy.Subscriber('/robotis/status', StatusMsg, self.robot_status_callback)
        #     self.thread_rate.sleep()
        #     if stop_thread():
        #         rospy.loginfo("[Walking] Thread killed")
        #         break

    def robot_status_callback(self, msg):
        self.mutex.acquire()
        self.module_name = msg.module_name
        self.status_msg  = msg.status_msg
        self.mutex.release()
        # rospy.loginfo(self.status_msg)

    def read_robot_status(self):
        thread1 = threading.Thread(target = self.thread_read_robot_status, args =(lambda : self.thread1_flag, )) 
        thread1.start()

    def publisher_(self, topic, msg, latch=False):
        if latch:
            for _ in range(4):
                topic.publish(msg)
                self.pub_rate.sleep()
        else:
            topic.publish(msg)

    def set_robot_pose(self, r_foot_x, r_foot_y, r_foot_z, r_foot_roll, r_foot_pitch, r_foot_yaw,\
                             l_foot_x, l_foot_y, l_foot_z, l_foot_roll, l_foot_pitch, l_foot_yaw,\
                             cob_x, cob_y, cob_z, cob_roll, cob_pitch, cob_yaw) :
        msg = RobotPose()

        # Right Foot
        msg.global_to_right_foot.position.x = r_foot_x
        msg.global_to_right_foot.position.y = r_foot_y
        msg.global_to_right_foot.position.z = r_foot_z
        r_foot_quaternion = euler_to_quaternion(r_foot_roll, r_foot_pitch, r_foot_yaw)
        msg.global_to_right_foot.orientation.x = r_foot_quaternion[0]
        msg.global_to_right_foot.orientation.y = r_foot_quaternion[1]
        msg.global_to_right_foot.orientation.z = r_foot_quaternion[2]
        msg.global_to_right_foot.orientation.w = r_foot_quaternion[3]

        # Left Foot
        msg.global_to_left_foot.position.x = l_foot_x
        msg.global_to_left_foot.position.y = l_foot_y
        msg.global_to_left_foot.position.z = l_foot_z
        l_foot_quaternion = euler_to_quaternion(l_foot_roll, l_foot_pitch, l_foot_yaw)
        msg.global_to_left_foot.orientation.x = l_foot_quaternion[0]
        msg.global_to_left_foot.orientation.y = l_foot_quaternion[1]
        msg.global_to_left_foot.orientation.z = l_foot_quaternion[2]
        msg.global_to_left_foot.orientation.w = l_foot_quaternion[3]

        # Centre of Body
        msg.global_to_center_of_body.position.x = cob_x
        msg.global_to_center_of_body.position.y = cob_y
        msg.global_to_center_of_body.position.z = cob_z
        cob_quaternion = euler_to_quaternion(cob_roll, cob_pitch, cob_yaw)
        msg.global_to_center_of_body.orientation.x = cob_quaternion[0]
        msg.global_to_center_of_body.orientation.y = cob_quaternion[1]
        msg.global_to_center_of_body.orientation.z = cob_quaternion[2]
        msg.global_to_center_of_body.orientation.w = cob_quaternion[3]

        if self.status_msg != "Walking_Started": 
            self.publisher_(self.robot_pose_pub, msg)
        else:
            rospy.warn("[Walking] Robot is walking now, just please set this parameter before starting robot's walk.")

    def set_balance_param(self, balance_dict):
        rospy.wait_for_service('/robotis/walking/set_balance_param')

        ## Walk Balance Parameter
        updating_duration                                 = balance_dict.get("updating_duration")
        balance_param = BalanceParam()
        balance_param.cob_x_offset_m                      = balance_dict.get("cob_x_offset_m")
        balance_param.cob_y_offset_m                      = balance_dict.get("cob_y_offset_m")
        balance_param.hip_roll_swap_angle_rad             = balance_dict.get("hip_roll_swap_angle_rad")
        balance_param.foot_roll_gyro_p_gain               = balance_dict.get("foot_roll_gyro_p_gain")
        balance_param.foot_roll_gyro_d_gain               = balance_dict.get("foot_roll_gyro_d_gain")
        balance_param.foot_pitch_gyro_p_gain              = balance_dict.get("foot_pitch_gyro_p_gain")
        balance_param.foot_pitch_gyro_d_gain              = balance_dict.get("foot_pitch_gyro_d_gain")
        balance_param.foot_roll_angle_p_gain              = balance_dict.get("foot_roll_angle_p_gain")
        balance_param.foot_roll_angle_d_gain              = balance_dict.get("foot_roll_angle_d_gain")
        balance_param.foot_pitch_angle_p_gain             = balance_dict.get("foot_pitch_angle_p_gain")
        balance_param.foot_pitch_angle_d_gain             = balance_dict.get("foot_pitch_angle_d_gain")
        balance_param.foot_x_force_p_gain                 = balance_dict.get("foot_x_force_p_gain")
        balance_param.foot_x_force_d_gain                 = balance_dict.get("foot_x_force_d_gain")
        balance_param.foot_y_force_p_gain                 = balance_dict.get("foot_y_force_p_gain")
        balance_param.foot_y_force_d_gain                 = balance_dict.get("foot_y_force_d_gain")
        balance_param.foot_z_force_p_gain                 = balance_dict.get("foot_z_force_p_gain")
        balance_param.foot_z_force_d_gain                 = balance_dict.get("foot_z_force_d_gain")
        balance_param.foot_roll_torque_p_gain             = balance_dict.get("foot_roll_torque_p_gain")
        balance_param.foot_roll_torque_d_gain             = balance_dict.get("foot_roll_torque_d_gain")
        balance_param.foot_pitch_torque_p_gain            = balance_dict.get("foot_pitch_torque_p_gain")
        balance_param.foot_pitch_torque_d_gain            = balance_dict.get("foot_pitch_torque_d_gain")
        balance_param.roll_gyro_cut_off_frequency         = balance_dict.get("roll_gyro_cut_off_frequency")
        balance_param.pitch_gyro_cut_off_frequency        = balance_dict.get("pitch_gyro_cut_off_frequency")
        balance_param.roll_angle_cut_off_frequency        = balance_dict.get("roll_angle_cut_off_frequency")
        balance_param.pitch_angle_cut_off_frequency       = balance_dict.get("pitch_angle_cut_off_frequency")
        balance_param.foot_x_force_cut_off_frequency      = balance_dict.get("foot_x_force_cut_off_frequency")
        balance_param.foot_y_force_cut_off_frequency      = balance_dict.get("foot_y_force_cut_off_frequency")
        balance_param.foot_z_force_cut_off_frequency      = balance_dict.get("foot_z_force_cut_off_frequency")
        balance_param.foot_roll_torque_cut_off_frequency  = balance_dict.get("foot_roll_torque_cut_off_frequency")
        balance_param.foot_pitch_torque_cut_off_frequency = balance_dict.get("foot_pitch_torque_cut_off_frequency")
        
        try:
            resp = self.set_walking_balance_param(updating_duration, balance_param)
            if resp.result == 0:
                rospy.loginfo("[Walking]: Succeed to set balance param")
            elif resp.result == 2:
                rospy.logerr("[Walking]: NOT_ENABLED_WALKING_MODULE")
            elif resp.result == 32:
                rospy.logerr("[Walking]: PREV_REQUEST_IS_NOT_FINISHED")
            else:
                rospy.loginfo("[Walking]: Failed to set balance param")
        except rospy.ServiceException as e: # python3
            rospy.logerr("Service call failed: %s" %e)

    def walk_command(self, command, step_num, step_time, step_length, side_step_length, step_angle_deg):
        msg = FootStepCommand()
        msg.command           = command
        msg.step_num          = step_num
        msg.step_time         = step_time
        msg.step_length       = step_length
        msg.side_step_length  = side_step_length
        msg.step_angle_rad    = np.radians(step_angle_deg)

        try:
            self.publisher_(self.walking_command_pub, msg)
            rospy.loginfo("[Walking] Publish walk command : {0}".format(msg))
        except rospy.ServiceException as e: # python3
            rospy.logerr("[Walking] Failed publish walk command")

# if __name__ == '__main__':
#     walk = Walking()
    # walk.read_robot_status()
    # sleep(5)
    # walk.kill_threads()

    # rospy.loginfo("init pose")
    # walk.publisher_(walk.walking_pub, "ini_pose", latch=True)
    # sleep(5)
    # rospy.loginfo("set walking module")
    # walk.publisher_(walk.walking_pub, "set_mode")
    # sleep(5)
    # rospy.loginfo("set balance_on")
    # walk.publisher_(walk.walking_pub, "balance_on")
    # sleep(5)
    # rospy.loginfo("walk forward")
    # walk.publisher_(walk.walking_pub, "forward")
    
    # walk.set_robot_pose(0.0, -0.093, -0.63, 0.0, 0.0, 0.0,\
    #                     0.0, 0.093, -0.63, 0.0, 0.0, 0.0,\
    #                     0.0, 0.0, 0.0, 0.0, 0.0, 0.0)

    # sleep(5)
    # walk.walk_command("forward", 2, 1.0, 0.1, 0.05, 5)
        
    # balance_dict = {
    #         "updating_duration"                     : 2.0*1.0,
    #         "cob_x_offset_m"                        : -0.015,
    #         "cob_y_offset_m"                        : -0.00,
    #         "hip_roll_swap_angle_rad"               : 0.00,
    #         "foot_roll_gyro_p_gain"                 : 0.25,
    #         "foot_roll_gyro_d_gain"                 : 0.00,
    #         "foot_pitch_gyro_p_gain"                : 0.25,
    #         "foot_pitch_gyro_d_gain"                : 0.00,
    #         "foot_roll_angle_p_gain"                : 0.35,
    #         "foot_roll_angle_d_gain"                : 0.00,
    #         "foot_pitch_angle_p_gain"               : 0.25,
    #         "foot_pitch_angle_d_gain"               : 0.00,
    #         "foot_x_force_p_gain"                   : 0.025,
    #         "foot_x_force_d_gain"                   : 0.00,
    #         "foot_y_force_p_gain"                   : 0.025,
    #         "foot_y_force_d_gain"                   : 0.00,
    #         "foot_z_force_p_gain"                   : 0.001,
    #         "foot_z_force_d_gain"                   : 0.00,
    #         "foot_roll_torque_p_gain"               : 0.0006,
    #         "foot_roll_torque_d_gain"               : 0.00,
    #         "foot_pitch_torque_p_gain"              : 0.0003,
    #         "foot_pitch_torque_d_gain"              : 0.00,
    #         "roll_gyro_cut_off_frequency"           : 40.0,
    #         "pitch_gyro_cut_off_frequency"          : 40.0,
    #         "roll_angle_cut_off_frequency"          : 40.0,
    #         "pitch_angle_cut_off_frequency"         : 40.0,
    #         "foot_x_force_cut_off_frequency"        : 20.0,
    #         "foot_y_force_cut_off_frequency"        : 20.0,
    #         "foot_z_force_cut_off_frequency"        : 20.0,
    #         "foot_roll_torque_cut_off_frequency"    : 20.0,
    #         "foot_pitch_torque_cut_off_frequency"   : 20.0
    # }
    # walk.set_balance_param(balance_dict)
