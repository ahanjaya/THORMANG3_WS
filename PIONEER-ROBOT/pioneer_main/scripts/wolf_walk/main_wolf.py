#!/usr/bin/env python3

import os
import sys
import cv2
import yaml
import time
import pptk
import rospy
import rospkg
import numpy as np
from time import sleep
from std_msgs.msg import String, Bool, Int16
from pioneer_motion.action import Action
from pioneer_motion.motion import Motion
from pioneer_sensors.sensor import Sensor
from pioneer_walking.walking import Walking
from pioneer_utils.export_excel_wolf import Excel

class Wolf_Walk:
    def __init__(self, save):      
        self.save_data    = self.str_to_bool(save)
    
        if self.save_data:
            rospack         = rospkg.RosPack()
            data_path       = rospack.get_path("pioneer_main") + "/data/wolf_walk"

            self.n_folder   = len(os.walk(data_path).__next__()[1])
            self.data_path  = "{}/{}".format(data_path, self.n_folder)

            if not os.path.exists(self.data_path):
                os.mkdir(self.data_path)

            self.excel      = Excel('{}/wolf_data_{}.xlsx'   .format(self.data_path, self.n_folder))
            self.lidar_file = "{}/wolf_lidar_pcl-{}.npz"     .format(self.data_path, self.n_folder)
            self.yaml_file  = "{}/wolf_initial_pose-{}.yaml" .format(self.data_path, self.n_folder)

            rospy.loginfo('[WW] Data path: {}'.format(self.data_path))

        self.main_rate    = rospy.Rate(60)
        self.sensor       = Sensor("Thormang3_Wolf")
        self.action       = Action("Thormang3_Wolf")
        self.motion       = Motion()
        self.walking      = Walking()

        self.thread_rate  = rospy.Rate(15)
        self.thread1_flag = False
        self.scan_finish  = False
        self.visual_ptk1  = None
        self.robot_frame  = 0
        self.tripod_frame = 0
        self.state        = None
        self.balance      = False
        self.walk_mode    = False
        self.pull_motion  = 'black_chair'
        self.num_step     = 12
        self.init_head_p  = 40

        self.debug        = True
        if self.debug:
            self.num_step = 5

        ## Param
        # rospy.set_param('/initial_pose/centre_of_body/cob_x_offset_m', -0.1)

        ## Publisher
        self.save_pub  = rospy.Publisher('/pioneer/wolf/save_data', Bool,  queue_size=10)

        ## Subscriber
        rospy.Subscriber('/robotis/sensor/move_lidar',  String,  self.lidar_turn_callback)
        rospy.Subscriber('/pioneer/wolf/robot_frame',   Int16,   self.robot_frame_callback)
        rospy.Subscriber('/pioneer/wolf/tripod_frame',  Int16,   self.tripod_frame_callback)
        rospy.Subscriber('/pioneer/wolf/state',         String,  self.state_callback)
        rospy.Subscriber('/pioneer/wolf/num_step',      Int16,   self.num_step_callback)
        rospy.Subscriber('/pioneer/wolf/pull_motion',   String,  self.pull_motion_callback)
    
    def str_to_bool(self, s):
        if s == 'true':
            return True
        elif s == 'false':
            return False
        else:
            raise ValueError # evil ValueError that doesn't tell you what the wrong value was

    def state_callback(self, msg):
        if msg.data == "update_balance":
            self.set_initial_pose()
        else:
            self.state = msg.data
            rospy.loginfo("[WW] Received: {}".format(self.state))

    def wait_robot(self, obj, msg):
        while obj.status_msg != msg:
            pass # do nothing

    def lidar_turn_callback(self, msg):
        if msg.data == "end":
            rospy.loginfo("[WW] Lidar finish scan")

            while self.sensor.lidar_pcl is None:
                pass

            self.plot_point_cloud('wolf_lidar_pcl', self.sensor.lidar_pcl, hardware='lidar') # <-- plot

            if self.save_data:
                np.savez(self.lidar_file, pcl=self.sensor.lidar_pcl)
                rospy.loginfo('[WW] Saved lidar pcl data: {}'.format(self.lidar_file))

    def robot_frame_callback(self, msg):
        self.robot_frame = msg.data

    def tripod_frame_callback(self, msg):
        self.tripod_frame = msg.data

    def num_step_callback(self, msg):
        self.num_step = msg.data
        rospy.loginfo('[WW] Update num step: {}'.format(self.num_step))

    def pull_motion_callback(self, msg):
        self.pull_motion = msg.data
        rospy.loginfo('[WW] Update pull motion: {}'.format(self.pull_motion))
    
    def plot_point_cloud(self, label, pcl_data, hardware):
        rospy.loginfo("[Wolf] {} - length pcl : {}".format(label, pcl_data.shape))
        visual_ptk = pptk.viewer(pcl_data[:,:3])

        if hardware == "lidar":
            visual_ptk.attributes(pcl_data[:,-1])
            visual_ptk.set(point_size=0.0025)
        elif hardware == "realsense":
            visual_ptk.set(point_size=0.0025)
            color = pcl_data[:, 3:]
            visual_ptk.attributes(color / 255)
        return visual_ptk

    def close_all(self):
        if self.visual_ptk1 is not None:
            self.visual_ptk1.close()

    def set_initial_pose(self):
        self.balance = True

        ## apply = rospy.get_param("/initial_pose/centre_of_body/apply")
        # cob_x = rospy.get_param("/initial_pose/centre_of_body/cob_x_offset_m")
        # cob_y = rospy.get_param("/initial_pose/centre_of_body/cob_y_offset_m")

        cob_x = -0.05 #-0.6 #-0.06 #-0.02 # -0.015 -0.1
        cob_y = -0.00

        balance_dict = {
            "updating_duration"                     : 2.0*1.0,

            ####### cob_offset #######
            "cob_x_offset_m"                        : cob_x, #-0.015
            "cob_y_offset_m"                        : cob_y, #-0.00

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

        rospy.loginfo('[WW] Finish set initial pose')
        rospy.loginfo('[WW] cob_x_offset_m : {}'.format(cob_x))
        rospy.loginfo('[WW] cob_y_offset_m : {}'.format(cob_y))

        if self.save_data:
            centre_of_body = { 'x_offset': cob_x, 'y_offset': cob_y}
            initial_config = {}
            initial_config['pull_motion']    = self.pull_motion
            initial_config['centre_of_body'] = centre_of_body
            with open(self.yaml_file, 'w') as f:
                yaml.dump(initial_config, f, default_flow_style=False)

    def wait_action(self):
        sleep(0.5)
        while not self.action.finish_action:
            pass

    def run(self):
        sensor  = self.sensor
        motion  = self.motion
        action  = self.action
        walking = self.walking

        real_sense_cnt,     rate_cnt            = 0, 0
        cur_len_real_sense, prev_len_real_sense = 0, 0

        self.state = 'ini_pose'
        sleep(2)

        while not rospy.is_shutdown():
            if self.state == 'ini_pose':
                rospy.loginfo('[WW] Robot State : {}'.format(self.state))
                self.walk_mode = False
                self.balance   = False

                motion.publisher_(motion.init_pose_pub, "ini_pose", latch=True)
                self.wait_robot(motion, "Finish Init Pose")

                # self.state = None
                if self.debug:
                    self.state = None
                else:
                    if self.save_data:
                        self.state = 'scan_lidar'
                    else:
                        self.state = 'pull_pose'

            elif self.state == 'scan_lidar':
                rospy.loginfo('[WW] Robot State : {}'.format(self.state))
                self.walk_mode = False
                self.balance   = False

                motion.publisher_(motion.module_control_pub, "head_control_module", latch=True)
                motion.publisher_(motion.move_lidar_pub, "start") # scan full head_p
                sleep(1)
                motion.publisher_(motion.override_orig_pos_lidar_pub,  self.init_head_p) # overide original lidar pose
                self.wait_robot(motion, "Finish head joint in order to make pointcloud")
                
                if self.debug:
                    self.state = None
                else:
                    # self.state = 'pull_pose'
                    self.state = 'walk_mode'

            elif self.state == 'pull_pose':
                rospy.loginfo('[WW] Robot State : {}'.format(self.state))
                self.balance = False
                
                action.motor.publisher_(action.motor.module_control_pub, "none", latch=True)
                # action.set_init_config(torque=50)
                action.play_motion(self.pull_motion)
                self.wait_action()

                if self.debug:
                    self.state = None
                else:
                    self.state = 'walk_mode'

            elif self.state == 'release' or  self.state == 'release_human':
                rospy.loginfo('[WW] Robot State : {}'.format(self.state))
                self.balance = False
                
                action.motor.publisher_(action.motor.module_control_pub, "none", latch=True)
                action.play_motion(self.state)
                self.wait_action()
                self.state = None

            elif self.state == 'walk_mode':
                rospy.loginfo('[WW] Robot State : {}'.format(self.state))
                self.walk_mode = True
                self.balance   = False
                
                walking.publisher_(walking.walking_pub, "set_mode")
                self.wait_robot(walking, "Walking_Module_is_enabled")
                
                if self.debug:
                    self.state = None
                else:
                    # self.state = 'balance_on'
                    self.state = 'update_balance'

            elif self.state == 'balance_on':
                rospy.loginfo('[WW] Robot State : {}'.format(self.state))
                
                if self.walk_mode:
                    self.balance = True

                    if self.save_data:
                        cob_x, cob_y = -0.015, -0.00
                        centre_of_body = { 'x_offset': cob_x, 'y_offset': cob_y}
                        initial_config = {}
                        initial_config['pull_motion']    = 'default'
                        initial_config['centre_of_body'] = centre_of_body
                        with open(self.yaml_file, 'w') as f:
                            yaml.dump(initial_config, f, default_flow_style=False)

                    walking.publisher_(walking.walking_pub, "balance_on")
                    self.wait_robot(walking, "Balance_Param_Setting_Finished")
                    self.wait_robot(walking, "Joint_FeedBack_Gain_Update_Finished")
                else:
                    rospy.logwarn('[WW] Set walk mode first !')

                if self.debug:
                    self.state = None 
                else:
                    self.state = 'walk_backward'

            elif self.state == 'update_balance':
                rospy.loginfo('[WW] Robot State : {}'.format(self.state))
                self.balance = True
                
                self.set_initial_pose()
                self.wait_robot(walking, "Balance_Param_Setting_Finished")

                if self.debug:
                    self.state = None 
                else:
                    self.state = 'walk_backward'

            elif self.state == 'walk_backward':
                rospy.loginfo('[WW] Robot State : {}'.format(self.state))

                if self.balance:
                    walking.walk_command("backward", self.num_step, 0.4, 0.1, 0.05, 5)
                    
                    if self.save_data:
                        self.save_pub.publish(True)  # turn on record robot & tripod
                        self.wait_robot(walking, "Walking_Started")
                        self.state = 'save_data'
                    else:
                        self.state = None
                else:
                    rospy.logwarn('[WW] Turn on balance first !')
                    self.state = None

            elif self.state == 'walk_forward':
                rospy.loginfo('[WW] Robot State : {}'.format(self.state))

                if self.balance:
                    walking.walk_command("forward", 2, 1.0, 0.1, 0.05, 5)
                else:
                    rospy.logwarn('[WW] Turn on balance first !')
                self.state = None

            elif self.state == 'save_data':
                # rospy.loginfo('[WW] Robot State : {}'.format(self.state))

                if walking.status_msg == "Walking_Finished":
                    rospy.loginfo("[WW] Walk Finished")
                    self.save_pub.publish(False)  # turn off record robot & tripod
                    self.state = None

                else:
                    cur_len_real_sense = sensor.real_sense_pcl.shape[0]
                    if cur_len_real_sense != prev_len_real_sense :
                        real_sense_cnt += 1
                        np.savez("{}/wolf_realsense_pcl-{}-{}.npz".format(self.data_path, self.n_folder, real_sense_cnt), pcl=sensor.real_sense_pcl)
                        rospy.loginfo('[WW] Saving realsense pcl data: {}'.format(real_sense_cnt))

                    # save excel
                    rate_cnt += 1
                    self.excel.add_data(no = rate_cnt, \
                                        imu_roll    = sensor.imu_ori['roll'],   imu_pitch    = sensor.imu_ori['pitch'],  imu_yaw  = sensor.imu_ori['yaw'], \
                                        lf_x        = sensor.left_torque['x'],  lf_y         = sensor.left_torque['y'],  lf_z     = sensor.left_torque['z'],  \
                                        rf_x        = sensor.right_torque['x'], rf_y         = sensor.right_torque['y'], rf_z     = sensor.right_torque['z'], \
                                        des_roll    = walking.des_pose_roll,    des_pitch    = walking.des_pose_pitch, \
                                        robot_frame = self.robot_frame,         tripod_frame = self.tripod_frame,        rs_frame = real_sense_cnt)

                    prev_len_real_sense = cur_len_real_sense

            self.main_rate.sleep()

        self.thread1_flag = True
        
if __name__ == '__main__':
    rospy.init_node('pioneer_wolf_walk') #, disable_signals=True)

    # if using ros launch length of sys.argv is 4
    if len(sys.argv) == 4:
        save = sys.argv[1]
        rospy.loginfo("[Wolf] Pioneer Main Wolf Walk - Running")
        rospy.loginfo("[WW] Save Data : {}\n".format(save))

        wolf = Wolf_Walk(save)
        wolf.run()
    else:
        rospy.logerr("[WW] Exit Argument not fulfilled")