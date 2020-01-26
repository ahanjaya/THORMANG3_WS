#!/usr/bin/env python3

import math
import rospy
import threading
import numpy as np
from time import sleep
from multipledispatch import dispatch
from sensor_msgs.msg import JointState
from std_msgs.msg import String, Bool, Float64
from robotis_controller_msgs.msg import SyncWriteItem

class Motor:
    def __init__(self, robot_name):
        # rospy.init_node('pioneer_motor', anonymous=False)
        self.robot_name     = robot_name

        self.pub_rate       = rospy.Rate(10)
        self.thread_rate    = rospy.Rate(10) #60 lower is slower, higher is faster
        self.joint_position = {}
        self.joint_velocity = {}
        self.joint_effort   = {}

        self.goal_position  = {}
        self.goal_velocity  = {}
        self.goal_effort    = {}
        self.thread1_flag   = False
        self.curr_pos       = None
        self.last_pos       = None
        self.moving         = False
        self.mutex          = threading.Lock()

        ## publish
        self.module_control_pub = rospy.Publisher('/robotis/enable_ctrl_module',    String,        queue_size=10) #, latch=True)
        self.set_joint_pub      = rospy.Publisher('/robotis/set_joint_states',      JointState,    queue_size=10) #, latch=True)
        self.sync_write_pub     = rospy.Publisher('/robotis/sync_write_item',       SyncWriteItem, queue_size=10) #, latch=True)
        self.status_pub         = rospy.Publisher('/robotis/direct_control/status', Bool,          queue_size=10) #, latch=True)
        self.status_val_pub     = rospy.Publisher('/robotis/direct_control/value',  Float64,       queue_size=10) #, latch=True)

        self.read_dynamixel()

    def kill_threads(self):
        self.thread1_flag = True
        
    def thread_read_dynamixel(self, stop_thread):
        rospy.Subscriber('/robotis/present_joint_states', JointState, self.present_joint_states_callback)
        rospy.spin()

    def thread_robot_status(self, stop_thread):

        while not rospy.is_shutdown():
            self.curr_pos = list(self.joint_position.values())
            if self.curr_pos != self.last_pos:
                self.moving = True
            else:
                self.moving = False

            self.publisher_(self.status_pub, self.moving, latch=False)
            self.last_pos = self.curr_pos
            if stop_thread():
                rospy.loginfo("[Motor] Thread killed")
                break
            self.thread_rate.sleep()

    def read_dynamixel(self):
        thread1 = threading.Thread(target = self.thread_read_dynamixel, args =(lambda : self.thread1_flag, )) 
        thread1.start()

        thread2 = threading.Thread(target = self.thread_robot_status, args =(lambda : self.thread1_flag, )) 
        thread2.start()

    def present_joint_states_callback(self, msg):
        self.mutex.acquire()
        self.joint_position = dict(zip( msg.name, np.around( np.degrees(msg.position),2).tolist() ))
        self.joint_velocity = dict(zip( msg.name, msg.velocity) )
        self.joint_effort   = dict(zip( msg.name, msg.effort) )
        self.mutex.release()

    def goal_joint_states_callback(self, msg):
        self.mutex.acquire()
        self.goal_velocity = dict(zip(msg.name, msg.velocity))
        self.goal_effort   = dict(zip(msg.name, msg.effort))
        self.mutex.release()

    def publisher_(self, topic, msg, latch=False):
        if latch:
            for i in range(4):
                topic.publish(msg)
                self.pub_rate.sleep()
        else:
            topic.publish(msg)

    @dispatch(list, list)
    def set_joint_states(self, joint_name, joint_pose_deg):
        '''
        Set Position
        '''
        if len(joint_name) == len(joint_pose_deg):
            joint           = JointState()
            joint.name      = joint_name
            joint.position  = np.radians(joint_pose_deg)
            joint.velocity  = [ 0 for _ in joint_name ]
            joint.effort    = [ 0 for _ in joint_name ]

            # joint.velocity  = [ self.goal_velocity.get(_) for _ in joint_name ]
            # joint.effort    = [ self.goal_effort.get(_)   for _ in joint_name]
            self.publisher_(self.set_joint_pub, joint, latch=False)
            # rospy.loginfo('Joint name: {0} \t Pos: {1}'.format(joint.name, joint.position))
        else:
            rospy.logerr("[Motor] Length set_joint_states (position) not equal")

    @dispatch(list, list, list)
    def set_joint_states(self, joint_name, joint_pose_deg, joint_speed):
        '''
        Set Position and Speed
        
        Velocity Note:
        1) H54-100-S500-R
            http://emanual.robotis.com/docs/en/dxl/pro/h54-100-s500-r/ --> velocity limit : 17000
            H54-100-S500-R --> velocity_to_value_ratio = 4793.01226

            Equation set velocity by rostopic set_joint_state:
                max = 17000/4793.01226
                    = 3.546830068

        2) H42-20-S300-R
            emanual.robotis.com/docs/en/dxl/pro/h42-20-s300-r/ --> velocity limit : 10300
            H54-100-S500-R --> velocity_to_value_ratio = 2900.59884

            Equation set velocity by rostopic set_joint_state:
                max = 10300/2900.59884
                    = 3.550990871

        '''
        if ( len(joint_name) == len(joint_pose_deg) and \
             len(joint_name) == len(joint_speed) ):

            joint           = JointState()
            joint.name      = joint_name
            joint.position  = np.radians(joint_pose_deg)
            joint.velocity  = [      np.interp(joint_speed[i], [0, 100], [0, 3.550990871]) if joint_name[i]=="head_p" or joint_name[i]=="head_y" \
                                else np.interp(joint_speed[i], [0, 100], [0, 3.546830068]) for i in range(len(joint_name)) ]
            joint.effort    = [ 0 for _ in joint_name ]

            # joint.velocity  = joint_speed
            # joint.effort    = [ self.goal_effort.get(_) for _ in joint_name ]
            self.publisher_(self.set_joint_pub, joint)
            # rospy.loginfo('Joint name: {0} \t Pos: {1} \t Speed: {2}'.format(joint.name, joint.position, joint.velocity))
        else:
            rospy.logerr("[Motor] Length set_joint_states (position, speed) not equal")

    @dispatch(list, list, list, list)
    def set_joint_states(self, joint_name, joint_pose_deg, joint_speed, joint_torque):
        '''
        Set Position, Speed, Torque

        Torque Note:
        1) H54-100-S500-R
            dxl_init.yaml  --> torque_limit : 310
            H54-100-S500-R --> torque_to_current_value_ratio = 9.66026

            Equation set torque by rostopic set_joint_state:
                max = 310/9.66026
                    = 32.090233596

        2) H42-20-S300-R
            dxl_init.yaml  --> torque_limit : 372
            H54-100-S500-R --> torque_to_current_value_ratio = 27.15146

            Equation set torque by rostopic set_joint_state:
                max = 372/27.15146
                    = 13.700920687
        '''
        if ( len(joint_name) == len(joint_pose_deg)  and \
             len(joint_name) == len(joint_speed) and \
             len(joint_name) == len(joint_torque) ):

            joint           = JointState()
            joint.name      = joint_name
            joint.position  = np.radians(joint_pose_deg)
            joint.velocity  = [      np.interp(joint_speed[i], [0, 100], [0, 3.550990871]) if joint_name[i]=="head_p" or joint_name[i]=="head_y" \
                                else np.interp(joint_speed[i], [0, 100], [0, 3.546830068]) for i in range(len(joint_name)) ]

            joint.effort    = [      np.interp(joint_torque[i], [0, 100], [0, 13.700920687]) if joint_name[i]=="head_p" or joint_name[i]=="head_y" \
                                else np.interp(joint_torque[i], [0, 100], [0, 32.090233596]) for i in range(len(joint_name)) ]

            # joint.velocity  = joint_speed
            # joint.effort    = joint_torque

            self.publisher_(self.set_joint_pub, joint)
            # rospy.loginfo('Joint name: {0} \t Pos: {1} \t Speed: {2} \t Torque: {3}'.format(joint.name, joint.position, joint.velocity, joint.effort))
        else:
            rospy.logerr("[Motor] Length set_joint_states (position, speed, torque) not equal")

    @dispatch(list, bool)
    def set_joint_states(self, joint_name, torque):
        '''
        Enable/Disable Torque
        '''
        sync_write           = SyncWriteItem()
        sync_write.item_name = "torque_enable"

        if self.robot_name  == "Thormang3_Wolf" :   # Thormang3 Full size
            if joint_name[0] == "all":
                joint_name = [ "l_arm_el_y", "l_arm_sh_p1", "l_arm_sh_p2", "l_arm_sh_r", "l_arm_wr_r", "l_arm_wr_y", "l_arm_wr_p", "l_arm_grip",
                               "r_arm_el_y", "r_arm_sh_p1", "r_arm_sh_p2", "r_arm_sh_r", "r_arm_wr_r", "r_arm_wr_y", "r_arm_wr_p", "r_arm_grip" ]

            elif joint_name[0] == "left_arm":
                joint_name = [ "l_arm_el_y", "l_arm_sh_p1", "l_arm_sh_p2", "l_arm_sh_r", "l_arm_wr_r", "l_arm_wr_y", "l_arm_wr_p", "l_arm_grip" ]

            elif joint_name[0] == "right_arm":
                joint_name = [ "r_arm_el_y", "r_arm_sh_p1", "r_arm_sh_p2", "r_arm_sh_r", "r_arm_wr_r", "r_arm_wr_y", "r_arm_wr_p", "r_arm_grip" ]

        elif self.robot_name  == "Thormang3_Bear" : # Thormang3 Upper Body
            if joint_name[0] == "all":
                joint_name = [ "head_p", "head_y", "torso_y", 
                                "l_arm_el_y", "l_arm_sh_p1", "l_arm_sh_p2", "l_arm_sh_r", "l_arm_wr_r", "l_arm_wr_y", "l_arm_wr_p", 
                                "r_arm_el_y", "r_arm_sh_p1", "r_arm_sh_p2", "r_arm_sh_r", "r_arm_wr_r", "r_arm_wr_y", "r_arm_wr_p" ]

        sync_write.joint_name = joint_name
        sync_write.value      = [ torque for _ in range(len(sync_write.joint_name )) ]

        # turn off torque
        if not torque: 
            self.publisher_(self.sync_write_pub, sync_write)
        # turn on torque
        else:
            joint           = JointState()
            joint.name      = joint_name

            self.mutex.acquire()
            joint.position  = [ np.radians(self.joint_position.get(_)) for _ in joint_name ] # read present position
            self.mutex.release()

            joint.velocity  = [ 0 for _ in joint_name ]
            joint.effort    = [ 0 for _ in joint_name ]

            # torque          = 2 # 0 default
            # joint.effort    = [      np.interp(torque, [0, 100], [0, 13.700920687]) if joint=="head_p" or joint=="head_y" \
            #                     else np.interp(torque, [0, 100], [0, 32.090233596]) for joint in joint_name ]
            self.publisher_(self.set_joint_pub, joint)        # set present position
            self.publisher_(self.sync_write_pub, sync_write)  # turn on torque
        # rospy.loginfo('Joint name: {0} \t Torque: {1}'.format(joint_name, sync_write.value))

    def set_joint_torque(self, joint_name, torque):
        sync_write           = SyncWriteItem()
        sync_write.item_name = "goal_torque"

        if joint_name[0] == "all":
            if self.robot_name  == "Thormang3_Wolf" :   # Thormang3 Full size
                joint_name = [ "l_arm_el_y", "l_arm_sh_p1", "l_arm_sh_p2", "l_arm_sh_r", "l_arm_wr_r", "l_arm_wr_y", "l_arm_wr_p", "l_arm_grip",
                               "r_arm_el_y", "r_arm_sh_p1", "r_arm_sh_p2", "r_arm_sh_r", "r_arm_wr_r", "r_arm_wr_y", "r_arm_wr_p", "r_arm_grip" ]
        
            elif self.robot_name  == "Thormang3_Bear" : # Thormang3 Upper Body
                joint_name = [  "head_p", "head_y", "torso_y", 
                                "l_arm_el_y", "l_arm_sh_p1", "l_arm_sh_p2", "l_arm_sh_r", 
                                "r_arm_el_y", "r_arm_sh_p1", "r_arm_sh_p2", "r_arm_sh_r"]

        sync_write.joint_name = joint_name
        sync_write.value = [ int(np.interp(torque, [0, 100], [0, 372])) if joint_name[i]=="head_p" or joint_name[i]=="head_y" \
                        else int(np.interp(torque, [0, 100], [0, 310])) for i in range(len(joint_name)) ]
        # sync_write.value = [ int(np.interp(torque, [0, 100], [0, 310])) for _ in range(len(joint_name)) ]

        # rospy.loginfo('Joint name: {0} \t Torque: {1}'.format(sync_write.joint_name, sync_write.value))
        self.publisher_(self.sync_write_pub, sync_write)

    def set_joint_velocity(self, joint_name, velocity):
        sync_write           = SyncWriteItem()
        sync_write.item_name = "goal_velocity"

        if joint_name[0] == "all":
            if self.robot_name  == "Thormang3_Wolf" :   # Thormang3 Full size
                joint_name = [  "l_arm_el_y", "l_arm_sh_p1", "l_arm_sh_p2", "l_arm_sh_r", "l_arm_wr_r", "l_arm_wr_y", "l_arm_wr_p",  
                                "r_arm_el_y", "r_arm_sh_p1", "r_arm_sh_p2", "r_arm_sh_r", "r_arm_wr_r", "r_arm_wr_y", "r_arm_wr_p",]
        
            elif self.robot_name  == "Thormang3_Bear" : # Thormang3 Upper Body
                joint_name = [  "head_p", "head_y", "torso_y", 
                                "l_arm_el_y", "l_arm_sh_p1", "l_arm_sh_p2", "l_arm_sh_r", 
                                "r_arm_el_y", "r_arm_sh_p1", "r_arm_sh_p2", "r_arm_sh_r"]

        sync_write.joint_name = joint_name
        sync_write.value = [ int(np.interp(velocity, [0, 100], [0, 10300])) if joint_name[i]=="head_p" or joint_name[i]=="head_y" \
                        else int(np.interp(velocity, [0, 100], [0, 17000])) for i in range(len(joint_name)) ]

        # rospy.loginfo('Joint name: {0} \t Torque: {1}'.format(sync_write.joint_name, sync_write.value))
        self.publisher_(self.sync_write_pub, sync_write)