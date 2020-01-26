#!/usr/bin/env python3

import math
import rospy
import threading
import numpy as np 
from time import sleep
from scipy.stats import norm
from std_msgs.msg import String, Bool
from pioneer_utils.utils import *
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose, PoseArray
from robotis_controller_msgs.msg import StatusMsg, SyncWriteItem
from thormang3_manipulation_module_msgs.srv import GetKinematicsPose
from thormang3_manipulation_module_msgs.msg import KinematicsPose, KinematicsArrayPose

class Kinematics:
    def __init__(self):
        # rospy.init_node('pioneer_kinematics', anonymous=False)
        self.pi          = 3.1415
        self.min         = 0
        self.max         = 10
        self.xp          = [self.min, self.max]
        self.fp          = [-self.pi, self.pi]
        self.left_arr, self.right_arr = False, False
        self.left_tra, self.right_tra = False, False
        
        self.pub_rate       = rospy.Rate(10)
        self.thread_rate    = rospy.Rate(30) # 60
        self.module_name    = None
        self.status_msg     = None
        self.thread1_flag   = False
        self.mutex          = threading.Lock()

        ## Publisher
        self.module_control_pub    = rospy.Publisher('/robotis/enable_ctrl_module',                   String,               queue_size=10)
        self.send_ini_pose_msg_pub = rospy.Publisher('/robotis/manipulation/ini_pose_msg',            String,               queue_size=10)
        self.send_ik_msg_pub       = rospy.Publisher('/robotis/manipulation/kinematics_pose_msg',     KinematicsPose,       queue_size=5)
        self.send_ik_arr_msg_pub   = rospy.Publisher('/robotis/manipulation/kinematics_pose_arr_msg', KinematicsArrayPose,  queue_size=5)
        self.set_joint_pub         = rospy.Publisher('/robotis/set_joint_states',                     JointState,           queue_size=10)
        self.sync_write_pub        = rospy.Publisher('/robotis/sync_write_item',                      SyncWriteItem,        queue_size=10)
        self.en_align_key_pub      = rospy.Publisher('/robotis/enable_align_keyboard',                Bool,                 queue_size=1)
        self.en_typing_pub         = rospy.Publisher('/robotis/enable_typing',                        Bool,                 queue_size=1)

        ## Service Client
        self.get_kinematics_pose_client = rospy.ServiceProxy('/robotis/manipulation/get_kinematics_pose', GetKinematicsPose)

        self.read_robot_status()

    def kill_threads(self):
        self.thread1_flag = True

    def thread_read_robot_status(self, stop_thread):
        rospy.Subscriber('/robotis/status', StatusMsg, self.robot_status_callback)
        rospy.spin()

        # while True:
        #     ## Subscriber
        #     rospy.Subscriber('/robotis/status', StatusMsg, self.robot_status_callback)
        #     self.thread_rate.sleep()
        #     if stop_thread():
        #         rospy.loginfo("[Kinematics] Thread killed")
        #         break

    def robot_status_callback(self, msg):
        self.mutex.acquire()
        self.module_name = msg.module_name
        self.status_msg  = msg.status_msg

        if self.status_msg == "Start Left Arm Arr Trajectory":
            self.left_arr = True
        elif self.status_msg == "Finish Left Arm Arr Trajectory":
            self.left_arr = False
        if self.status_msg == "Start Right Arm Arr Trajectory":
            self.right_arr = True
        elif self.status_msg == "Finish Right Arm Arr Trajectory":
            self.right_arr = False

        if self.status_msg == "Start Left Arm Trajectory":
            self.left_tra = True
        elif self.status_msg == "End Left Arm Trajectory":
            self.left_tra = False
        if self.status_msg == "Start Right Arm Trajectory":
            self.right_tra = True
        elif self.status_msg == "End Right Arm Trajectory":
            self.right_tra = False
        # rospy.loginfo(self.status_msg)
        self.mutex.release()

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

    def get_kinematics_pose(self, group_name):
        rospy.wait_for_service('/robotis/manipulation/get_kinematics_pose')
        try:
            resp = self.get_kinematics_pose_client(group_name)
            euler_rad = quaternion_to_euler( resp.group_pose.orientation.x, resp.group_pose.orientation.y,
                                             resp.group_pose.orientation.z, resp.group_pose.orientation.w )
            euler_deg = np.degrees(euler_rad)
            return {'x'     : resp.group_pose.position.x,
                    'y'     : resp.group_pose.position.y,
                    'z'     : resp.group_pose.position.z,              
                    'roll'  : euler_deg[0],
                    'pitch' : euler_deg[1],
                    'yaw'   : euler_deg[2] }
        except rospy.ServiceException as e: # python3
            print ("Service call failed: %s" %e)

    def set_kinematics_pose(self, group_name, time, **data):
        msg                 = KinematicsPose()
        msg.name            = group_name
        msg.pose.position.x = data.get('x')
        msg.pose.position.y = data.get('y')
        msg.pose.position.z = data.get('z')

        roll    = np.radians( data.get('roll') )
        pitch   = np.radians( data.get('pitch') )
        yaw     = np.radians( data.get('yaw') )
        quaternion = euler_to_quaternion(roll, pitch, yaw)
        msg.pose.orientation.x = quaternion[0]
        msg.pose.orientation.y = quaternion[1]
        msg.pose.orientation.z = quaternion[2]
        msg.pose.orientation.w = quaternion[3]
        msg.time = time

        self.publisher_(self.send_ik_msg_pub, msg)

    def set_kinematics_arr_pose(self, group_name, time, **data):
        msg                 = KinematicsArrayPose()
        msg.name            = group_name

        for i in range(data.get('total')):
            geo_pose = Pose()
            geo_pose.position.x = data.get('x')[i]
            geo_pose.position.y = data.get('y')[i]
            geo_pose.position.z = data.get('z')[i]

            roll    = np.radians( data.get('roll')[i] )
            pitch   = np.radians( data.get('pitch')[i] )
            yaw     = np.radians( data.get('yaw')[i] )

            quaternion = euler_to_quaternion(roll, pitch, yaw)
            geo_pose.orientation.x = quaternion[0]
            geo_pose.orientation.y = quaternion[1]
            geo_pose.orientation.z = quaternion[2]
            geo_pose.orientation.w = quaternion[3]

            msg.pose.poses.append(geo_pose)

        msg.time = time
        self.publisher_(self.send_ik_arr_msg_pub, msg)

    def trajectory_sin(self, group, x, y, z, roll, pitch, yaw, xc, yc, zc, time, res):
        # get current pose
        cur     = self.get_kinematics_pose(group)
        nround  = 2
        nums    = int(time/res)
        t       = np.linspace(0.0, np.pi, num=nums)
        s       = np.sin(t)

        x_tar   = np.round(x, nround)
        y_tar   = np.round(y, nround)
        z_tar   = np.round(z, nround)
        x_cur   = np.round(cur.get('x'), nround)
        y_cur   = np.round(cur.get('y'), nround)
        z_cur   = np.round(cur.get('z'), nround)

        ## 1st rule (direct movement)
        if y_tar == y_cur and z_tar == z_cur: # x movement
            x_d = np.interp( t, [0, np.pi], [x_cur, x_tar])
            y_d = np.interp( s, [0, 1],     [y_cur, y_cur + yc])
            z_d = np.interp( s, [0, 1],     [z_cur, z_cur + zc])

        elif x_tar == x_cur and z_tar == z_cur: # y_movement
            x_d = np.interp( s, [0, 1],     [x_cur, x_cur + xc])
            y_d = np.interp( t, [0, np.pi], [y_cur, y_tar])
            z_d = np.interp( s, [0, 1],     [z_cur, z_cur + zc])
        
        elif x_tar == x_cur and y_tar == y_cur: # z_movement
            x_d = np.interp( s, [0, 1],     [x_cur, x_cur + xc])
            y_d = np.interp( s, [0, 1],     [y_cur, y_cur + yc])
            z_d = np.interp( t, [0, np.pi], [z_cur, z_tar])

        ## 2nd rule (omni movement)
        else:
            x_d = np.linspace(x_cur, x_tar, num=nums) + np.interp( s, [0, 1], [0, xc])
            y_d = np.linspace(y_cur, y_tar, num=nums) + np.interp( s, [0, 1], [0, yc])
            z_d = np.linspace(z_cur, z_tar, num=nums) + np.interp( s, [0, 1], [0, zc])

        roll_d  = np.full( nums, roll )
        pitch_d = np.full( nums, pitch )
        yaw_d   = np.full( nums, yaw )

        # np.set_printoptions(suppress=True)
        # print("x_d: ", x_d)
        # print("y_d: ", y_d)
        # print("z_d: ", z_d)
        self.set_kinematics_arr_pose(group, res , **{ 'total': nums, 'x': x_d, 'y': y_d, 'z': z_d, 'roll': roll_d, 'pitch': pitch_d, 'yaw': yaw_d })

    def trajectory_gaussian(self, group, x, y, z, roll, pitch, yaw, mu, sig, xc, yc, zc, time, res):
        # get current pose
        cur     = self.get_kinematics_pose(group)
        nround  = 2
        nums    = int(time/res)
        t       = np.linspace(-5, 5, num=time/res)
        s       = norm.pdf(t, mu, sig)

        x_tar   = np.round(x, nround)
        y_tar   = np.round(y, nround)
        z_tar   = np.round(z, nround)
        x_cur   = np.round(cur.get('x'), nround)
        y_cur   = np.round(cur.get('y'), nround)
        z_cur   = np.round(cur.get('z'), nround)

        ## 1st rule (direct movement)
        if y_tar == y_cur and z_tar == z_cur: # x movement
            x_d = np.interp( t, [-5, 5], [x_cur, x_tar])
            y_d = np.interp( s, [0, 1],     [y_cur, y_cur + yc])
            z_d = np.interp( s, [0, 1],     [z_cur, z_cur + zc])

        elif x_tar == x_cur and z_tar == z_cur: # y_movement
            x_d = np.interp( s, [0, 1],     [x_cur, x_cur + xc])
            y_d = np.interp( t, [-5, 5], [y_cur, y_tar])
            z_d = np.interp( s, [0, 1],     [z_cur, z_cur + zc])
        
        elif x_tar == x_cur and y_tar == y_cur: # z_movement
            x_d = np.interp( s, [0, 1],     [x_cur, x_cur + xc])
            y_d = np.interp( s, [0, 1],     [y_cur, y_cur + yc])
            z_d = np.interp( t, [-5, 5], [z_cur, z_tar])

        ## 2nd rule (omni movement)
        else:
            x_d = np.linspace(x_cur, x_tar, num=nums) + np.interp( s, [0, 1], [0, xc])
            y_d = np.linspace(y_cur, y_tar, num=nums) + np.interp( s, [0, 1], [0, yc])
            z_d = np.linspace(z_cur, z_tar, num=nums) + np.interp( s, [0, 1], [0, zc])

        roll_d  = np.full( nums, roll )
        pitch_d = np.full( nums, pitch )
        yaw_d   = np.full( nums, yaw )

        # np.set_printoptions(suppress=True)
        # print("x_d: ", x_d)
        # print("y_d: ", y_d)
        # print("z_d: ", z_d)
        self.set_kinematics_arr_pose(group, res , **{ 'total': nums, 'x': x_d, 'y': y_d, 'z': z_d, 'roll': roll_d, 'pitch': pitch_d, 'yaw': yaw_d })

    def limiter(self, value):
        if value >= self.max:
            return self.max
        elif value <= self.min:
            return self.min
        else:
            return value

    def set_gripper(self, group_name, group_value, thumb_y_value):
        group_value   = self.limiter(group_value)
        thumb_y_value = self.limiter(thumb_y_value)
        
        if group_name == "left_arm":
            joint           = JointState()
            joint.name      = ['l_arm_thumb_p', 'l_arm_index_p', 'l_arm_middle_p', 'l_arm_finger45_p']
            joint.position  = [ np.interp(group_value, self.xp, self.fp)  for _ in range(len(joint.name))]
            
            joint.name.append('l_arm_thumb_y')
            joint.position.append( np.interp(thumb_y_value, self.xp, self.fp) )
            
            joint.velocity  = [ 0 for _ in range(len(joint.name)+1)]
            joint.effort    = [ 0 for _ in range(len(joint.name)+1)]
            self.publisher_(self.set_joint_pub, joint)
            # rospy.loginfo("[Kinematics] {0} gripper: {1}, thumb_yaw: {2}".format(group_name, group_value, thumb_y_value))

        elif group_name == "right_arm":
            joint           = JointState()
            joint.name      = ['r_arm_thumb_p', 'r_arm_index_p', 'r_arm_middle_p', 'r_arm_finger45_p']
            joint.position  = [ np.interp(group_value, self.xp, self.fp)  for _ in range(len(joint.name))]
            
            joint.name.append('r_arm_thumb_y')
            joint.position.append( np.interp(thumb_y_value, self.xp, self.fp) )
            
            joint.velocity  = [ 0 for _ in range(len(joint.name)+1)]
            joint.effort    = [ 0 for _ in range(len(joint.name)+1)]
            self.publisher_(self.set_joint_pub, joint)
            # rospy.loginfo("[Kinematics] {0} gripper: {1}, thumb_yaw: {2}".format(group_name, group_value, thumb_y_value))

        else:
            rospy.logerr("[Kinematics] Set gripper: {0} unknown name".format(joint_name))

    def set_joint_pos(self, joint_name, joint_pose_deg):
        if len(joint_name) == len(joint_pose_deg):       
            joint           = JointState()
            joint.name      = joint_name
            joint.position  = np.radians(joint_pose_deg)
            joint.velocity  = [ 0 for _ in range(len(joint.name))]
            joint.effort    = [ 0 for _ in range(len(joint.name))]
            self.publisher_(self.set_joint_pub, joint, latch=False)
            # rospy.loginfo('[Kinematics] Joint name: {0} \t Pos: {1}'.format(joint.name, joint.position))
        else:
            rospy.logerr("[Kinematics] joint_name and joint_pose are not equal")

    def set_joint_torque(self, joint_name, torque):
        sync_write           = SyncWriteItem()
        sync_write.item_name = "goal_torque"

        if joint_name[0] == "all":
            sync_write.joint_name = ["head_p", "head_y", "torso_y", 
                                    "l_arm_el_y", "l_arm_sh_p1", "l_arm_sh_p2", "l_arm_sh_r", 
                                    "r_arm_el_y", "r_arm_sh_p1", "r_arm_sh_p2", "r_arm_sh_r"]
        elif joint_name[0] == "hand":
            sync_write.joint_name = ["l_arm_el_y", "l_arm_sh_p1", "l_arm_sh_p2", "l_arm_sh_r", 
                                    "r_arm_el_y", "r_arm_sh_p1", "r_arm_sh_p2", "r_arm_sh_r"]
        else:
            sync_write.joint_name = joint_name

        sync_write.value = [ int(np.interp(torque, [0, 100], [0, 310])) for _ in range(len(sync_write.joint_name)) ]
        # rospy.loginfo('[Kinematics] Joint name: {0} \t Torque: {1}'.format(sync_write.joint_name, sync_write.value))
        self.publisher_(self.sync_write_pub, sync_write)