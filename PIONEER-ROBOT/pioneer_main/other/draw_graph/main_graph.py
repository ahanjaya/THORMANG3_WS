#!/usr/bin/env python3

import rospy
from std_msgs.msg import Bool, String
from pioneer_simulation.msg import Pose2DArray
from robotis_controller_msgs.msg import StatusMsg
from thormang3_manipulation_module_msgs.msg import KinematicsPose, KinematicsArrayPose

# from pioneer_kinematics.kinematics import Kinematics

class Main_Graph:
    def __init__(self):
        rospy.init_node('pioneer_main', anonymous=False)
        rospy.loginfo("[Main] Pioneer Placement Keyboard - Running")

        # Subscriber
        rospy.Subscriber("/pioneer/placement/placement_trigger",    Bool,        self.placement_trigger_callback)
        rospy.Subscriber("/pioneer/placement/init_pose",            Bool,        self.ini_pose_callback)
        rospy.Subscriber("/pioneer/placement/left_arm_arr_points",  Pose2DArray, self.left_arm_arr_points_callback)
        rospy.Subscriber("/pioneer/placement/right_arm_arr_points", Pose2DArray, self.right_arm_arr_points_callback)
        rospy.Subscriber('/robotis/status',                         StatusMsg,   self.robot_status_callback)

        # Publisher
        self.finish_placement_pub  = rospy.Publisher("/pioneer/placement/finish_placement",           Bool,                 queue_size=1)
        self.send_ini_pose_msg_pub = rospy.Publisher('/robotis/manipulation/ini_pose_msg',            String,               queue_size=10)
        self.send_ik_msg_pub       = rospy.Publisher('/robotis/manipulation/kinematics_pose_msg',     KinematicsPose,       queue_size=5)
        self.send_ik_arr_msg_pub   = rospy.Publisher('/robotis/manipulation/kinematics_pose_arr_msg', KinematicsArrayPose,  queue_size=5)

    def placement_trigger_callback(self, msg):
        pass

    def ini_pose_callback(self, msg):
        pass
        
    def left_arm_arr_points_callback(self, msg):
        pass

    def right_arm_arr_points_callback(self, msg):
        pass

    def robot_status_callback(self, msg):
        pass

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    main_graph = Main_Graph()
    main_graph.run()