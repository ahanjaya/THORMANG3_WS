#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
from robotis_controller_msgs.msg import StatusMsg
from thormang3_manipulation_module_msgs.msg import KinematicsPose, KinematicsArrayPose

class Main_Graph:
    def __init__(self):
        rospy.init_node('thormang3_manager', anonymous=False)
        rospy.loginfo("[Main] Thormang Manager - Running")

        # Subscriber
        rospy.Subscriber("/robotis/manipulation/ini_pose_msg",              String,               self.ini_pose_callback)
        rospy.Subscriber("/robotis/manipulation/kinematics_pose_msg",       KinematicsPose,       self.kinematics_pose_callback)
        rospy.Subscriber("/robotis/manipulation/kinematics_pose_arr_msg",   KinematicsArrayPose,  self.kinematics_pose_arr_callback)

        # Publisher
        self.send_ini_pose_msg_pub = rospy.Publisher('/robotis/status',  StatusMsg,  queue_size=10)

    def ini_pose_callback(self, msg):
        pass

    def kinematics_pose_callback(self, msg):
        pass

    def kinematics_pose_arr_callback(self, msg):
        pass

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    main_graph = Main_Graph()
    main_graph.run()