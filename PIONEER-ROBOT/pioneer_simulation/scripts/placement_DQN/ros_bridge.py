#!/usr/bin/env python3

import rospy
import numpy as np
from time import sleep
from std_msgs.msg import Bool
from geometry_msgs.msg import Pose2D
from pioneer_simulation.msg import Pose2DArray

class Placement_Simulation:
    def __init__(self):
        rospy.init_node('pioneer_placement_simulation', anonymous=False)
        rospy.loginfo("[PS] Pioneer Placement Simulation- Running")

        self.total_points      = 100
        self.main_rate         = rospy.Rate(60)
        self.keyboard_position = {'x': None, 'y': None, 'theta': None}
        self.received          = False
        self.save_frame        = False
        self.shutdown          = False

        # Publisher
        self.left_arm_points_pub  = rospy.Publisher("/pioneer/placement/left_arm_arr_points",  Pose2DArray, queue_size=1)
        self.right_arm_points_pub = rospy.Publisher("/pioneer/placement/right_arm_arr_points", Pose2DArray, queue_size=1)
        self.final_pose_pub       = rospy.Publisher("/pioneer/placement/final_pose",           Pose2D,      queue_size=1)

        # Subscriber
        rospy.Subscriber("/pioneer/placement/keyboard_position", Pose2D, self.keyboard_position_callback)
        rospy.Subscriber("/pioneer/placement/save_frame",        Bool,   self.save_frame_callback)
        rospy.Subscriber("/pioneer/placement/shutdown_signal",   Bool,   self.shutdown_callback)

    def shutdown_callback(self, msg):
        self.shutdown = True
        rospy.signal_shutdown('Exit')

    def keyboard_position_callback(self, msg):
        self.keyboard_position['x']     = msg.x
        self.keyboard_position['y']     = msg.y
        self.keyboard_position['theta'] = msg.theta
        self.received                   = True

        rospy.loginfo("[PS] Recv. keyboard position (X: {}, Y: {}, Theta: {:.2f})" .format(msg.x, msg.y, msg.theta))

    def pack_points(self, group, num_point, x, y):
        points      = Pose2DArray()
        points.name = group

        for i in range (num_point):
            point   = Pose2D()
            point.x = x[i]
            point.y = y[i]
            points.poses.append(point)
        return points

    def send_trajectory(self, left_arm_points, right_arm_points):
        self.left_arm_points_pub.publish(left_arm_points)
        self.right_arm_points_pub.publish(right_arm_points)

    def send_final_pose(self, **keyboard_final_pose):        
        pose       = Pose2D()
        pose.x     = keyboard_final_pose['x']  
        pose.y     = keyboard_final_pose['y']  
        pose.theta = keyboard_final_pose['theta']  

        rospy.loginfo("[PS] Final keyboard position (X: {}, Y: {}, Theta: {:.2f})".format(pose.x, pose.y, pose.theta))
        self.final_pose_pub.publish(pose)

    def save_frame_callback(self, msg):
        self.save_frame = msg.data
