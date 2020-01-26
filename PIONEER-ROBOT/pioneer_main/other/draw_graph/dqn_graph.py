#!/usr/bin/env python3

import rospy
from std_msgs.msg import Bool
from geometry_msgs.msg import Pose2D
from pioneer_simulation.msg import Pose2DArray

class DQN:
    def __init__(self):
        rospy.init_node('pioneer_DQN', anonymous=False)
        rospy.loginfo("[PS] Pioneer Placement Simulation- Running")

        # Publisher
        self.left_arm_points_pub  = rospy.Publisher("/pioneer/placement/left_arm_arr_points",  Pose2DArray, queue_size=1)
        self.right_arm_points_pub = rospy.Publisher("/pioneer/placement/right_arm_arr_points", Pose2DArray, queue_size=1)

        # Subscriber
        rospy.Subscriber("/pioneer/placement/keyboard_position", Pose2D, self.keyboard_position_callback)

    def keyboard_position_callback(self, msg):
        pass

    def send_trajectory(self, left_arm_points, right_arm_points):
        self.left_arm_points_pub.publish(left_arm_points)
        self.right_arm_points_pub.publish(right_arm_points)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    dqn = DQN()
    dqn.run()
