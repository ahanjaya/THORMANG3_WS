#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Pose2D, Point32
from sensor_msgs.msg import Image
from std_msgs.msg import Bool

class Yolo_Graph:
    def __init__(self):
        rospy.init_node('pioneer_yolov3', anonymous=False)
        rospy.loginfo("[Yolo] Pioneer YoloV3 Placement")

        # Publisher
        left_arm_pos_pub      = rospy.Publisher("/pioneer/placement/left_arm_point",    Point32, queue_size=1)
        right_arm_pos_pub     = rospy.Publisher("/pioneer/placement/right_arm_point",   Point32, queue_size=1)
        placement_trigger_pub = rospy.Publisher("/pioneer/placement/placement_trigger", Bool,    queue_size=1)
        ini_pose_pub          = rospy.Publisher("/pioneer/placement/init_pose",         Bool,    queue_size=1)
        keyboard_pos_pub      = rospy.Publisher("/pioneer/placement/keyboard_position", Pose2D,  queue_size=1)

        # Subscriber
        rospy.Subscriber("/pioneer/placement/finish_placement",  Bool,   self.finish_placement_callback)
        rospy.Subscriber("/robotis/sensor/camera/image_raw",     Image,  self.image_callback)

    def finish_placement_callback(self, msg):
        pass

    def image_callback(self, msg):
        pass

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    yolo = Yolo_Graph()
    yolo.run()