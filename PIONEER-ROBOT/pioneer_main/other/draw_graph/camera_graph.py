#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image

class Camera_Graph:
    def __init__(self):
        rospy.init_node('uvc_camera', anonymous=False)
        rospy.loginfo("[Main] Camera - Running")

        # Publisher
        self.image_pub = rospy.Publisher('/robotis/sensor/camera/image_raw',  Image,  queue_size=10)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    camera = Camera_Graph()
    camera.run()