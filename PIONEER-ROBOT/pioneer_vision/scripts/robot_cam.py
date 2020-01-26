#!/usr/bin/env python3

import os
import sys
import cv2
import rospy
import rospkg
import numpy as np
from time import sleep
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, Int16

class Robot_Cam:
    def __init__(self, save):
        rospack             = rospkg.RosPack()
        self.data_path      = rospack.get_path("pioneer_main") + "/data/wolf_walk"
        self.save_data      = False
        self.frame_width    = rospy.get_param("/uvc_camera_center_node/width")
        self.frame_height   = rospy.get_param("/uvc_camera_center_node/height")
        self.source_image   = np.zeros((self.frame_width, self.frame_height, 3), np.uint8)
        self.robot_frame    = 0
        self.main_rate      = rospy.Rate(15)

        self.save           = self.str_to_bool(save)
        if self.save:
            sleep(2)
            n_folder  = len(os.walk(self.data_path).__next__()[1]) - 1
            data_path = "{}/{}".format(self.data_path, n_folder)
        
            cam_file  = "{}/wolf_robot_cam-{}.avi" .format(data_path, n_folder)
            fourcc    = cv2.VideoWriter_fourcc(*'MJPG')
            self.out  = cv2.VideoWriter(cam_file, fourcc, 30, (self.frame_width, self.frame_height))

        ## Publisher
        self.robot_frame_pub = rospy.Publisher('/pioneer/wolf/robot_frame', Int16,  queue_size=10)

        ## Subscriber
        rospy.Subscriber('/robotis/sensor/camera/image_raw', Image,  self.images_callback)
        rospy.Subscriber('/pioneer/wolf/save_data',          Bool,   self.save_data_callback)
    
    def str_to_bool(self, s):
        if s == 'true':
            return True
        elif s == 'false':
            return False
        else:
            raise ValueError # evil ValueError that doesn't tell you what the wrong value was

    def images_callback(self, img):
        dt = np.dtype(np.uint8)
        dt = dt.newbyteorder('>')
        arr = np.frombuffer(img.data,dtype=dt)
        arr = np.reshape(arr, (480, 640, 3))
        self.source_image = cv2.cvtColor(arr, cv2.COLOR_BGR2RGB)

        if self.save_data:
            self.out.write(self.source_image)
            self.robot_frame += 1
            self.robot_frame_pub.publish(self.robot_frame)

    def save_data_callback(self, msg):
        self.save_data = msg.data
        
    def run(self):
        rospy.spin()

        # while not rospy.is_shutdown():
        #     frame = self.source_image.copy()

        #     if self.save_data:
        #         self.out.write(frame)
        #         self.robot_frame += 1
        #         self.robot_frame_pub.publish(self.robot_frame)

        #         # cv2.putText(frame, 'Rec.', (20, 40), cv2.FONT_HERSHEY_TRIPLEX, 0.8, (0, 0, 255), lineType=cv2.LINE_AA)

        #     # cv2.imshow('Robot', frame)
        #     self.main_rate.sleep()

            # ch = 0xFF & cv2.waitKey(1)
            # if ch == 27:
                # break

        # cv2.destroyAllWindows()

if __name__ == '__main__':
    rospy.init_node('pioneer_robot_cam', anonymous=False)

    # if using ros launch length of sys.argv is 4
    if len(sys.argv) == 4:
        save = sys.argv[1]
        rospy.loginfo("[Robot Frame] Pioneer Wolf Robot Frame - Running")
        rospy.loginfo("[Robot Frame] Save Data : {}\n".format(save))

        cam = Robot_Cam(save)
        cam.run()
    else:
        rospy.logerr("[Robot Frame] Exit Argument not fulfilled")