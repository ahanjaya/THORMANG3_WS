#!/usr/bin/env python3

import os
import sys
import cv2
import rospy
import rospkg
import numpy as np
from time import sleep
from std_msgs.msg import Bool, Int16

class Tripod:
    def __init__(self, save):
        self.save         = self.str_to_bool(save)
        rospack           = rospkg.RosPack()
        self.data_path    = rospack.get_path("pioneer_main") + "/data/wolf_walk"
        self.save_data    = False
        self.tripod_frame = 0

        self.cap = cv2.VideoCapture(0)
        self.cap.set(3, 800)
        self.cap.set(4, 600)

        self.frame_width  = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        self.frame_height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))

        if self.save:
            sleep(2)
            n_folder  = len(os.walk(self.data_path).__next__()[1]) - 1
            data_path = "{}/{}".format(self.data_path, n_folder)
            
            cam_file  = "{}/wolf_tripod_cam-{}.avi" .format(data_path, n_folder)
            fourcc    = cv2.VideoWriter_fourcc(*'MJPG')
            self.out  = cv2.VideoWriter(cam_file, fourcc, 30, (self.frame_width, self.frame_height))

        ## Publisher
        self.tripod_frame_pub = rospy.Publisher('/pioneer/wolf/tripod_frame', Int16,  queue_size=10)

        ## Subscriber
        rospy.Subscriber('/pioneer/wolf/save_data', Bool, self.save_data_callback)

    def str_to_bool(self, s):
        if s == 'true':
            return True
        elif s == 'false':
            return False
        else:
            raise ValueError # evil ValueError that doesn't tell you what the wrong value was

    def save_data_callback(self, msg):
        self.save_data = msg.data
        
    def run(self):
        while not rospy.is_shutdown():
            _, frame = self.cap.read()

            if self.save_data:
                self.out.write(frame)
                self.tripod_frame += 1
                self.tripod_frame_pub.publish(self.tripod_frame)

                cv2.putText(frame, 'Rec.', (20, 40), cv2.FONT_HERSHEY_TRIPLEX, 0.8, (0, 0, 255), lineType=cv2.LINE_AA)

            cv2.imshow('Tripod', frame)
            ch = 0xFF & cv2.waitKey(1)
            if ch == 27:
                break

        self.cap.release()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    rospy.init_node('pioneer_tripod', anonymous=False)
    
    # if using ros launch length of sys.argv is 4
    if len(sys.argv) == 4:
        save = sys.argv[1]
        rospy.loginfo("[Tripod] Pioneer Wolf Tripod - Running")
        rospy.loginfo("[Tripod] Save Data : {}\n".format(save))

        tripod = Tripod(save)
        tripod.run()
    else:
        rospy.logerr("[Tripod] Exit Argument not fulfilled")