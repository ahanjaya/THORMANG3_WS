#!/usr/bin/env python3

import os
import rospy
import rospkg
import cv2
import pyautogui
import numpy as np
from time import sleep
from time import perf_counter

class Screen_Record:
    def __init__(self):
        rospy.init_node('pioneer_screen_record', anonymous=False)
        rospy.loginfo("[SS] Pioneer Wolf Screen Record - Running")
        rospack        = rospkg.RosPack()
        self.data_path = rospack.get_path("pioneer_main") + "/data/wolf_walk"

        sleep(2)
        n_folder       = len(os.walk(self.data_path).__next__()[1]) - 1
        data_path      = "{}/{}".format(self.data_path, n_folder)
        cam_file       = "{}/wolf_screen_cam-{}.avi" .format(data_path, n_folder)
        fourcc         = cv2.VideoWriter_fourcc(*'MJPG')
        screen_size    = (3840, 1080)
        self.out       = cv2.VideoWriter(cam_file, fourcc, 30.0, screen_size)
        self.main_rate = rospy.Rate(30)

    def run(self):

        while not rospy.is_shutdown():
            img   = pyautogui.screenshot()
            frame = np.array(img)
            # print(frame.shape)
            frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            self.out.write(frame)

            # ch = 0xFF & cv2.waitKey(1)
            # if ch == 27:
            #     break
            self.main_rate.sleep()

        self.out.release()

if __name__ == '__main__':
    ss = Screen_Record()
    ss.run()