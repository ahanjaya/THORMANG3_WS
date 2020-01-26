#!/usr/bin/env python3

import cv2
import rospkg
import pickle
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.image as mpimg

class Calibration:
    def __init__(self):
        rospack         = rospkg.RosPack()
        self.calib_path = rospack.get_path("pioneer_vision") + "/data/"
   
    def cal_undistort(self, img, mtx, dist):
        undist = cv2.undistort(img, mtx, dist, None, mtx)
        return undist
   
    def run(self):
        # Read in the saved objpoints and imgpoints
        dist_pickle = pickle.load( open( self.calib_path + "wide_dist_pickle.p", "rb" ) )
        objpoints   = dist_pickle["objpoints"]
        imgpoints   = dist_pickle["imgpoints"]# Read in an image

        img         = cv2.imread( self.calib_path + 'test_image.png')
        ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, img.shape[1:], None, None)
        
        undistorted = self.cal_undistort(img, mtx, dist)
        
        f, (ax1, ax2) = plt.subplots(1, 2, figsize=(24, 9))
        f.tight_layout()
        ax1.imshow(img)
        ax1.set_title('Original Image', fontsize=50)
        ax2.imshow(undistorted)
        ax2.set_title('Undistorted Image', fontsize=50)
        plt.subplots_adjust(left=0., right=1, top=0.9, bottom=0.)
        plt.show()

        # while not rospy.is_shutdown():


if __name__ == '__main__':
    calib = Calibration()
    calib.run()