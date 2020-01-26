#!/usr/bin/env python3

import cv2
import glob
import rospy
import numpy as np
from cv2 import aruco
from sensor_msgs.msg import Image
from pioneer_vision.camera import Camera

class Aruco:
    def __init__(self):
        self.camera       = Camera()
        # frame             = self.camera.source_image.copy()
        # self.frame_width  = frame.shape[0]
        # self.frame_height = frame.shape[1]

        self.mtx, self.dist  = None, None
        self.kx, self.ky     = None, None

    def calibration(self):
        ###---------------------- CALIBRATION ---------------------------
        # termination criteria for the iterative algorithm
        criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

        # prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
        # checkerboard of size (7 x 6) is used
        objp = np.zeros((6*7,3), np.float32)
        objp[:,:2] = np.mgrid[0:7,0:6].T.reshape(-1,2)

        # arrays to store object points and image points from all the images.
        objpoints = [] # 3d point in real world space
        imgpoints = [] # 2d points in image plane.

        # iterating through all calibration images
        # in the folder
        images = glob.glob(self.calib_path + 'calib_images/*.jpg')

        for fname in images:
            img  = cv2.imread(fname)
            gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)

            # find the chess board (calibration pattern) corners
            ret, corners = cv2.findChessboardCorners(gray, (7,6),None)

            # if calibration pattern is found, add object points,
            # image points (after refining them)
            if ret == True:
                objpoints.append(objp)
                # Refine the corners of the detected corners
                corners2 = cv2.cornerSubPix(gray,corners, (11,11), (-1,-1), criteria)
                imgpoints.append(corners2)

                # Draw and display the corners
                img = cv2.drawChessboardCorners(img, (7,6), corners2, ret)

        ret, self.mtx, self.dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)

    # Checks if a matrix is a valid rotation matrix.
    def is_rotation_matrix(self, R) :
        Rt = np.transpose(R)
        shouldBeIdentity = np.dot(Rt, R)
        I = np.identity(3, dtype = R.dtype)
        n = np.linalg.norm(I - shouldBeIdentity)
        return n < 1e-6
    
    # Calculates rotation matrix to euler angles
    # The result is the same as MATLAB except the order
    # of the euler angles ( x and z are swapped ).
    def rotation_matrix_to_euler_angles(self, R) :
        assert(self.is_rotation_matrix(R))     
        sy = np.sqrt(R[0,0] * R[0,0] +  R[1,0] * R[1,0])     
        singular = sy < 1e-6 
        if  not singular :
            x = np.arctan2(R[2,1] , R[2,2])
            y = np.arctan2(-R[2,0], sy)
            z = np.arctan2(R[1,0], R[0,0])
        else :
            x = np.arctan2(-R[1,2], R[1,1])
            y = np.arctan2(-R[2,0], sy)
            z = 0
    
        return np.array([x, y, z])

    def run(self):
        camera     = self.camera
        aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_50)
        parameters = aruco.DetectorParameters_create()

        while not rospy.is_shutdown():
            frame    = camera.source_image.copy()
            gray_img = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            corners, ids, rejectedImgPoints = aruco.detectMarkers(gray_img, aruco_dict, parameters=parameters)
            
            # if np.all(ids != None):
            #     M = cv2.moments(corners[i])

            #     # self.kx = int(M["m10"] / M["m00"])
            #     # self.ky = int(M["m01"] / M["m00"])

            #     rvec, tvec ,_ = aruco.estimatePoseSingleMarkers(corners[i], 0.05, self.mtx, self.dist)
            #     aruco.drawAxis(res_img, self.mtx, self.dist, rvec, tvec, 0.1)
            #     rmat, _    = cv2.Rodrigues(rvec)
            #     rx, ry, rz = np.degrees(self.rotation_matrix_to_euler_angles(rmat))
            #     # rz = keyb_aruco_pos.theta = np.round(rz, 2)
            #     cv2.putText(res_img, "angle= " + str(rz), (self.kx, self.ky+20), \
            #                     cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,255), 2)

            aruco.drawDetectedMarkers(frame, corners, ids)
            cv2.imshow('image', frame)

            k = cv2.waitKey(20)
            if k == 27:
                break

        cv2.destroyAllWindows()
        rospy.loginfo("[Aruco] Shutdown")

if __name__ == '__main__':
    rospy.init_node('pioneer_aruco', anonymous=False)
    aruco_class = Aruco()
    aruco_class.run()        