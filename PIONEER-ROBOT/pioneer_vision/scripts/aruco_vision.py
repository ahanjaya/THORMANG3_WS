#!/usr/bin/env python3

import sys
import cv2
import yaml
import glob
import rospy
import rospkg
import numpy as np
from cv2 import aruco
from time import sleep
from std_msgs.msg import Bool
from sensor_msgs.msg import Image
from geometry_msgs.msg import Pose2D
from geometry_msgs.msg import Point32

class Aruco:
    def __init__(self, mode):
        
        self.rospack    = rospkg.RosPack()
        self.calib_path = self.rospack.get_path("pioneer_vision") + "/scripts/"
        self.mode       = mode

        if self.mode == "align_keyboard":
            self.config_path  = self.rospack.get_path("pioneer_main") + "/config/thormang3_align_keyboard_ws.yaml"
            self.video_path   = self.rospack.get_path("pioneer_vision") + "/data/thormang3_align_keyboard.avi"
        elif self.mode == "typing":
            self.config_path  = self.rospack.get_path("pioneer_main") + "/config/thormang3_typing_ws.yaml"
            self.video_path   = self.rospack.get_path("pioneer_vision") + "/data/thormang3_typing.avi"
        elif self.mode == "keyboard_calibration":
            self.config_path    = self.rospack.get_path("pioneer_main") + "/config/thormang3_typing_ws.yaml"
            self.keyboard_path  = self.rospack.get_path("pioneer_main") + "/config/thormang3_keyboard_cfg.yaml"
            self.video_path     = self.rospack.get_path("pioneer_vision") + "/data/thormang3_keyboard.avi"

        self.source_img  = np.zeros((rospy.get_param("/uvc_camera_center_node/width"), rospy.get_param("/uvc_camera_center_node/height"), 3), np.uint8)
        self.frame_size  = (self.source_img.shape[:-1])
        self.out         = cv2.VideoWriter(self.video_path, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'), 30, self.frame_size )

        self.mtx, self.dist         = None, None
        self.l_point, self.r_point  = None, None
        self.left_ws, self.right_ws = None, None
        self.kx, self.ky            = None, None

        self.keys         = {}
        self.keyboard_cfg = {}
        self.l_synch      = None
        self.recorder     = False

        ## Publisher
        self.left_aruco_pos_pub  = rospy.Publisher("/pioneer/aruco/left_position",    Point32, queue_size=1)
        self.right_aruco_pos_pub = rospy.Publisher("/pioneer/aruco/right_position",   Point32, queue_size=1)
        self.keyb_aruco_pos_pub  = rospy.Publisher("/pioneer/aruco/keyboard_position",Pose2D,  queue_size=1)
        self.left_arm_pos_pub    = rospy.Publisher("/pioneer/target/left_arm_point",  Point32, queue_size=1)
        self.right_arm_pos_pub   = rospy.Publisher("/pioneer/target/right_arm_point", Point32, queue_size=1)
        self.arm_start_pub       = rospy.Publisher("/pioneer/target/start",           Bool,    queue_size=1)
        self.arm_sync_pub        = rospy.Publisher("/pioneer/target/sync_arm",        Bool,    queue_size=1)
        self.ini_pose_pub        = rospy.Publisher("/pioneer/init_pose",              Bool,    queue_size=1)
        self.typing_pose_pub     = rospy.Publisher("/pioneer/typing",                 Bool,    queue_size=1)

        ## Subscriber
        rospy.Subscriber('/robotis/sensor/camera/image_raw', Image,   self.images_callback)
        rospy.Subscriber("/pioneer/aruco/lsync_position",    Point32, self.l_sync_callback)

    def images_callback(self, img):
        dt  = np.dtype(np.uint8)
        dt  = dt.newbyteorder('>')
        arr = np.frombuffer(img.data,dtype=dt)
        arr = np.reshape(arr,(480,640,3))
        self.source_img = cv2.cvtColor(arr, cv2.COLOR_BGR2RGB)

    def l_sync_callback(self, msg):
        x = int(msg.x)
        y = int(msg.y)
        self.l_synch = np.array([x, y])
        self.check_roi('left_arm', (self.l_synch), self.left_ws, False)

    def mouse_event(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            self.check_roi('left_arm', (x, y), self.left_ws)
        elif event == cv2.EVENT_LBUTTONUP:
            self.arm_start_pub.publish(True)
            
        elif event == cv2.EVENT_RBUTTONDOWN:
            self.check_roi('right_arm', (x, y), self.right_ws)
        elif event == cv2.EVENT_RBUTTONUP:
            self.arm_start_pub.publish(True)

        elif event == cv2.EVENT_MBUTTONDOWN:
            if self.mode != "keyboard_calibration":
                self.check_roi('right_arm', (x, y), self.right_ws)
            else:
                self.typing_pose_pub.publish(True)
        elif event == cv2.EVENT_MBUTTONUP:
            if self.mode != "keyboard_calibration":
                self.arm_sync_pub.publish(True)
                sleep(0.5)
                self.arm_start_pub.publish(True)

        elif event == cv2.EVENT_MOUSEWHEEL:
            self.ini_pose_pub.publish(True)
            self.l_point = self.r_point = None

    def check_roi(self, arm, points, area, pub=True):
        area = area.reshape((4,2))
        if arm == 'left_arm':
            x_min_b  = area[0][0]
            x_min_a  = area[1][0]
            x_max_a  = area[2][0]
            x_max_b  = area[3][0]
        elif arm == 'right_arm':
            x_max_b  = area[0][0]
            x_max_a  = area[1][0]
            x_min_a  = area[2][0]
            x_min_b  = area[3][0]

        y_frame_min = area[1][1]
        y_frame_max = area[0][1]

        x_frame_min = np.interp( points[1], [y_frame_min, y_frame_max], [x_min_a, x_min_b] )
        x_frame_min = int( np.round(x_frame_min, 0) )
        x_frame_max = np.interp( points[1], [y_frame_min, y_frame_max], [x_max_a, x_max_b] )
        x_frame_max = int( np.round(x_frame_max, 0) )

        # Y Check
        if points[1] >= y_frame_min and points[1] <= y_frame_max:
            # X Check
            if points[0] >= x_frame_min and points[0] <= x_frame_max:
                tar_pos   = Point32()
                tar_pos.x = points[0]
                tar_pos.y = points[1]

                if arm == 'left_arm':
                    self.l_point = (points[0], points[1])
                    if pub:
                        self.left_arm_pos_pub.publish(tar_pos)
                elif arm == 'right_arm':
                    self.r_point = (points[0], points[1])
                    if pub:
                        self.right_arm_pos_pub.publish(tar_pos)
        #     else:
        #         rospy.logerr("[{0}] X Out of range".format(arm))
        # else:
        #     rospy.logerr("[{0}] Y Out of range".format(arm))

    def load_config(self, arm):
        try:
            with open(self.config_path, 'r') as f:
                aruco_ws = yaml.safe_load(f)
        except yaml.YAMLError as exc:
            print(exc)
        try:
            config = aruco_ws[arm]
            pts    = []
            for i in ['P1', 'P2', 'P3', 'P4']:
                pts.append( [config[i]['cx'], config[i]['cy']] )
            pts = np.array( pts, np.int32)

            # polygon y_frame average point
            # y_frame_min = np.mean([ pts[1][1], pts[2][1] ], dtype=int)
            # y_frame_max = np.mean([ pts[0][1], pts[3][1] ], dtype=int)
            # pts[1][1] = pts[2][1] = y_frame_min
            # pts[0][1] = pts[3][1] = y_frame_max

            # rospy.loginfo('[Aruco] Load {0} workspace successfully'.format(arm))
            return pts.reshape((-1,1,2))
        except:
            return None

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

    def logging_keyb(self, k, row, arm):
        if arm == 'left_arm':
            self.keys[chr(k)] = self.l_point
        elif arm == 'right_arm':
            self.keys[chr(k)] = self.r_point
        
        rospy.loginfo("[Aruco] {} button : {}".format(chr(k), self.keys[chr(k)]) )

        if row == '_1st_row':
            l_row = ["1", "2", "3", "4", "5"]
            r_row = ["6", "7", "8", "9", "0", "-", "="]

            if l_row[0] in self.keys and l_row[-1] in self.keys and \
                r_row[0] in self.keys and r_row[-1] in self.keys:

                first_btn = self.keys[l_row[0]]
                last_btn  = self.keys[l_row[-1]]
                x = np.linspace(first_btn[0], last_btn[0], len(l_row), dtype=int) # X
                y = np.linspace(first_btn[1], last_btn[1], len(l_row), dtype=int) # Y

                temp_key = {}
                for idx, val in enumerate( l_row ):
                    temp_key[val] = (int(x[idx]), int(y[idx]))

                first_btn = self.keys[r_row[0]]
                last_btn  = self.keys[r_row[-1]]
                x = np.linspace(first_btn[0], last_btn[0], len(r_row), dtype=int) # X
                y = np.linspace(first_btn[1], last_btn[1], len(r_row), dtype=int) # Y

                for idx, val in enumerate( r_row ):
                    temp_key[val] = (int(x[idx]), int(y[idx]))
            
                self.keyboard_cfg[row] = temp_key

        elif row == '_2nd_row':
            l_row = ["q", "w", "e", "r", "t"]
            r_row = ["y", "u", "i", "o", "p", "[", "]"]

            if l_row[0] in self.keys and l_row[-1] in self.keys and \
                r_row[0] in self.keys and r_row[-1] in self.keys:

                first_btn = self.keys[l_row[0]]
                last_btn  = self.keys[l_row[-1]]
                x = np.linspace(first_btn[0], last_btn[0], len(l_row), dtype=int) # X
                y = np.linspace(first_btn[1], last_btn[1], len(l_row), dtype=int) # Y

                temp_key = {}
                for idx, val in enumerate( l_row ):
                    temp_key[val] = (int(x[idx]), int(y[idx]))

                first_btn = self.keys[r_row[0]]
                last_btn  = self.keys[r_row[-1]]
                x = np.linspace(first_btn[0], last_btn[0], len(r_row), dtype=int) # X
                y = np.linspace(first_btn[1], last_btn[1], len(r_row), dtype=int) # Y

                for idx, val in enumerate( r_row ):
                    temp_key[val] = (int(x[idx]), int(y[idx]))
            
                self.keyboard_cfg[row] = temp_key

        elif row == '_3rd_row':
            l_row = ["a", "s", "d", "f", "g"]
            r_row = ["h", "j", "k", "l", ";", "'"]

            if l_row[0] in self.keys and l_row[-1] in self.keys and \
                r_row[0] in self.keys and r_row[-1] in self.keys:

                first_btn = self.keys[l_row[0]]
                last_btn  = self.keys[l_row[-1]]
                x = np.linspace(first_btn[0], last_btn[0], len(l_row), dtype=int) # X
                y = np.linspace(first_btn[1], last_btn[1], len(l_row), dtype=int) # Y

                temp_key = {}
                for idx, val in enumerate( l_row ):
                    temp_key[val] = (int(x[idx]), int(y[idx]))

                first_btn = self.keys[r_row[0]]
                last_btn  = self.keys[r_row[-1]]
                x = np.linspace(first_btn[0], last_btn[0], len(r_row), dtype=int) # X
                y = np.linspace(first_btn[1], last_btn[1], len(r_row), dtype=int) # Y

                for idx, val in enumerate( r_row ):
                    temp_key[val] = (int(x[idx]), int(y[idx]))
            
                self.keyboard_cfg[row] = temp_key

        elif row == '_4th_row':
            l_row = ["z", "x", "c", "v", "b"]
            r_row = ["n", "m", ",", ".", "/"]

            if l_row[0] in self.keys and l_row[-1] in self.keys and \
                r_row[0] in self.keys and r_row[-1] in self.keys:

                first_btn = self.keys[l_row[0]]
                last_btn  = self.keys[l_row[-1]]
                x = np.linspace(first_btn[0], last_btn[0], len(l_row), dtype=int) # X
                y = np.linspace(first_btn[1], last_btn[1], len(l_row), dtype=int) # Y

                temp_key = {}
                for idx, val in enumerate( l_row ):
                    temp_key[val] = (int(x[idx]), int(y[idx]))

                first_btn = self.keys[r_row[0]]
                last_btn  = self.keys[r_row[-1]]
                x = np.linspace(first_btn[0], last_btn[0], len(r_row), dtype=int) # X
                y = np.linspace(first_btn[1], last_btn[1], len(r_row), dtype=int) # Y

                for idx, val in enumerate( r_row ):
                    temp_key[val] = (int(x[idx]), int(y[idx]))
            
                self.keyboard_cfg[row] = temp_key
  
    def run(self):
        aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_50)
        parameters = aruco.DetectorParameters_create()
        left_aruco_pos  = Point32()
        right_aruco_pos = Point32()
        keyb_aruco_pos  = Point32()

        cv2.namedWindow("image")
        cv2.setMouseCallback("image", self.mouse_event)
        
        if self.mode == "keyboard_calibration":
            self.calibration()
            
        # load polygon
        if not self.recorder:
            self.left_ws  = self.load_config('left_arm')
            self.right_ws = self.load_config('right_arm')

        while not rospy.is_shutdown():
            res_img  = self.source_img.copy()
            gray_img = cv2.cvtColor(self.source_img, cv2.COLOR_BGR2GRAY)
            corners, ids, rejectedImgPoints = aruco.detectMarkers(gray_img, aruco_dict, parameters=parameters)
            
            left_aruco_pos.x  = left_aruco_pos.y  = -1
            right_aruco_pos.x = right_aruco_pos.y = -1
            keyb_aruco_pos.x  = keyb_aruco_pos.y  = -1

            if np.all(ids != None):
                for i in range(0, ids.size):
                    M = cv2.moments(corners[i])

                    if self.mode == "keyboard_calibration":
                        if ids[i,0] == 10:
                            self.kx = keyb_aruco_pos.x = int(M["m10"] / M["m00"])
                            self.ky = keyb_aruco_pos.y = int(M["m01"] / M["m00"])

                            rvec, tvec ,_ = aruco.estimatePoseSingleMarkers(corners[i], 0.05, self.mtx, self.dist)
                            aruco.drawAxis(res_img, self.mtx, self.dist, rvec, tvec, 0.1)
                            rmat, _    = cv2.Rodrigues(rvec)
                            rx, ry, rz = np.degrees(self.rotation_matrix_to_euler_angles(rmat))
                            # rz = keyb_aruco_pos.theta = np.round(rz, 2)
                            rz = keyb_aruco_pos.theta = 0.0
                            cv2.putText(res_img, "angle= " + str(rz), (self.kx, self.ky+20), \
                                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,255), 2)

                            self.keyboard_cfg['aruco_ref'] = {'cx': self.kx, 'cy': self.ky, 'angle':float(rz)}
                    else:
                        if ids[i,0] == 0:
                            left_aruco_pos.x  = int(M["m10"] / M["m00"])
                            left_aruco_pos.y  = int(M["m01"] / M["m00"])
                        elif ids[i,0] == 1:
                            right_aruco_pos.x = int(M["m10"] / M["m00"])
                            right_aruco_pos.y = int(M["m01"] / M["m00"])
                    
            aruco.drawDetectedMarkers(res_img, corners, ids)
            
            if self.mode == "keyboard_calibration":
                self.keyb_aruco_pos_pub.publish(keyb_aruco_pos)
            else:
                self.left_aruco_pos_pub.publish(left_aruco_pos)
                self.right_aruco_pos_pub.publish(right_aruco_pos)

            if self.l_point != None:
                cv2.circle(res_img, self.l_point, 5, (255,0,0), -1) # Left arm point
            if self.r_point != None:
                cv2.circle(res_img, self.r_point, 5, (0,0,255), -1) # Right arm point

            cv2.polylines(res_img,[self.left_ws], True, (255,0,0), 2)
            cv2.polylines(res_img,[self.right_ws], True, (0,0,255), 2)
            cv2.putText(res_img, "Mode: " + self.mode , (5 , 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255,255,255), 2)

            if self.recorder:
                self.out.write(res_img)

            cv2.imshow('image', res_img)

            k = cv2.waitKey(20)
            if k == 27:
                break
            if self.mode == "keyboard_calibration":
                # first row
                if k == ord('1'):
                    self.logging_keyb(k, '_1st_row', 'left_arm')
                elif k == ord('5'):
                    self.logging_keyb(k, '_1st_row', 'left_arm')

                elif k == ord('6'):
                    self.logging_keyb(k, '_1st_row', 'right_arm')    
                elif k == ord('='):
                    self.logging_keyb(k, '_1st_row', 'right_arm')

                # second row
                elif k == ord('q'):
                    self.logging_keyb(k, '_2nd_row', 'left_arm')
                elif k == ord('t'):
                    self.logging_keyb(k, '_2nd_row', 'left_arm')
                elif k == ord('y'):
                    self.logging_keyb(k, '_2nd_row', 'right_arm')
                elif k == ord(']'):
                    self.logging_keyb(k, '_2nd_row', 'right_arm')

                # third row
                elif k == ord('a'):
                    self.logging_keyb(k, '_3rd_row', 'left_arm')
                elif k == ord('g'):
                    self.logging_keyb(k, '_3rd_row', 'left_arm')
                elif k == ord('h'):
                    self.logging_keyb(k, '_3rd_row', 'right_arm')
                elif k == ord("'"):
                    self.logging_keyb(k, '_3rd_row', 'right_arm')

                # forth row
                elif k == ord('z'):
                    self.logging_keyb(k, '_4th_row', 'left_arm')
                elif k == ord('b'):
                    self.logging_keyb(k, '_4th_row', 'left_arm')
                elif k == ord('n'):
                    self.logging_keyb(k, '_4th_row', 'right_arm')
                elif k == ord('/'):
                    self.logging_keyb(k, '_4th_row', 'right_arm')

                elif k == ord('s'):
                    # print(self.keyboard_cfg)
                    if '_1st_row' in self.keyboard_cfg and \
                       '_2nd_row' in self.keyboard_cfg and \
                       '_3rd_row' in self.keyboard_cfg and \
                       '_4th_row' in self.keyboard_cfg and \
                       'aruco_ref' in self.keyboard_cfg :

                        rospy.loginfo("[Aruco] Save Keyboard Config")
                        with open(self.keyboard_path, 'w') as f:
                            yaml.dump(self.keyboard_cfg, f, default_flow_style=False)
            else:
                if k == ord('l'):
                    rospy.loginfo("[Aruco] Load Workspace Map")
                    self.left_ws  = self.load_config('left_arm')
                    self.right_ws = self.load_config('right_arm')
              
        rospy.loginfo("[Aruco] Shutdown")

if __name__ == '__main__':
    rospy.init_node('pioneer_vision_aruco', anonymous=False)

    # if using ros launch length of sys.argv is 4
    if len(sys.argv) == 4:
        if sys.argv[1] == "keyboard_calibration" or \
           sys.argv[1] == "align_keyboard" or \
           sys.argv[1] == "typing":

            rospy.loginfo("[Aruco] Pioneer Calibration {}".format(sys.argv[1]))

            aruco_ws = Aruco(sys.argv[1])
            aruco_ws.run()
        else:
            rospy.logerr("[Aruco] Exit Unknown Mode")
    else:
        rospy.logerr("[Aruco] Exit Argument not fulfilled")
        