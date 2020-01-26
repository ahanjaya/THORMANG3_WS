#!/usr/bin/env python3

import yaml
import rospy
import rospkg
import numpy as np
from time import sleep
from std_msgs.msg import Bool
from geometry_msgs.msg import Point32
from geometry_msgs.msg import Pose2D
from pioneer_simulation.msg import Pose2DArray
from pioneer_kinematics.kinematics import Kinematics

class Placement_Keyboard:
    def __init__(self):
        np.set_printoptions(suppress=True)
        rospy.init_node('pioneer_placement_keyboard', anonymous=False)
        rospy.loginfo("[Main] Pioneer Placement Keyboard - Running")

        rospack        = rospkg.RosPack()
        self.ws_path   = rospack.get_path("pioneer_main") + "/config/thormang3_align_keyboard_ws.yaml"

        # Subscriber
        rospy.Subscriber("/pioneer/placement/left_arm_point",       Point32,     self.left_arm_pos_callback)
        rospy.Subscriber("/pioneer/placement/right_arm_point",      Point32,     self.right_arm_pos_callback)
        rospy.Subscriber("/pioneer/placement/approach_keyboard",    Bool,        self.approach_keyboard_callback)
        rospy.Subscriber("/pioneer/placement/placement_trigger",    Bool,        self.placement_trigger_callback)
        rospy.Subscriber("/pioneer/placement/grasp_keyboard",       Bool,        self.grip_key_callback)
        rospy.Subscriber("/pioneer/placement/init_pose",            Bool,        self.ini_pose_callback)
        rospy.Subscriber("/pioneer/placement/left_arm_arr_points",  Pose2DArray, self.left_arm_arr_points_callback)
        rospy.Subscriber("/pioneer/placement/right_arm_arr_points", Pose2DArray, self.right_arm_arr_points_callback)
        rospy.Subscriber("/pioneer/placement/shutdown_signal",      Bool,        self.shutdown_callback)

        # Publisher
        self.finish_placement_pub = rospy.Publisher("/pioneer/placement/finish_placement",  Bool, queue_size=1)

        # Variables
        self.kinematics = Kinematics()
        self.main_rate  = rospy.Rate(20)
        self.state      = None
        self.shutdown   = False

        self.zl, self.rl, self.pl, self.yl = 0.62,  150,  -1, -29  # 0.63
        self.zr, self.rr, self.pr, self.yr = 0.62, -150, -1, 29    # 0.63

        self.left_points, self.right_points        = None, None
        self.ik_l_trajectory, self.ik_r_trajectory = None, None
        self.ik_debug  = False

    def left_arm_pos_callback(self, msg):
        self.left_keyboard = (msg.x, msg.y)
        rospy.loginfo('[Main] Left keyboard frame (X: {} Y: {})'.format(msg.x, msg.y) )
        self.lx_ik, self.ly_ik = self.left_arm_ik(msg.x, msg.y)

    def right_arm_pos_callback(self, msg):
        self.right_keyboard = (msg.x, msg.y)
        rospy.loginfo('[Main] Right keyboard frame (X: {} Y: {})'.format(msg.x, msg.y) )
        self.rx_ik, self.ry_ik = self.right_arm_ik(msg.x, msg.y)

    def approach_keyboard_callback(self, msg):
        if msg.data == True:
            self.state = 'approach_keyboard'

    def placement_trigger_callback(self, msg):
        if msg.data == True:
            self.state = 'placement_trigger'

    def ini_pose_callback(self, msg):
        if msg.data == True:
            self.state = 'init_pose'    

    def grip_key_callback(self, msg):
        if msg.data == True:
            self.state = 'grip_keyboard'

    def shutdown_callback(self, msg):
        self.shutdown = msg.data
        rospy.signal_shutdown('Exit')

    def wait_robot(self, obj, msg):
        sleep(0.2)
        if msg == "End Left Arm Trajectory":
            while obj.left_tra != False:
                if self.shutdown:
                    break
                else:
                    pass # do nothing
        elif msg == "End Right Arm Trajectory":
            while obj.right_tra != False:
                if self.shutdown:
                    break
                else:
                    pass # do nothing
        else:
            while obj.status_msg != msg:
                if self.shutdown:
                    break
                else:
                    pass # do nothing

    def left_arm_arr_points_callback(self, msg):
        group                = msg.name
        self.l_num_points    = len(msg.poses)
        l_frame_tra          = [ (pose.x, pose.y)   for pose in msg.poses]
        l_frame_tra          = np.array( l_frame_tra )
        l_frame_tra[:,1]     = rospy.get_param("/uvc_camera_center_node/height") - l_frame_tra[:,1]
        diff_keyboard        = np.array(self.left_keyboard) - np.array([l_frame_tra[0][0], l_frame_tra[0][1]])
        self.l_frame_tra     = [ self.translate_2D_point( (pose[0], pose[1]), diff_keyboard ) for pose in l_frame_tra ]
        self.ik_l_trajectory = [ self.left_arm_ik(pose[0], pose[1]) for pose in self.l_frame_tra]
        # rospy.loginfo('[PA] Group: {}, Total_Points: {}, IK_Point: {}'.format(group, self.l_num_points, self.ik_l_trajectory))
        rospy.loginfo('[PA] {} Arr Received: {}'.format(group, self.l_num_points))

    def right_arm_arr_points_callback(self, msg):
        group                = msg.name
        self.r_num_points    = len(msg.poses)
        r_frame_tra          = [ (pose.x, pose.y)   for pose in msg.poses]
        r_frame_tra          = np.array( r_frame_tra )
        r_frame_tra[:,1]     = rospy.get_param("/uvc_camera_center_node/height") - r_frame_tra[:,1]
        diff_keyboard        = np.array(self.right_keyboard) - np.array([r_frame_tra[0][0], r_frame_tra[0][1]])
        self.r_frame_tra     = [ self.translate_2D_point( (pose[0], pose[1]), diff_keyboard ) for pose in r_frame_tra ]
        self.ik_r_trajectory = [ self.right_arm_ik(pose[0], pose[1]) for pose in self.r_frame_tra]
        # rospy.loginfo('[PA] Group: {}, Total_Points: {}, IK_Points: {}'.format(group, self.r_num_points, self.ik_r_trajectory))
        rospy.loginfo('[PA] {} Arr Received: {}'.format(group, self.r_num_points))

    def translate_2D_point(self, point, diff_point):
        return tuple(np.array(point) + diff_point)

    def load_ws_config(self, arm):
        try:
            with open(self.ws_path, 'r') as f:
                aruco_ws = yaml.safe_load(f)
                rospy.loginfo('[Main] Loaded {} workspace'.format(arm))
            return aruco_ws[arm]

        except yaml.YAMLError as exc:
            print(exc)
            return None

    def parse_Yframe(self, data):
        y_frame_min = np.mean([ data['P2']['cy'], data['P3']['cy'] ], dtype=int)
        y_frame_max = np.mean([ data['P1']['cy'], data['P4']['cy'] ], dtype=int)

        ik_xmin = data['P1']['ik_x']
        ik_xmax = data['P2']['ik_x']
        return ( y_frame_min, y_frame_max, ik_xmin, ik_xmax )

    def parse_Xframe(self, data):
        x_frame_min = ( data['P1']['cx'], data['P2']['cx'] )
        x_frame_max = ( data['P3']['cx'], data['P4']['cx'] )

        ik_ymin_lower = data['P4']['ik_y'] #0.1
        ik_ymax_lower = data['P1']['ik_y'] #0.24
        ik_ymin_upper = data['P3']['ik_y'] #0.05
        ik_ymax_upper = data['P2']['ik_y'] #0.34
        return ( x_frame_min, x_frame_max, ik_ymax_lower, ik_ymax_upper, ik_ymin_lower, ik_ymin_upper)

    def move_arm(self, arm, time, x, y):
        if arm == "left_arm":
            self.kinematics.set_kinematics_pose(arm , time, **{ 'x': x, 'y': y, 'z': self.zl, 'roll': self.rl, 'pitch': self.pl, 'yaw': self.yl })
            # self.prev_lik = (x, y)
            rospy.loginfo('[Main] Lx_ik : {:.2f} Ly_ik : {:.2f}'.format(x, y))

        elif arm == "right_arm":
            self.kinematics.set_kinematics_pose(arm , time, **{ 'x': x, 'y': y, 'z': self.zr, 'roll': self.rr, 'pitch': self.pr, 'yaw': self.yr })
            # self.prev_rik = (x, y)
            rospy.loginfo('[Main] Rx_ik : {:.2f} Ry_ik : {:.2f}'.format(x, y))

    def ik_reference(self, left_ws, right_ws):
        if left_ws != None:
            yleft_data    = self.parse_Yframe(left_ws)
            self.ly_frame_min  = yleft_data[0]
            self.ly_frame_max  = yleft_data[1]
            self.ly_ws    = range(self.ly_frame_min, self.ly_frame_max+1)
            self.lik_xmin = yleft_data[2]
            self.lik_xmax = yleft_data[3]

            xleft_data          = self.parse_Xframe(left_ws)
            self.lx_min_b       = xleft_data[0][0]
            self.lx_min_a       = xleft_data[0][1]
            self.lx_max_a       = xleft_data[1][0]
            self.lx_max_b       = xleft_data[1][1]

            self.lik_ymax_lower = xleft_data[2] #0.24
            self.lik_ymax_upper = xleft_data[3] #0.34
            self.lik_ymin_lower = xleft_data[4] #0.1
            self.lik_ymin_upper = xleft_data[5] #0.05

        if right_ws != None:
            yright_data   = self.parse_Yframe(right_ws)
            self.ry_frame_min  = yright_data[0]
            self.ry_frame_max  = yright_data[1]
            self.ry_ws         = range(self.ry_frame_min, self.ry_frame_max+1)
            self.rik_xmin      = yright_data[2]
            self.rik_xmax      = yright_data[3]

            xright_data        = self.parse_Xframe(right_ws)
            self.rx_max_b      = xright_data[0][0]
            self.rx_max_a      = xright_data[0][1]
            self.rx_min_a      = xright_data[1][0]
            self.rx_min_b      = xright_data[1][1]

            self.rik_ymax_lower = xright_data[2] #0.24
            self.rik_ymax_upper = xright_data[3] #0.34
            self.rik_ymin_lower = xright_data[4] #0.1
            self.rik_ymin_upper = xright_data[5] #0.05

    def left_arm_ik(self, cx, cy):
        if cy in self.ly_ws:
            # Left IK X Target
            lx_ik = np.interp( cy, [self.ly_frame_min, self.ly_frame_max], [self.lik_xmax, self.lik_xmin] )

            # Mapping CX Frame
            lx_frame_min = np.interp( cy, [self.ly_frame_min, self.ly_frame_max], [self.lx_min_a, self.lx_min_b] )
            lx_frame_min = int( np.round(lx_frame_min, 0) )
            lx_frame_max = np.interp( cy, [self.ly_frame_min, self.ly_frame_max], [self.lx_max_a, self.lx_max_b] )
            lx_frame_max = int( np.round(lx_frame_max, 0) )

            # Mapping IK_Y
            lik_ymax     = np.interp( cy, [self.ly_frame_min, self.ly_frame_max], [self.lik_ymax_upper, self.lik_ymax_lower] )
            lik_ymax     = np.round(lik_ymax, 4)
            lik_ymin     = np.interp( cy, [self.ly_frame_min, self.ly_frame_max], [self.lik_ymin_upper, self.lik_ymin_lower] )
            lik_ymin     = np.round(lik_ymin, 4)

            lx_ws = range(lx_frame_min, lx_frame_max+1)
            if cx in lx_ws:
                # Left IK Y Target
                ly_ik = np.interp( cx, [lx_frame_min, lx_frame_max], [lik_ymax, lik_ymin] )

                return lx_ik, ly_ik
                # print()
                # rospy.loginfo('[Left Arm] Input Coor X: {0}, Y: {1}'.format(cx, cy))
                # rospy.loginfo('[Left Arm] X_IK: {0:.2f}, Y_IK: {1:.2f}'.format(self.lx_ik, self.ly_ik))
            else:
                return None, None
                # rospy.logerr('[Left Arm] X Frame target is out of range')
        else:
            return None, None
            # rospy.logerr('[Left Arm] Y Frame target is out of range')

    def right_arm_ik(self, cx, cy):
        if cy in self.ry_ws:
            # Right IK X Target
            rx_ik = np.interp( cy, [self.ry_frame_min, self.ry_frame_max], [self.rik_xmax, self.rik_xmin] )

            # Mapping CX Frame
            rx_frame_min = np.interp( cy, [self.ry_frame_min, self.ry_frame_max], [self.rx_min_a, self.rx_min_b] )
            rx_frame_min = int( np.round(rx_frame_min, 0) )
            rx_frame_max = np.interp( cy, [self.ry_frame_min, self.ry_frame_max], [self.rx_max_a, self.rx_max_b] )
            rx_frame_max = int( np.round(rx_frame_max, 0) )

            # Mapping IK_Y
            rik_ymax     = np.interp( cy, [self.ry_frame_min, self.ry_frame_max], [self.rik_ymax_upper, self.rik_ymax_lower] )
            rik_ymax     = np.round(rik_ymax, 4)
            rik_ymin     = np.interp( cy, [self.ry_frame_min, self.ry_frame_max], [self.rik_ymin_upper, self.rik_ymin_lower] )
            rik_ymin     = np.round(rik_ymin, 4)

            rx_ws = range(rx_frame_min, rx_frame_max+1)
            if cx in rx_ws:
                # Left IK Y Target
                ry_ik = np.interp( cx, [rx_frame_min, rx_frame_max], [rik_ymin, rik_ymax] )
                
                return rx_ik, ry_ik
                # print()
                # rospy.loginfo('[Right Arm] Input Coor X: {0}, Y: {1}'.format(cx, cy))
                # rospy.loginfo('[Right Arm] X_IK: {0:.2f}, Y_IK: {1:.2f}'.format(self.rx_ik, self.ry_ik))
            else:
                return None, None
                # rospy.logerr('[Right Arm] X Frame target is out of range')
        else:
            return None, None
            # rospy.logerr('[Right Arm] Y Frame target is out of range')

    def run(self):
        kinematics = self.kinematics

        kinematics.publisher_(kinematics.module_control_pub,    "manipulation_module", latch=True)  # <-- Enable Manipulation mode
        kinematics.publisher_(kinematics.send_ini_pose_msg_pub, "align_keyboard_pose", latch=True)
        kinematics.publisher_(kinematics.en_align_key_pub,       True,                 latch=False) # <-- Enable Align Keboard mode
        self.wait_robot(kinematics, "End Init Trajectory")

        # set init head, torso, gripper
        kinematics.set_joint_pos(['head_p', 'head_y', 'torso_y'], [30, 0, 0])
        kinematics.set_gripper("left_arm", 0, 0)
        kinematics.set_gripper("right_arm", 0, 0)
        sleep(1)
        kinematics.set_joint_pos(['l_arm_finger45_p', 'r_arm_finger45_p'], [180, 180])
        sleep(1)
        rospy.loginfo('[Main] Finish Init Head & Hand')

        # set hand joint torques
        torque_lvl = 20
        kinematics.set_joint_torque(['hand'], torque_lvl)
        rospy.loginfo('[Main] Set Torque : {}%'.format(torque_lvl))

        # load config file
        left_ws  = self.load_ws_config('left_arm')
        right_ws = self.load_ws_config('right_arm')
        self.ik_reference(left_ws, right_ws)

        rospy.loginfo("[Main] Save Data: {}".format(rospy.get_param("/pioneer/placement/save_data")))

        while not rospy.is_shutdown():
            if self.shutdown:
                break

            if self.state == 'init_pose':
                rospy.loginfo('[Main] Robot State : {}'.format(self.state))

                cur_left_arm  = kinematics.get_kinematics_pose("left_arm")
                cur_right_arm = kinematics.get_kinematics_pose("right_arm")
            
                if cur_left_arm['z'] < 0.7 or cur_right_arm['z'] < 0.7:
                    self.lx_ik = cur_left_arm['x']
                    self.ly_ik = cur_left_arm['y'] + 0.07
                    self.move_arm("left_arm" , 2.0, self.lx_ik, self.ly_ik)

                    self.rx_ik = cur_right_arm['x']
                    self.ry_ik = cur_right_arm['y'] - 0.07
                    self.move_arm("right_arm" , 2.0, self.rx_ik, self.ry_ik)
                    sleep(2.5)
    
                kinematics.publisher_(kinematics.send_ini_pose_msg_pub, "align_keyboard_pose", latch=False)
                kinematics.publisher_(kinematics.en_align_key_pub, True, latch=False) # <-- Enable Align Keboard mode 

                self.lx_ik = self.ly_ik = self.rx_ik = self.ry_ik = None
                self.state = None

            elif self.state == 'approach_keyboard':
                rospy.loginfo('[Main] Robot State : {}'.format(self.state))
                # sleep(1)

                if self.lx_ik != None and self.ly_ik != None \
                    and self.rx_ik != None and self.ry_ik != None:
                    # print('move arm now')

                    self.ly_ik += 0.055
                    self.lx_ik -= 0.0
                    self.move_arm("left_arm" , 2.0, self.lx_ik, self.ly_ik)

                    self.ry_ik -= 0.055
                    self.rx_ik -= 0.0
                    self.move_arm("right_arm" , 2.0, self.rx_ik, self.ry_ik)
                else:
                    rospy.logwarn('[Main] Robot arm singularities \n Please move keyboard to workspace')
                self.state = None

            elif self.state == 'grip_keyboard':
                rospy.loginfo('[Main] Robot State : {}'.format(self.state))

                # grip by offseting ik <-- not good when change object
                cur_left_arm  = kinematics.get_kinematics_pose("left_arm")
                cur_right_arm = kinematics.get_kinematics_pose("right_arm")

                self.lx_ik = cur_left_arm['x']
                self.ly_ik = cur_left_arm['y'] - 0.06
                self.rx_ik = cur_right_arm['x']
                self.ry_ik = cur_right_arm['y'] + 0.06

                self.move_arm("left_arm" , 2.0, self.lx_ik, self.ly_ik)
                self.move_arm("right_arm" , 2.0, self.rx_ik, self.ry_ik)

                ##-----------------------------
                ## grip by offseting pixel
                # offset_pixel_x = 35
                # self.left_keyboard = (self.left_keyboard[0]+offset_pixel_x, self.left_keyboard[1]) # x. y
                # self.lx_ik, self.ly_ik = self.left_arm_ik(self.left_keyboard[0], self.left_keyboard[1])
                # self.right_keyboard = (self.right_keyboard[0]-offset_pixel_x, self.right_keyboard[1]) # x. y
                # self.rx_ik, self.ry_ik = self.right_arm_ik(self.right_keyboard[0], self.right_keyboard[1])
                
                # self.move_arm("left_arm" , 2.0, self.lx_ik, self.ly_ik)
                # self.move_arm("right_arm" , 2.0, self.rx_ik, self.ry_ik)

                self.state = None

            elif self.state == 'placement_trigger':
                rospy.loginfo('[Main] Robot State : {}'.format(self.state))

                if self.ik_l_trajectory != None and self.ik_r_trajectory != None:
                    if self.ik_debug:
                        for i in range (self.l_num_points):
                            rospy.loginfo( 'frame_l: {}, {}'.format(self.l_frame_tra[i][0], self.l_frame_tra[i][1] ) )
                        for i in range (self.r_num_points):
                            rospy.loginfo( 'frame_r: {}, {}'.format(self.r_frame_tra[i][0], self.r_frame_tra[i][1] ) )

                        for i in range (self.l_num_points):
                            rospy.loginfo( 'ik_l: {}'.format(self.ik_l_trajectory[i]) )
                        for i in range (self.r_num_points):
                            rospy.loginfo( 'ik_r: {}'.format(self.ik_r_trajectory[i]) )
        
                    ikl_res = [] 
                    for val in self.ik_l_trajectory: 
                        if val != (None,None) : 
                            ikl_res.append(val) 
                        else:
                            break

                    ikr_res = [] 
                    for val in self.ik_r_trajectory: 
                        if val != (None,None) : 
                            ikr_res.append(val) 
                        else:
                            break

                    lim_trajectory = min(len(ikl_res), len(ikr_res))

                    if self.ik_debug:
                        for i in range(lim_trajectory):
                            # rospy.loginfo( 'ik_l: {}'.format(self.ik_l_trajectory[i]) )
                            # rospy.loginfo( 'ik_r: {}'.format(self.ik_r_trajectory[i]) )
                            # input("Press enter to continue..")
                            print()
                            self.move_arm("left_arm"  , 1.0, self.ik_l_trajectory[i][0], self.ik_l_trajectory[i][1])
                            self.move_arm("right_arm" , 1.0, self.ik_r_trajectory[i][0], self.ik_r_trajectory[i][1])
                            sleep(1.5)
                    else:
                        ik_l_trajectory = np.array(self.ik_l_trajectory)
                        xl  = ik_l_trajectory[:lim_trajectory, 0]
                        yl  = ik_l_trajectory[:lim_trajectory, 1]
                        zl  = np.full(lim_trajectory, self.zl)
                        r_l = np.full(lim_trajectory, self.rl)
                        p_l = np.full(lim_trajectory, self.pl)
                        y_l = np.full(lim_trajectory, self.yl)

                        ik_r_trajectory = np.array(self.ik_r_trajectory)
                        xr  = ik_r_trajectory[:lim_trajectory, 0]
                        yr  = ik_r_trajectory[:lim_trajectory, 1]
                        zr  = np.full(lim_trajectory, self.zr)
                        r_r = np.full(lim_trajectory, self.rr)
                        p_r = np.full(lim_trajectory, self.pr)
                        y_r = np.full(lim_trajectory, self.yr)

                        # rospy.loginfo('[Main] ik_l_trajectory : {}'.format(ik_l_trajectory))
                        # rospy.loginfo('[Main] ik_r_trajectory : {}'.format(ik_r_trajectory))
                        # sleep(0.2)

                        kinematics.set_kinematics_arr_pose("left_arm",  0.01 , **{ 'total': lim_trajectory, 'x': xl, 'y': yl, 'z': zl, 'roll': r_l, 'pitch': p_l, 'yaw': y_l })
                        kinematics.set_kinematics_arr_pose("right_arm", 0.01 , **{ 'total': lim_trajectory, 'x': xr, 'y': yr, 'z': zr, 'roll': r_r, 'pitch': p_r, 'yaw': y_r })

                        sleep(1)
                        while kinematics.left_arr == True and kinematics.right_arr == True:
                            pass
                        sleep(2)

                        rospy.loginfo('[Main] Finish placement ...')
                        self.finish_placement_pub.publish(True)

                self.state = None

            self.main_rate.sleep()
        kinematics.kill_threads()

if __name__ == '__main__':
    pk = Placement_Keyboard()
    pk.run()