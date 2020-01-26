#!/usr/bin/env python3

import sys
import yaml
import rospy
import rospkg
import numpy as np
from time import sleep
from std_msgs.msg import Bool
from geometry_msgs.msg import Point32
from pioneer_kinematics.kinematics import Kinematics

rospack = rospkg.RosPack()
np.set_printoptions(suppress=True)

def left_aruco_pos_callback(msg):
    global lmarker_x, lmarker_y
    lmarker_x = msg.x
    lmarker_y = msg.y

def right_aruco_pos_callback(msg):
    global rmarker_x, rmarker_y
    rmarker_x = msg.x
    rmarker_y = msg.y

def left_arm_pos_callback(msg):
    global left_tar_x, left_tar_y
    left_tar_x = msg.x
    left_tar_y = msg.y

def right_arm_pos_callback(msg):
    global right_tar_x, right_tar_y
    right_tar_x = msg.x
    right_tar_y = msg.y

def arm_start_callback(msg):
    global state
    if msg.data == True:
        state = 'move_arm'

def arm_sync_callback(msg):
    global state
    if msg.data == True:
        state = 'sync_move_arms'

def ini_pose_callback(msg):
    global state
    if msg.data == True:
        state = 'init_pose'

def typing_pose_callback(msg):
    global state
    if msg.data == True:
        state = 'typing'

def wait_robot(obj, msg):
    sleep(0.1)
    while obj.status_msg != msg:
        pass # do nothing

def logging(obj, arm, x, y, idx, config):
    global lmarker_x, lmarker_y, rmarker_x, rmarker_y
    # buffering
    # sleep(1)
    wait_robot(obj, "End " + arm + " Trajectory")
    sleep(1)

    # wait until aruco stable
    if arm == 'Left Arm':
        while lmarker_x == -1 or lmarker_y == -1:
            pass
        config['P'+str(idx)] = dict(ik_x = x, ik_y = y, cx = lmarker_x, cy = lmarker_y)

    elif arm == 'Right Arm':
        while rmarker_x == -1 or rmarker_y == -1:
            pass
        config['P'+str(idx)] = dict(ik_x = x, ik_y = y, cx = rmarker_x, cy = rmarker_y)
    
    print('{0} (Points: {1})  IK X: {2} \t IK_Y: {3} \t CX:{4} \t CY:{5}'.format(arm, idx, x, y, lmarker_x, lmarker_y))

def load_config(arm):
    global config_path
    try:
        with open(config_path, 'r') as f:
            aruco_ws = yaml.safe_load(f)
            rospy.loginfo('[MC] Loaded {} workspace'.format(arm))
        return aruco_ws[arm]

    except yaml.YAMLError as exc:
        print(exc)
        return None

def parse_Yframe(data):
    y_frame_min = np.mean([ data['P2']['cy'], data['P3']['cy'] ], dtype=int)
    y_frame_max = np.mean([ data['P1']['cy'], data['P4']['cy'] ], dtype=int)

    # y_frame_min = ( data['P2']['cy'], data['P3']['cy'] )
    # y_frame_max = ( data['P1']['cy'], data['P4']['cy'] )

    ik_xmin = data['P1']['ik_x']
    ik_xmax = data['P2']['ik_x']

    return ( y_frame_min, y_frame_max, ik_xmin, ik_xmax )

def parse_Xframe(data):
    x_frame_min = ( data['P1']['cx'], data['P2']['cx'] )
    x_frame_max = ( data['P3']['cx'], data['P4']['cx'] )

    ik_ymin_lower = data['P4']['ik_y'] #0.1
    ik_ymax_lower = data['P1']['ik_y'] #0.24
    ik_ymin_upper = data['P3']['ik_y'] #0.05
    ik_ymax_upper = data['P2']['ik_y'] #0.34

    return ( x_frame_min, x_frame_max, ik_ymax_lower, ik_ymax_upper, ik_ymin_lower, ik_ymin_upper)

def main(mode):
    # global variables
    global state, config_path
    global lmarker_x, lmarker_y, rmarker_x, rmarker_y
    global left_tar_x, left_tar_y, right_tar_x, right_tar_y
    left_tar_x,  left_tar_y  = None, None
    right_tar_x, right_tar_y = None, None
   
    state              = None
    scan_workspace     = False
    save_config        = False

    lx_ik, ly_ik       = None, None
    rx_ik, ry_ik       = None, None
    lik_xmin, lik_xmax = None, None

    prev_lik, prev_rik = (), ()
    left_ws, right_ws  = {}, {}
    prev_l_tar         = (None, None)
    prev_r_tar         = (None, None)

    if mode == "align_keyboard":
        config_path  = rospack.get_path("pioneer_main") + "/config/thormang3_align_keyboard_ws.yaml"
    elif mode == "typing":
        config_path  = rospack.get_path("pioneer_main") + "/config/thormang3_typing_ws.yaml"

    # Subscriber
    rospy.Subscriber("/pioneer/aruco/left_position",    Point32, left_aruco_pos_callback)
    rospy.Subscriber("/pioneer/aruco/right_position",   Point32, right_aruco_pos_callback)
    rospy.Subscriber("/pioneer/target/left_arm_point",  Point32, left_arm_pos_callback)
    rospy.Subscriber("/pioneer/target/right_arm_point", Point32, right_arm_pos_callback)
    rospy.Subscriber("/pioneer/target/start",           Bool,    arm_start_callback)
    rospy.Subscriber("/pioneer/target/sync_arm",        Bool,    arm_sync_callback)
    rospy.Subscriber("/pioneer/init_pose",              Bool,    ini_pose_callback)
    rospy.Subscriber("/pioneer/typing",                 Bool,    typing_pose_callback)

    # Publisher
    l_sync_point_pub = rospy.Publisher("/pioneer/aruco/lsync_position", Point32, queue_size=1)

    main_rate = rospy.Rate(20)

    # Kinematics
    kinematics = Kinematics()
    kinematics.publisher_(kinematics.module_control_pub, "manipulation_module", latch=True)  # <-- Enable Manipulation mode
    if mode == "align_keyboard":
        kinematics.publisher_(kinematics.send_ini_pose_msg_pub, "align_keyboard_pose", latch=True)
        kinematics.publisher_(kinematics.en_align_key_pub, True, latch=False) # <-- Enable Align Keboard mode

    elif mode == "typing":
        kinematics.publisher_(kinematics.send_ini_pose_msg_pub, "typing_pose", latch=True)
        kinematics.publisher_(kinematics.en_typing_pub, True, latch=False) # <-- Enable Typing mode

    wait_robot(kinematics, "End Init Trajectory")

    # set init head, torso, gripper
    kinematics.set_joint_pos(['head_p', 'head_y', 'torso_y'], [30, 0, 0])
    kinematics.set_gripper("left_arm", 0, 0)
    kinematics.set_gripper("right_arm", 0, 0)
    sleep(1)
    rospy.loginfo('[MC] Finish Init Head & Hand')

    # set finger
    if mode == "align_keyboard":
        kinematics.set_joint_pos(['l_arm_finger45_p', 'r_arm_finger45_p'], [180, 180])
        lik_ymin_lower = 0.00 #0.1
        lik_ymax_lower = 0.28 #0.26
        lik_ymin_upper = 0.00 #0.05
        lik_ymax_upper = 0.28 #0.34
        lik_xmin       = 0.25 #0.2
        lik_xmax       = 0.55 #0.45        
        zl, rl, pl, yl = 0.65, 150, -1, -29
        zr, rr, pr, yr = 0.65, -150, -1, 29

    elif mode == "typing":
        kinematics.set_joint_pos(['l_arm_index_p', 'l_arm_thumb_p'], [-86, -70])
        kinematics.set_joint_pos(['r_arm_index_p', 'r_arm_thumb_p'], [-86, -70])

        lik_ymin_lower = -0.02
        lik_ymax_lower = 0.28
        lik_ymin_upper = -0.05
        lik_ymax_upper = 0.34
        lik_xmin       = 0.25 
        lik_xmax       = 0.55        
        zl, rl, pl, yl = 0.72, 10, 0, 0
        zr, rr, pr, yr = 0.74, -10, 0, 0
    
    lp1 = (lik_xmin, lik_ymax_lower)
    lp2 = (lik_xmax, lik_ymax_upper)
    lp3 = (lik_xmax, lik_ymin_upper)
    lp4 = (lik_xmin, lik_ymin_lower)
    left_arm_pts = [ lp1, lp2, lp3, lp4 ]

    rik_xmin       = lik_xmin
    rik_xmax       = lik_xmax 
    rik_ymin_lower = lik_ymin_lower * -1
    rik_ymax_lower = lik_ymax_lower * -1
    rik_ymin_upper = lik_ymin_upper * -1
    rik_ymax_upper = lik_ymax_upper * -1
    rp1 = (rik_xmin, rik_ymax_lower)
    rp2 = (rik_xmax, rik_ymax_upper)
    rp3 = (rik_xmax, rik_ymin_upper)
    rp4 = (rik_xmin, rik_ymin_lower)
    right_arm_pts = [ rp1, rp2, rp3, rp4 ]

    if scan_workspace:
        # Left Arm
        for idx, pts in enumerate(left_arm_pts):
            x, y = pts
            kinematics.set_kinematics_pose("left_arm" , 4.0, **{ 'x': x, 'y': y, 'z': zl, 'roll': rl, 'pitch': pl, 'yaw': yl })
            logging(kinematics, 'Left Arm', x, y, idx+1, left_ws)
        # move to P1
        kinematics.set_kinematics_pose("left_arm" , 4.0, **{ 'x': left_arm_pts[0][0], 'y': left_arm_pts[0][1], 'z': zl, 'roll': rl, 'pitch': pl, 'yaw': yl })

        # Right Arm
        for idx, pts in enumerate(right_arm_pts):
            x, y = pts
            kinematics.set_kinematics_pose("right_arm" , 4.0, **{ 'x': x, 'y': y, 'z': zr, 'roll': rr, 'pitch': pr, 'yaw': yr })
            logging(kinematics, 'Right Arm', x, y, idx+1, right_ws)
        # move to P1
        kinematics.set_kinematics_pose("right_arm" , 4.0, **{ 'x': right_arm_pts[0][0], 'y': right_arm_pts[0][1], 'z': zr, 'roll': rr, 'pitch': pr, 'yaw': yr })

        if save_config:
            aruco_ws = {}
            aruco_ws['left_arm']  = left_ws
            aruco_ws['right_arm'] = right_ws
            with open(config_path, 'w') as f:
                yaml.dump(aruco_ws, f, default_flow_style=False)
    else:
        left_ws  = load_config('left_arm')
        right_ws = load_config('right_arm')

    if left_ws != None:
        yleft_data    = parse_Yframe(left_ws)
        ly_frame_min  = yleft_data[0]
        ly_frame_max  = yleft_data[1]
        ly_ws         = range(ly_frame_min, ly_frame_max+1)
        lik_xmin      = yleft_data[2]
        lik_xmax      = yleft_data[3]

        xleft_data    = parse_Xframe(left_ws)
        lx_min_b      = xleft_data[0][0]
        lx_min_a      = xleft_data[0][1]
        lx_max_a      = xleft_data[1][0]
        lx_max_b      = xleft_data[1][1]

        lik_ymax_lower = xleft_data[2] #0.24
        lik_ymax_upper = xleft_data[3] #0.34
        lik_ymin_lower = xleft_data[4] #0.1
        lik_ymin_upper = xleft_data[5] #0.05

    if right_ws != None:
        yright_data   = parse_Yframe(right_ws)
        ry_frame_min  = yright_data[0]
        ry_frame_max  = yright_data[1]
        ry_ws         = range(ry_frame_min, ry_frame_max+1)
        rik_xmin      = yright_data[2]
        rik_xmax      = yright_data[3]

        xright_data   = parse_Xframe(right_ws)
        rx_max_b      = xright_data[0][0]
        rx_max_a      = xright_data[0][1]
        rx_min_a      = xright_data[1][0]
        rx_min_b      = xright_data[1][1]

        rik_ymax_lower = xright_data[2] #0.24
        rik_ymax_upper = xright_data[3] #0.34
        rik_ymin_lower = xright_data[4] #0.1
        rik_ymin_upper = xright_data[5] #0.05

    while not rospy.is_shutdown():
        # lx_tar, ly_tar = left_tar_x, left_tar_y
        # rx_tar, ry_tar= right_tar_x, right_tar_y
        l_tar = (left_tar_x, left_tar_y)
        r_tar = (right_tar_x, right_tar_y)

        if state == 'init_pose':
            rospy.loginfo('[MC] Robot State : {}'.format(state))
            if mode == "align_keyboard":
                kinematics.publisher_(kinematics.send_ini_pose_msg_pub, "align_keyboard_pose", latch=False)
                kinematics.publisher_(kinematics.en_align_key_pub, True, latch=False) # <-- Enable Align Keboard mode
            elif mode == "typing":
                kinematics.publisher_(kinematics.send_ini_pose_msg_pub, "typing_pose", latch=False)
                kinematics.publisher_(kinematics.en_typing_pub, True, latch=False) # <-- Enable Typing mode

            lx_ik = ly_ik = rx_ik = ry_ik = None
            prev_lik = prev_rik = ()
            state = None

        elif state == 'move_arm':
            rospy.loginfo('[MC] Robot State : {}'.format(state))

            if lx_ik != None and ly_ik != None:
                kinematics.set_kinematics_pose("left_arm" , 2.0, **{ 'x': lx_ik, 'y': ly_ik, 'z': zl, 'roll': rl, 'pitch': pl, 'yaw': yl })
                prev_lik = (lx_ik, ly_ik)
                last_arm = "left_arm"
            if rx_ik != None and ry_ik != None:
                kinematics.set_kinematics_pose("right_arm" , 2.0, **{ 'x': rx_ik, 'y': ry_ik, 'z': zr, 'roll': rr, 'pitch': pr, 'yaw': yr })
                prev_rik = (rx_ik, ry_ik)
                last_arm = "right_arm"
            state = None

        elif state == "typing":
            if last_arm == 'left_arm':
                zl_typing    = 0.03
                kinematics.set_kinematics_pose("left_arm" , 0.4, **{ 'x': lx_ik, 'y': ly_ik, 'z': zl-zl_typing, 'roll': rl, 'pitch': pl, 'yaw': yl })
                wait_robot(kinematics, "End Left Arm Trajectory")
                kinematics.set_kinematics_pose("left_arm" , 0.4, **{ 'x': lx_ik, 'y': ly_ik, 'z': zl+zl_typing, 'roll': rl, 'pitch': pl, 'yaw': yl })
            
            elif last_arm == 'right_arm':
                zr_typing    = 0.03
                kinematics.set_kinematics_pose("right_arm" , 0.4, **{ 'x': rx_ik, 'y': ry_ik, 'z': zr-zr_typing, 'roll': rr, 'pitch': pr, 'yaw': yr })
                wait_robot(kinematics, "End Right Arm Trajectory")
                kinematics.set_kinematics_pose("right_arm" , 0.4, **{ 'x': rx_ik, 'y': ry_ik, 'z': zr+zr_typing, 'roll': rr, 'pitch': pr, 'yaw': yr })

            state = None

        elif state == 'sync_move_arms':
            rospy.loginfo('[MC] Robot State : {}'.format(state))

            if prev_rik and prev_lik:
                rospy.loginfo("[MC] Finish Sync Calculating")
                diff_ikr  = np.array([rx_ik, ry_ik]) - np.array(prev_rik)
                ikl_synch = prev_lik + diff_ikr
                lx_ik     = ikl_synch[0]
                ly_ik     = ikl_synch[1]

                ly_p = np.interp( lx_ik, [lik_xmin, lik_xmax], [ly_frame_max, ly_frame_min] )
                # Mapping CX Frame
                lx_frame_min = np.interp( ly_p, [ly_frame_min, ly_frame_max], [lx_min_a, lx_min_b] )
                lx_frame_min = int( np.round(lx_frame_min, 0) )
                lx_frame_max = np.interp( ly_p, [ly_frame_min, ly_frame_max], [lx_max_a, lx_max_b] )
                lx_frame_max = int( np.round(lx_frame_max, 0) )
                # Mapping IK_Y
                lik_ymax     = np.interp( ly_p, [ly_frame_min, ly_frame_max], [lik_ymax_upper, lik_ymax_lower] )
                lik_ymax     = np.round(lik_ymax, 4)
                lik_ymin     = np.interp( ly_p, [ly_frame_min, ly_frame_max], [lik_ymin_upper, lik_ymin_lower] )
                lik_ymin     = np.round(lik_ymin, 4)
                lx_p         = np.interp( ly_ik, [lik_ymin, lik_ymax], [lx_frame_max, lx_frame_min] )
                
                lp   = Point32()
                lp.x = lx_p
                lp.y = ly_p
                l_sync_point_pub.publish(lp)
            state = None

        else :
            if prev_l_tar != l_tar:
                if l_tar[1] in ly_ws:
                    # Mapping CY Frame
                    # ly_frame_min = np.interp( l_tar[1], [ly_frame_min, ly_frame_max], [ly_min_a, ly_min_b] )
                    # ly_frame_min = int( np.round(ly_frame_min, 0) )

                    # Left IK X Target
                    lx_ik = np.interp( l_tar[1], [ly_frame_min, ly_frame_max], [lik_xmax, lik_xmin] )

                    # Mapping CX Frame
                    lx_frame_min = np.interp( l_tar[1], [ly_frame_min, ly_frame_max], [lx_min_a, lx_min_b] )
                    lx_frame_min = int( np.round(lx_frame_min, 0) )
                    lx_frame_max = np.interp( l_tar[1], [ly_frame_min, ly_frame_max], [lx_max_a, lx_max_b] )
                    lx_frame_max = int( np.round(lx_frame_max, 0) )

                    # Mapping IK_Y
                    lik_ymax     = np.interp( l_tar[1], [ly_frame_min, ly_frame_max], [lik_ymax_upper, lik_ymax_lower] )
                    lik_ymax     = np.round(lik_ymax, 4)
                    lik_ymin     = np.interp( l_tar[1], [ly_frame_min, ly_frame_max], [lik_ymin_upper, lik_ymin_lower] )
                    lik_ymin     = np.round(lik_ymin, 4)

                    lx_ws = range(lx_frame_min, lx_frame_max+1)
                    if l_tar[0] in lx_ws:
                        # Left IK Y Target
                        ly_ik = np.interp( l_tar[0], [lx_frame_min, lx_frame_max], [lik_ymax, lik_ymin] )

                        print()
                        rospy.loginfo('[Left Arm] Input Coor X: {0}, Y: {1}'.format(l_tar[0], l_tar[1]))
                        rospy.loginfo('[Left Arm] X_IK: {0:.2f}, Y_IK: {1:.2f}'.format(lx_ik, ly_ik))
                    else:
                        rospy.logerr('[Left Arm] X Frame target is out of range')
                else:
                    rospy.logerr('[Left Arm] Y Frame target is out of range')
                prev_l_tar = l_tar  

            if prev_r_tar != r_tar:
                if r_tar[1] in ry_ws:
                    # Left IK X Target
                    rx_ik = np.interp( r_tar[1], [ry_frame_min, ry_frame_max], [rik_xmax, rik_xmin] )

                    # Mapping CX Frame
                    rx_frame_min = np.interp( r_tar[1], [ry_frame_min, ry_frame_max], [rx_min_a, rx_min_b] )
                    rx_frame_min = int( np.round(rx_frame_min, 0) )
                    rx_frame_max = np.interp( r_tar[1], [ry_frame_min, ry_frame_max], [rx_max_a, rx_max_b] )
                    rx_frame_max = int( np.round(rx_frame_max, 0) )

                    # Mapping IK_Y
                    rik_ymax     = np.interp( r_tar[1], [ry_frame_min, ry_frame_max], [rik_ymax_upper, rik_ymax_lower] )
                    rik_ymax     = np.round(rik_ymax, 4)
                    rik_ymin     = np.interp( r_tar[1], [ry_frame_min, ry_frame_max], [rik_ymin_upper, rik_ymin_lower] )
                    rik_ymin     = np.round(rik_ymin, 4)

                    rx_ws = range(rx_frame_min, rx_frame_max+1)
                    if r_tar[0] in rx_ws:
                        # Left IK Y Target
                        ry_ik = np.interp( r_tar[0], [rx_frame_min, rx_frame_max], [rik_ymin, rik_ymax] )

                        print()
                        rospy.loginfo('[Right Arm] Input Coor X: {0}, Y: {1}'.format(r_tar[0], r_tar[1]))
                        rospy.loginfo('[Right Arm] X_IK: {0:.2f}, Y_IK: {1:.2f}'.format(rx_ik, ry_ik))
                    else:
                        rospy.logerr('[Right Arm] X Frame target is out of range')
                else:
                    rospy.logerr('[Right Arm] Y Frame target is out of range')
                prev_r_tar = r_tar

        main_rate.sleep()
        
    kinematics.kill_threads()

if __name__ == '__main__':
    # print('Number of arguments:', len(sys.argv), 'arguments.')
    # print('Argument List:', str(sys.argv))

    rospy.init_node('pioneer_mouse_click', anonymous=False)

    # if using ros launch length of sys.argv is 4
    if len(sys.argv) == 4:
        if sys.argv[1] == "align_keyboard" or sys.argv[1] == "typing":
            rospy.loginfo("[MC] Pioneer Calibration : {}".format(sys.argv[1]))

            main(sys.argv[1])
        else:
            rospy.logerr("[MC] Exit Unknown Mode")
    else:
        rospy.logerr("[MC] Exit Argument not fulfilled")