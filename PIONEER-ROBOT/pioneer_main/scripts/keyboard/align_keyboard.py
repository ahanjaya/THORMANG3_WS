#!/usr/bin/env python3

import yaml
import rospy
import rospkg
import numpy as np
from time import sleep
from std_msgs.msg import Bool
from geometry_msgs.msg import Point32
from pioneer_kinematics.kinematics import Kinematics

rospack  = rospkg.RosPack()
ws_path  = rospack.get_path("pioneer_main") + "/config/thormang3_align_keyboard_ws.yaml"
np.set_printoptions(suppress=True)

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

def grip_key_callback(msg):
    global state
    if msg.data == True:
        state = 'grip_keyboard'

def wait_robot(obj, msg):
    while obj.status_msg != msg:
        pass # do nothing

def load_ws_config(arm):
    try:
        with open(ws_path, 'r') as f:
            aruco_ws = yaml.safe_load(f)
            rospy.loginfo('[AK] Loaded {} workspace'.format(arm))
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

def move_arm(kinematics, arm, x, y):
    global prev_lik, prev_rik
    zl, rl, pl, yl = 0.63, 150, -1, -29 # 0.63
    zr, rr, pr, yr = 0.635, -150, -1, 29 # 0.64

    if arm == "left_arm":
        kinematics.set_kinematics_pose(arm , 2.0, **{ 'x': x, 'y': y, 'z': zl, 'roll': rl, 'pitch': pl, 'yaw': yl })
        prev_lik = (x, y)
    elif arm == "right_arm":
        kinematics.set_kinematics_pose(arm , 2.0, **{ 'x': x, 'y': y, 'z': zr, 'roll': rr, 'pitch': pr, 'yaw': yr })
        prev_rik = (x, y)

def main():
    rospy.init_node('pioneer_main_align_keyboard', anonymous=False)
    rospy.loginfo("[AK] Pioneer Align Keyboard - Running")

    rospy.Subscriber("/pioneer/target/left_arm_point",    Point32, left_arm_pos_callback)
    rospy.Subscriber("/pioneer/target/right_arm_point",   Point32, right_arm_pos_callback)
    rospy.Subscriber("/pioneer/target/start",             Bool,    arm_start_callback)
    rospy.Subscriber("/pioneer/target/sync_arm",          Bool,    arm_sync_callback)
    rospy.Subscriber("/pioneer/target/grasp_keyboard",    Bool,    grip_key_callback)
    rospy.Subscriber("/pioneer/init_pose",                Bool,    ini_pose_callback)

    l_sync_point_pub = rospy.Publisher("/pioneer/aruco/lsync_position", Point32, queue_size=1)

    main_rate = rospy.Rate(20)

    # global variables
    global start_ik, start_sync, grasp_keyboard, init_pose
    start_sync     = False
    start_ik       = False
    grasp_keyboard = False
    init_pose      = False

    global state
    global left_tar_x, left_tar_y, right_tar_x, right_tar_y
    global prev_lik, prev_rik
    left_tar_x,  left_tar_y  = None, None
    right_tar_x, right_tar_y = None, None
   
    state              = None
    lx_ik, ly_ik       = None, None
    rx_ik, ry_ik       = None, None
    lik_xmin, lik_xmax = None, None

    prev_lik, prev_rik = (), ()
    left_ws, right_ws  = {}, {}
    prev_l_tar         = (None, None)
    prev_r_tar         = (None, None)

    ##################
    ## Kinematics
    ##################
    kinematics = Kinematics()
    kinematics.publisher_(kinematics.module_control_pub,    "manipulation_module", latch=True)  # <-- Enable Manipulation mode
    kinematics.publisher_(kinematics.send_ini_pose_msg_pub, "align_keyboard_pose", latch=True)
    kinematics.publisher_(kinematics.en_align_key_pub,       True,                 latch=False) # <-- Enable Align Keboard mode
    wait_robot(kinematics, "End Init Trajectory")

    # set init head, torso, gripper
    kinematics.set_joint_pos(['head_p', 'head_y', 'torso_y'], [30, 0, 0])
    kinematics.set_gripper("left_arm", 0, 0)
    kinematics.set_gripper("right_arm", 0, 0)
    sleep(1)
    kinematics.set_joint_pos(['l_arm_finger45_p', 'r_arm_finger45_p'], [180, 180])
    sleep(1)
    rospy.loginfo('[AK] Finish Init Head & Hand')

    # load config file
    left_ws  = load_ws_config('left_arm')
    right_ws = load_ws_config('right_arm')

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
        if left_tar_x == -1 and left_tar_y == -1:
            lx_ik = ly_ik = None
        else:
            l_tar = (left_tar_x, left_tar_y)

        if right_tar_x == -1 and right_tar_y == -1:
            rx_ik = ry_ik = None
        else:
            r_tar = (right_tar_x, right_tar_y)

        if state == 'init_pose':
            rospy.loginfo('[AK] Robot State : {}'.format(state))

            if prev_lik and prev_rik:
                lx_ik = prev_lik[0]
                ly_ik = prev_lik[1] + 0.062
                move_arm(kinematics, "left_arm" , lx_ik, ly_ik)
                rx_ik = prev_rik[0]
                ry_ik = prev_rik[1] - 0.062
                move_arm(kinematics, "right_arm" , rx_ik, ry_ik)
                sleep(2.5)

                kinematics.publisher_(kinematics.send_ini_pose_msg_pub, "align_keyboard_pose", latch=False)
                kinematics.publisher_(kinematics.en_align_key_pub, True, latch=False) # <-- Enable Align Keboard mode
            else:
                kinematics.publisher_(kinematics.send_ini_pose_msg_pub, "align_keyboard_pose", latch=False)
                kinematics.publisher_(kinematics.en_align_key_pub, True, latch=False) # <-- Enable Align Keboard mode

            lx_ik = ly_ik = rx_ik = ry_ik = None
            prev_lik = prev_rik = ()
            state = None

        elif state == 'move_arm':
            rospy.loginfo('[AK] Robot State : {}'.format(state))

            if lx_ik != None and ly_ik != None \
                and rx_ik != None and ry_ik != None:
                print('move arm now')

                ly_ik += 0.05
                lx_ik -= 0.0 #0.025
                move_arm(kinematics, "left_arm" , lx_ik, ly_ik)
                ry_ik -= 0.05
                rx_ik -= 0.0
                move_arm(kinematics, "right_arm" , rx_ik, ry_ik)
            else:
                print('LX_IK: {}, LY_IK: {}'.format(lx_ik, ly_ik))
                print('RX_IK: {}, RY_IK: {}'.format(rx_ik, ry_ik))
                rospy.logwarn('[AK] Robot arm singularities \n Please move keyboard to workspace')
            state = None

        elif state == 'grip_keyboard':
            rospy.loginfo('[AK] Robot State : {}'.format(state))
            if prev_lik and prev_rik:
                lx_ik = prev_lik[0]
                ly_ik = prev_lik[1] - 0.06
                move_arm(kinematics, "left_arm" , lx_ik, ly_ik)

                rx_ik = prev_rik[0]
                ry_ik = prev_rik[1] + 0.06
                move_arm(kinematics, "right_arm" , rx_ik, ry_ik)
            state = None

        elif state == 'sync_move_arms':
            rospy.loginfo('[AK] Robot State : {}'.format(state))

            diff_ikr  = np.array([rx_ik, ry_ik]) - np.array(prev_rik)
            ikl_synch = prev_lik + diff_ikr
            lx_ik = ikl_synch[0]
            ly_ik = ikl_synch[1]

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
            
            lp = Point32()
            lp.x = lx_p
            lp.y = ly_p
            l_sync_point_pub.publish(lp)

            sleep(0.2)
            move_arm(kinematics, "right_arm" , rx_ik, ry_ik)
            move_arm(kinematics, "left_arm"  , lx_ik, ly_ik)
            state = None
            
        else:
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
    main()
    