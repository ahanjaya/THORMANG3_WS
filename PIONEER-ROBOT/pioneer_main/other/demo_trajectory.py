#!/usr/bin/env python3

import rospy
from time import sleep
from pioneer_motor.motor import Motor
from pioneer_kinematics.kinematics import Kinematics

def main():
    rospy.init_node('pioneer_trajectory', anonymous=False)
    rospy.loginfo("[Tra] Pioneer Demo Trajectory - Running")

    ## Trajectory
    kinematics = Kinematics()
    kinematics.publisher_(kinematics.module_control_pub, "manipulation_module", latch=True)  # <-- Enable Manipulation mode
    kinematics.publisher_(kinematics.send_ini_pose_msg_pub, "ini_pose", latch=True)
    sleep(2)

    # mode = 'full'
    mode = 'left_arm'

    if mode == "full":
        # trajectory simulation circle
        kinematics.set_kinematics_pose("left_arm" , 2.0,  **{ 'x': 0.20, 'y':  0.30, 'z': 0.80, 'roll': 0.00, 'pitch': 0.00, 'yaw': 0.00 })
        kinematics.set_kinematics_pose("right_arm" , 2.0, **{ 'x': 0.20, 'y':  -0.30, 'z': 0.80, 'roll': 0.00, 'pitch': 0.00, 'yaw': 0.00 })

    sleep(0.1)
    while kinematics.left_tra == True and kinematics.right_tra == True:
        pass
    sleep(1)
 
    iter = 0
    
    while not rospy.is_shutdown():
        if mode == "full":
            kinematics.trajectory_sin(group="left_arm",  x=0.30, y=0.20,  z=0.90, roll=0.0, pitch=0.0, yaw=0.0, xc=-0.1, yc=-0.1, zc=-0.1, time=2, res=0.01)
            kinematics.trajectory_sin(group="right_arm", x=0.30, y=-0.20, z=0.90, roll=0.0, pitch=0.0, yaw=0.0, xc=-0.1, yc=0.1,  zc=-0.1, time=2, res=0.01)

            sleep(1) # because pubslishing array msg using for loop
            while kinematics.left_arr == True and kinematics.right_arr == True:
                pass # no action

            kinematics.trajectory_sin(group="left_arm",  x=0.20, y=0.30, z=0.85, roll=0.0, pitch=0.0, yaw=0.0, xc=0.1, yc=0.1, zc=0.1, time=2, res=0.01)
            kinematics.trajectory_sin(group="right_arm", x=0.20, y=-0.30, z=0.85, roll=0.0, pitch=0.0, yaw=0.0, xc=0.1, yc=-0.1, zc=0.1, time=2, res=0.01)
            sleep(1) # because pubslishing array msg using for loop

            while kinematics.left_arr == True and kinematics.right_arr == True:
                pass # no action

            iter += 1
            rospy.loginfo("[Tra] Iteration : {}".format(iter))
            if iter >= 2:
                mode = "none"

        elif mode == "left_arm":
            left_arm = kinematics.get_kinematics_pose("left_arm")
            print(left_arm)

            kinematics.set_kinematics_pose("left_arm" , 2.0,  **{ 'x': 0.40, 'y':  0.50, 'z': 0.80, 'roll': 0.00, 'pitch': 0.00, 'yaw': 0.00 })

            while kinematics.left_arr == True
                pass
            
            mode = "none"
            # pass

        elif mode == "none":
            break


    kinematics.publisher_(kinematics.send_ini_pose_msg_pub, "ini_pose", latch=True)
    kinematics.kill_threads()

if __name__ == '__main__':
    main()
    