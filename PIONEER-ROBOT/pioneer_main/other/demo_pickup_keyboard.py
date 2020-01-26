#!/usr/bin/env python3

import rospy
import numpy as np
from time import sleep
from pioneer_kinematics.kinematics import Kinematics

def wait(time_sec):
    keyboards_ = False
    if keyboards_ :
        input("Press [enter] for continue")
    else:
        sleep(time_sec)


def main():
    rospy.init_node('pioneer_main', anonymous=False)
    rospy.loginfo("Pioneer Main Demo Pick Up Keyboard- Running")

    kinematics = Kinematics()
    kinematics.publisher_(kinematics.module_control_pub, "manipulation_module", latch=True)  # <-- Enable Manipulation mode
    kinematics.publisher_(kinematics.send_ini_pose_msg_pub, "ini_pose", latch=True)
    sleep(2)

    kinematics.set_gripper("left_arm", 0, 5)
    kinematics.set_gripper("right_arm", 0, 5)

    while not rospy.is_shutdown():
        kinematics.set_kinematics_pose("right_arm", 1.0, **{ 'x': 0.20, 'y': -0.30, 'z': 0.60, 'roll': 90.00, 'pitch': 40.00, 'yaw': 0.00 })
        kinematics.set_kinematics_pose("left_arm" , 1.0, **{ 'x': 0.20, 'y':  0.30, 'z': 0.60, 'roll': -90.00, 'pitch': 40.00, 'yaw': 0.00 })
        wait(3)
        kinematics.set_kinematics_pose("right_arm", 2.0, **{ 'x': 0.40, 'y': -0.4, 'z': 0.62, 'roll': 90.00, 'pitch': 40.00, 'yaw': 0.00 })
        kinematics.set_kinematics_pose("left_arm", 2.0, **{ 'x': 0.40, 'y': 0.15, 'z': 0.62, 'roll': -90.00, 'pitch': 40.00, 'yaw': 0.00 })
        wait(3)
        kinematics.set_kinematics_pose("right_arm", 2.0, **{ 'x': 0.40, 'y': -0.33, 'z': 0.62, 'roll': 90.00, 'pitch': 40.00, 'yaw': 0.00 })
        kinematics.set_kinematics_pose("left_arm", 2.0, **{ 'x': 0.40, 'y': 0.08, 'z': 0.62, 'roll': -90.00, 'pitch': 40.00, 'yaw': 0.00 })
        wait(3)
        kinematics.set_gripper("left_arm", 3, 5)
        kinematics.set_gripper("right_arm", 3, 5)
        wait(0.3)
        kinematics.set_kinematics_pose("right_arm", 2.0, **{ 'x': 0.30, 'y': -0.2, 'z': 0.8, 'roll': 90.00, 'pitch': 30.00, 'yaw': 0.00 })
        kinematics.set_kinematics_pose("left_arm", 2.0, **{ 'x': 0.30, 'y': 0.2, 'z': 0.8, 'roll': -90.00, 'pitch': 30.00, 'yaw': 0.00 })
        wait(6)
        kinematics.set_kinematics_pose("right_arm", 2.0, **{ 'x': 0.40, 'y': -0.33, 'z': 0.62, 'roll': 90.00, 'pitch': 40.00, 'yaw': 0.00 })
        kinematics.set_kinematics_pose("left_arm", 2.0, **{ 'x': 0.40, 'y': 0.08, 'z': 0.62, 'roll': -90.00, 'pitch': 40.00, 'yaw': 0.00 })
        wait(3)
        kinematics.set_gripper("left_arm", 0, 5)
        kinematics.set_gripper("right_arm", 0, 5)
        kinematics.set_kinematics_pose("right_arm", 2.0, **{ 'x': 0.40, 'y': -0.4, 'z': 0.62, 'roll': 90.00, 'pitch': 40.00, 'yaw': 0.00 })
        kinematics.set_kinematics_pose("left_arm", 2.0, **{ 'x': 0.40, 'y': 0.15, 'z': 0.62, 'roll': -90.00, 'pitch': 40.00, 'yaw': 0.00 })
        wait(3)
        kinematics.publisher_(kinematics.send_ini_pose_msg_pub, "ini_pose")
        break

    kinematics.kill_threads()

if __name__ == '__main__':
    main()