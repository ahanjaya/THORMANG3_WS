#!/usr/bin/env python3

import rospy
import numpy as np
from time import sleep
from std_msgs.msg import String
from pioneer_motor.motor import Motor
from pioneer_motion.motion import Motion
from pioneer_kinematics.kinematics import Kinematics

class Cross_Arm:
    def __init__(self):
        rospy.init_node('pioneer_cross_arm', anonymous=False)
        rospy.loginfo("[CA] Pioneer Main Cross Arm- Running")
        
        self.kinematics = Kinematics()
        self.motion     = Motion()
        self.motor      = Motor()
        self.prev_arm   = None

    def wait_robot(self, obj, msg):
        while obj.status_msg != msg:
            pass # do nothing

    def run(self):
        kinematics = self.kinematics
        motion     = self.motion
        motor      = self.motor

        kinematics.publisher_(kinematics.module_control_pub, "manipulation_module", latch=True)  # <-- Enable Manipulation mode
        kinematics.publisher_(kinematics.send_ini_pose_msg_pub, "ini_pose", latch=True)
        self.wait_robot(kinematics, "End Init Trajectory")
        rospy.loginfo('[CA] Manipulation Init Pose...')

        # while not rospy.is_shutdown():
        #     kinematics.set_joint_pos(['head_y', 'torso_y', 'head_p'], [0, 20, 45])
        #     # kinematics.set_gripper("left_arm", 0, 5)
        #     # kinematics.set_gripper("right_arm", 0, 5)
        #     break

        while not rospy.is_shutdown():
            scan = input('Please input : ')
            if scan == "rotate":
                motor.publisher_(motor.module_control_pub, "none", latch=True)
                # motor.set_joint_states(["torso_y"], [20], [0], [5])
                motor.set_joint_states(["torso_y"], False)

            elif scan == "down":
                motion.publisher_(motion.module_control_pub, "head_control_module", latch=True)
                motion.set_head_joint_states(['head_y', 'head_p'], [0, 25])
                self.wait_robot(motion, "Head movement is finished.")
            elif scan == "scan":
                # motion.publisher_(motion.move_lidar_pub, "start")
                motion.publisher_(motion.move_lidar_range_pub, np.radians(30+1)) # scan head with range
                self.wait_robot(motion, "Finish head joint in order to make pointcloud")
                # motion.set_head_joint_states(['head_y', 'head_p'], [0, 25])
                # self.wait_robot(motion, "Head movement is finished.")

            elif scan == "left cross":
                if self.prev_arm == None:
                    self.prev_arm = scan
                    kinematics.publisher_(kinematics.module_control_pub, "manipulation_module", latch=True)
                    kinematics.set_joint_pos(['l_arm_thumb_p', 'l_arm_middle_p', 'l_arm_finger45_p'], [-60, 180, 180])
                    kinematics.set_kinematics_pose("left_arm", 2.0, **{ 'x': 0.4, 'y': 0.05, 'z': 0.80, 'roll': 0.00, 'pitch': 0.00, 'yaw': -10.00 })
                else:
                    rospy.logwarn('[CA] Please init first')
            elif scan == "right cross":
                if self.prev_arm == None:
                    self.prev_arm = scan
                    kinematics.publisher_(kinematics.module_control_pub, "manipulation_module", latch=True)
                    kinematics.set_joint_pos(['r_arm_thumb_p', 'r_arm_middle_p', 'r_arm_finger45_p'], [-60, 180, 180])
                    kinematics.set_kinematics_pose("right_arm", 2.0, **{ 'x': 0.4, 'y': -0.05, 'z': 0.80, 'roll': 0.00, 'pitch': 0.00, 'yaw': 10.00 })
                else:
                    rospy.logwarn('[CA] Please init first')
            
            elif scan == "left":
                if self.prev_arm == None:
                    self.prev_arm = scan
                    kinematics.publisher_(kinematics.module_control_pub, "manipulation_module", latch=True)
                    kinematics.set_joint_pos(['l_arm_thumb_p', 'l_arm_middle_p', 'l_arm_finger45_p'], [-60, 180, 180])
                    kinematics.set_kinematics_pose("left_arm", 2.0, **{ 'x': 0.4, 'y': 0.2, 'z': 0.80, 'roll': 0.00, 'pitch': 0.00, 'yaw': 10.00 })
                else:
                    rospy.logwarn('[CA] Please init first')
            elif scan == "right":
                if self.prev_arm == None:
                    self.prev_arm = scan
                    kinematics.publisher_(kinematics.module_control_pub, "manipulation_module", latch=True)
                    kinematics.set_joint_pos(['r_arm_thumb_p', 'r_arm_middle_p', 'r_arm_finger45_p'], [-60, 180, 180])
                    kinematics.set_kinematics_pose("right_arm", 2.0, **{ 'x': 0.4, 'y': -0.2, 'z': 0.80, 'roll': 0.00, 'pitch': 0.00, 'yaw': -10.00 })
                else:
                    rospy.logwarn('[CA] Please init first')

            elif scan == "init":
                kinematics.publisher_(kinematics.send_ini_pose_msg_pub, "ini_pose", latch=True)
                kinematics.set_gripper("left_arm", 0, 5)
                kinematics.set_gripper("right_arm", 0, 5)
                self.prev_arm = None

            elif scan == "exit":
                break                
            else:
                rospy.loginfo("[CA] Wrong input")

        kinematics.kill_threads()
        motion.kill_threads()
        motor.kill_threads()

if __name__ == '__main__':
    ca = Cross_Arm()
    ca.run()