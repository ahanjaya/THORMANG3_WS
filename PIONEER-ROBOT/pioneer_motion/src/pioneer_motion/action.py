#!/usr/bin/env python3

import yaml
import rospy
import rospkg
import threading
import numpy as np
from time import sleep
from std_msgs.msg import String
from pioneer_motor.motor import Motor

class Action:
    def __init__(self, robot_name):
        self.robot_name  = robot_name
        rospack          = rospkg.RosPack()

        if self.robot_name == "Thormang3_Wolf":
            self.motion_path = rospack.get_path("pioneer_motion") + "/config/thormang3_wolf_motion_bin.yaml"
        elif self.robot_name == "Thormang3_Bear":
            self.motion_path = rospack.get_path("pioneer_motion") + "/config/thormang3_bear_motion_bin.yaml"

        self.motor         = Motor(self.robot_name)
        self.pub_rate      = rospy.Rate(10)
        self.main_rate     = rospy.Rate(10)
        self.torque_flag   = True
        self.motion        = {}
        self.finish_action = False

        self.joint_id_to_name = {  1: "r_arm_sh_p1", 2: "l_arm_sh_p1", 3: "r_arm_sh_r",  4: "l_arm_sh_r",  5: "r_arm_sh_p2", 6: "l_arm_sh_p2", 7: "r_arm_el_y",  
                                   8: "l_arm_el_y",  9: "r_arm_wr_r", 10: "l_arm_wr_r", 11: "r_arm_wr_y", 12: "l_arm_wr_y", 13: "r_arm_wr_p", 14: "l_arm_wr_p",
                                   27: "torso_y",   28: "head_y",     29: "head_p",     30: "l_arm_grip", 31: "r_arm_grip" }

        self.filtered_join = [ "r_arm_sh_p1", "l_arm_sh_p1", "r_arm_sh_r", "l_arm_sh_r", "r_arm_sh_p2", "l_arm_sh_p2", "r_arm_el_y", "l_arm_el_y",  
                               "r_arm_wr_r",  "l_arm_wr_r",  "r_arm_wr_y", "l_arm_wr_y", "r_arm_wr_p",  "l_arm_wr_p",  "r_arm_grip", "l_arm_grip" ]

    def kill_threads(self):
        self.motor.kill_threads()

    def publisher_(self, topic, msg, latch=False):
        if latch:
            for _ in range(4):
                topic.publish(msg)
                self.pub_rate.sleep()
        else:
            topic.publish(msg)

    def save_motion(self):
        with open(self.motion_path, 'w') as f:
            yaml.dump(self.motion, f, default_flow_style=False)

    def load_motion(self):
        try:
            with open(self.motion_path, 'r') as f:
                data = yaml.safe_load(f)
                if data != None:
                    self.motion = data
                    # print('[Action] Loaded motion')
        except yaml.YAMLError as exc:
            print(exc)

    def set_torque(self, level):
        self.motor.set_joint_torque(['all'], level)
        print('[Action] Set torque level: {}%'.format(level))

    def set_velocity(self, level):
        self.motor.set_joint_velocity(['all'], level)
        print('[Action] Set velocity level: {}%'.format(level))

    def set_init_config(self, torque=80, velocity=0):
        self.set_torque(torque) # 30%
        sleep(0.1)
        self.set_velocity(velocity) # 30%

    def play_sub_motion(self, header_motion, sub_motion, set_motion=False):
        temp_motion = self.motion[header_motion][sub_motion].copy()   
        print('\t Motion {} {}'.format(header_motion, sub_motion))

        interval = temp_motion['time']
        speed    = temp_motion['velocity']
        del temp_motion['time']
        del temp_motion['velocity']
        self.set_velocity(speed)

        joint_names    = list(temp_motion.keys())
        joint_position = list(temp_motion.values())
        joint_velocity = [ 15 for _ in joint_names ]  # velocity unit in (0%-100%)
        joint_effort   = [ 20 for _ in joint_names ]  # torque unit in (0%-100%) # 15
        self.motor.set_joint_states(joint_names, joint_position, joint_velocity, joint_effort)

        if set_motion:
            if sub_motion == 1:
                if interval == 0:
                    sleep(0.5)
                    # while self.motor.moving:
                    #     pass
                else:
                    sleep(interval)
            else:
                if interval != 0:
                    sleep(interval)
                else:
                    sleep(0.5)
                    # while self.motor.moving:
                    #     pass
        else:
            sleep(0.5)
            # while self.motor.moving:
            #     pass

    def play_motion(self, motion_name):
        self.load_motion()

        if motion_name in self.motion:
            self.finish_action = False
            total_sub = len(self.motion[motion_name])

            if total_sub == 1:
                self.play_sub_motion(motion_name, 1)

            else:
                for sub_motion in self.motion[motion_name]:
                    self.play_sub_motion(motion_name, sub_motion, set_motion=True)

            self.finish_action = True
        else:
            rospy.loginfo('[Action] Invalid motion name')

    def play_motion_thread(self, motion_name):
        thread1 = threading.Thread(target = self.play_motion,  args=(motion_name, ))
        thread1.start()

    def torque_off(self, joint_id):
        joint = [ self.joint_id_to_name[int(id)] for id in joint_id ]
        print('[Action] Torque off: {}'.format(joint))
        self.motor.set_joint_states(joint, False)

    def run(self):
        motor = self.motor
        motor.publisher_(motor.module_control_pub, "none", latch=True)
        self.load_motion()

        if self.robot_name == "Thormang3_Bear":
            self.set_init_config(torque=50)

        while not rospy.is_shutdown():
            print()
            scan = input('[Action] Input : ')

            if scan == 'q' or scan == 'exit':
                print('[Action] Exit with code 0')
                break

            elif scan == 'off' or scan == 'torque off' :
                joint_name = input('\t Joint name : ')
                print('[Action] Torque off: {}'.format(joint_name))

                if joint_name == 'all' or joint_name == "left_arm" or joint_name == "right_arm" :
                    motor.set_joint_states([joint_name], False)
                    self.torque_flag = False
                else:
                    joint_id = joint_name.split()
                    try:
                        joint = [ self.joint_id_to_name[int(id)] for id in joint_id ]

                        print('[Action] Torque off: {}'.format(joint))
                        motor.set_joint_states(joint, False)
                        self.torque_flag = False
                    except:
                        print('[Action] Torque invalid input')
                
            elif scan == 'on' or scan == 'torque on' :
                joint_name = input('\t Joint name : ')
                print('[Action] Torque on: {}'.format(joint_name))

                if joint_name == 'all' or joint_name == "left_arm" or joint_name == "right_arm" :
                    motor.set_joint_states([joint_name], True)
                    self.torque_flag = True
                else:
                    joint_id = joint_name.split()
                    try:
                        joint = [ self.joint_id_to_name[int(id)] for id in joint_id ]

                        print('[Action] Torque on: {}'.format(joint))
                        motor.set_joint_states(joint, True)
                        self.torque_flag = True
                    except:
                        print('[Action] Torque invalid input')
           
            elif scan == 's' or scan == 'save':
                if self.torque_flag:
                    motion_name = input('\t Save motion name : ')
                    motion_name = motion_name.split()

                    try:
                        header_motion = motion_name[0]
                        sub_motion    = int(motion_name[1])

                        if header_motion not in self.motion:
                            self.motion[header_motion] = dict()

                        if self.robot_name == "Thormang3_Wolf": 
                            temp_dict = {key:value for key, value in motor.joint_position.items() if key in self.filtered_join}
                            temp_dict['time']     = 0
                            temp_dict['velocity'] = 5 # 5%

                        elif self.robot_name == "Thormang3_Bear": 
                            temp_dict = motor.joint_position
                            temp_dict['time']     = 0
                            temp_dict['velocity'] = 5 # 5%
                            temp_dict['l_arm_finger45_p'] = temp_dict['l_arm_index_p'] = temp_dict['l_arm_middle_p'] = temp_dict['l_arm_thumb_p'] = -180
                            temp_dict['r_arm_finger45_p'] = temp_dict['r_arm_index_p'] = temp_dict['r_arm_middle_p'] = temp_dict['r_arm_thumb_p'] = -180
                        
                        self.motion[header_motion][sub_motion] = temp_dict
                        self.save_motion()
                        print('[Action] Saved motion: {} '.format(motion_name))
                    except:
                        print('[Action] Please input motion with number')
                else:
                    print('[Action] Failed to save motion, Please turn on torque first')

            elif scan == 'p' or scan == 'play':
                self.load_motion() # refresh load
                motion_name = input('\t Play motion name : ')
                print('[Action] Play motion: {}'.format(motion_name))

                motion_name = motion_name.split()
                if len(motion_name) == 2:
                    try:
                        header_motion = motion_name[0]
                        sub_motion    = int(motion_name[1])
                        self.play_sub_motion(header_motion, sub_motion)
                    except:
                        print('[Action] Unknown motion')

                elif len(motion_name) == 1:
                    motion_name = motion_name[0]
                    self.play_motion(motion_name)
                
                else:
                    print('[Action] Unknown motion')

            elif scan == 'r' or scan == 'read':
                print(motor.joint_position)

            elif scan == 'h' or scan == 'help':
                print('[Action] --h (help)')
                print()
                print("\t----------Pioneer Action Editor-----------")
                print("\t-- Save Motion----------------------- [s]")
                print("\t-- Read Joints ---------------------- [r]")
                print("\t-- Torque Off ----------------------- [off]")
                print("\t-- Torque On ------------------------ [on]")
                print("\t-- Play Motion----------------------- [p]")
                print("\t-- Exit Action Editor --------------- [q]")
                print()

            elif scan == 'set':
                self.set_init_config(torque=50)

            else:
                print('[Action] Unknown input')

            self.main_rate.sleep()
        self.kill_threads()

if __name__ == '__main__':
    rospy.init_node('pioneer_action', anonymous=False)
    robot_name = "Thormang3_Wolf"
    # robot_name = "Thormang3_Bear"

    action = Action(robot_name)
    rospy.loginfo("[Action] {} running". format(robot_name))

    action.run()