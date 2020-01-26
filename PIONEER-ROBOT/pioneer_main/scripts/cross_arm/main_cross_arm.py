#!/usr/bin/env python3

import os
import sys
import time
import rospy
import rospkg
import threading
import numpy as np
from time import sleep
from pynput import mouse
from std_msgs.msg import String, Bool, Int16
from pioneer_motion.motion import Motion
from pioneer_motion.action import Action
from pioneer_kinematics.kinematics import Kinematics

class Cross_Arm:
    def __init__(self):
        rospy.init_node('pioneer_cross_arm') #, disable_signals=True)
        rospy.loginfo("[CA] Pioneer Main Cross Arm- Running")

        rospack          = rospkg.RosPack()
        self.video_file  = "vlc " + rospack.get_path("pioneer_main") + "/data/cross_arm/robot_dream.mp4"  

        self.kinematics  = Kinematics()
        self.motion      = Motion()
        self.action      = Action("Thormang3_Bear")
        self.main_rate   = rospy.Rate(10)

        self.arm          = None
        self.state        = None
        self.shutdown     = False
        self.object       = False
        self.failed       = False
        self.left_clicked = False

        self.scan_offset  = 50
        self.init_head_p  = 15
        self.volume_value = 80 # 60
        self.fail_counter = 0

        ## Publisher
        self.play_sound_pub = rospy.Publisher('/play_sound_file',   String,  queue_size=10)
        self.set_volume_pub = rospy.Publisher('/set_volume',        Int16,  queue_size=10)
        self.face_pub       = rospy.Publisher('/pioneer/face',      Bool,  queue_size=10)

        # self.shutdown_pub   = rospy.Publisher("/pioneer/shutdown_signal", Bool,    queue_size=10)

        ## Subscriber
        rospy.Subscriber("/pioneer/shutdown_signal",           Bool,   self.shutdown_callback)
        rospy.Subscriber("/pioneer/cross_arm/final_decision",  String, self.final_decision_callback)
        rospy.Subscriber("/pioneer/cross_arm/object",          Bool,   self.object_callback)

    def wait_robot(self, obj, msg):
        while obj.status_msg != msg:
            pass # do nothing

    def final_decision_callback(self, msg):
        self.arm = msg.data

    def object_callback(self, msg):
        self.object = msg.data

    def shutdown_callback(self, msg):
        self.shutdown = msg.data
        rospy.loginfo("[CA] Shutdown time")
        rospy.signal_shutdown('Exit')

    def set_volume(self, volume):
        msg      = Int16()
        msg.data = volume
        self.set_volume_pub.publish(msg)
        rospy.loginfo("[CA] Set PPC volume to : {}".format(msg.data))

    def play_sound(self, name):
        sounds = {  'left_arm'        : "/home/ppc/Music/thormang_bear_mp3/coin_left_arm.mp3", 
                    'right_arm'       : "/home/ppc/Music/thormang_bear_mp3/coin_right_arm.mp3",
                    'starwars'        : "/home/ppc/Music/cross_arm_iros_2019/starwars.mp3",
                    'dream_sound'     : "/home/ppc/Music/cross_arm_iros_2019/dream_sound.mp3",
                    'oh_sorry'        : "/home/ppc/Music/cross_arm_iros_2019/oh_sorry.mp3",
                    'i_wanted'        : "/home/ppc/Music/cross_arm_iros_2019/i_wanted.mp3",
                    'hello_everyone'  : "/home/ppc/Music/cross_arm_iros_2019/hello_everyone.mp3",
                    'hi'              : "/home/ppc/Music/cross_arm_iros_2019/hi.mp3",
                    'welcome'         : "/home/ppc/Music/cross_arm_iros_2019/welcome.mp3",
                    'intro1'          : "/home/ppc/Music/cross_arm_iros_2019/intro1.mp3",
                    'intro2'          : "/home/ppc/Music/cross_arm_iros_2019/intro2.mp3",
                    'intro3'          : "/home/ppc/Music/cross_arm_iros_2019/intro3.mp3",
                    'but'             : "/home/ppc/Music/cross_arm_iros_2019/but.mp3",
                    'dramatic'        : "/home/ppc/Music/cross_arm_iros_2019/dramatic.mp3",
                    'magic_capable'   : "/home/ppc/Music/cross_arm_iros_2019/magic_capable.mp3",
                    'dont_believe'    : "/home/ppc/Music/cross_arm_iros_2019/dont_believe.mp3",
                    'lets_see'        : "/home/ppc/Music/cross_arm_iros_2019/lets_see.mp3",
                    'partner'         : "/home/ppc/Music/cross_arm_iros_2019/partner.mp3",
                    'place_pen'       : "/home/ppc/Music/cross_arm_iros_2019/place_pen.mp3",
                    'pick_pen'        : "/home/ppc/Music/cross_arm_iros_2019/pick_pen.mp3",
                    'matches'         : "/home/ppc/Music/cross_arm_iros_2019/matches.mp3",
                    'basic'           : "/home/ppc/Music/cross_arm_iros_2019/basic.mp3",
                    'next_magic'      : "/home/ppc/Music/cross_arm_iros_2019/next_magic.mp3",
                    'place_crayon'    : "/home/ppc/Music/cross_arm_iros_2019/place_crayon.mp3",
                    'joker'           : "/home/ppc/Music/cross_arm_iros_2019/joker.mp3",
                    'mario'           : "/home/ppc/Music/cross_arm_iros_2019/mario.mp3",
                    'take_crayon'     : "/home/ppc/Music/cross_arm_iros_2019/take_crayon.mp3",
                    'please_applause' : "/home/ppc/Music/cross_arm_iros_2019/please_applause.mp3",
                    'close'           : "/home/ppc/Music/cross_arm_iros_2019/close.mp3",
                    'applause'        : "/home/ppc/Music/cross_arm_iros_2019/applause.mp3",
                    'applause_effect' : "/home/ppc/Music/cross_arm_iros_2019/applause_effect.mp3",
                    'volunteer'       : "/home/ppc/Music/cross_arm_iros_2019/volunteer.mp3",
                    'pickup_coin'     : "/home/ppc/Music/cross_arm_iros_2019/pickup_coin.mp3",
                    'ready?'          : "/home/ppc/Music/cross_arm_iros_2019/ready?.mp3",
                    'step1'           : "/home/ppc/Music/cross_arm_iros_2019/step1.mp3",
                    'step2'           : "/home/ppc/Music/cross_arm_iros_2019/step2.mp3",
                    'look_my_eye'     : "/home/ppc/Music/cross_arm_iros_2019/look_my_eye.mp3",
                    'i_know'          : "/home/ppc/Music/cross_arm_iros_2019/i_know.mp3",
                    'cross_arm'       : "/home/ppc/Music/cross_arm_iros_2019/cross_arm.mp3",
                    'begin'           : "/home/ppc/Music/cross_arm_iros_2019/begin.mp3",
                    'open_left_arm'   : "/home/ppc/Music/cross_arm_iros_2019/open_left_arm.mp3",
                    'open_right_arm'  : "/home/ppc/Music/cross_arm_iros_2019/open_right_arm.mp3",
                    'its_magic'       : "/home/ppc/Music/cross_arm_iros_2019/its_magic.mp3",
                    'proud'           : "/home/ppc/Music/cross_arm_iros_2019/proud.mp3",
                    'sad_titanic'     : "/home/ppc/Music/cross_arm_iros_2019/sad_titanic.mp3",
                    'not_working'     : "/home/ppc/Music/cross_arm_iros_2019/not_working.mp3",
                    'fail1'           : "/home/ppc/Music/cross_arm_iros_2019/fail1.mp3",
                    'fail2'           : "/home/ppc/Music/cross_arm_iros_2019/fail2.mp3",
                    'cheated'         : "/home/ppc/Music/cross_arm_iros_2019/cheated.mp3",
                    'retry'           : "/home/ppc/Music/cross_arm_iros_2019/retry.mp3",
                    'thank_you'       : "/home/ppc/Music/cross_arm_iros_2019/thank_you.mp3",
                    'joining_me'      : "/home/ppc/Music/cross_arm_iros_2019/joining_me.mp3",
                    'thankful'        : "/home/ppc/Music/cross_arm_iros_2019/thankful.mp3",
                    'closing'         : "/home/ppc/Music/cross_arm_iros_2019/closing.mp3"
                } 

        sound      = String()
        sound.data = sounds[name]
        self.play_sound_pub.publish(sound)

    def wait_action(self):
        sleep(0.5)
        while not self.action.finish_action:
            pass

    def on_click(self, x, y, button, pressed):
        if pressed:
            print ("Left mouse clicked")
            return False

    def wait_trigger(self):
        with mouse.Listener(on_click=self.on_click) as listener:
            listener.join()

    def play_video(self, file):
        os.system(file)

    def play_video_thread(self, file):
        thread1 = threading.Thread(target = self.play_video, args=(file, ))
        thread1.start()

    def run(self):
        kinematics = self.kinematics
        motion     = self.motion
        action     = self.action

        action.motor.publisher_(action.motor.module_control_pub, "none", latch=True)
        action.set_init_config(torque=50)
        sleep(1)

        action.play_motion("standby")
        self.wait_action()
        sleep(0.1)
        self.set_volume(self.volume_value)

        run_robot = input("\nStart Magic Show (y/n)? ")
        if run_robot == 'y':
            # self.state = 'thinking'
            # self.state = 'matches'
            # self.state = 'crayon'
            # self.state = 'volunteer'
            self.state = 'instructions'
        else:
            self.shutdown = True

        while not rospy.is_shutdown():
            if self.shutdown:
                rospy.loginfo('[CA] Main Cross Arm Exit ...')
                break

            if self.state == 'thinking':
                rospy.loginfo('[CA] Robot State : {}'.format(self.state))

                action.play_motion_thread("thinking")
                sleep(2)       

                self.play_sound('dream_sound') 
                self.play_video_thread(self.video_file)
                sleep(76)
                self.wait_action()
                self.state = 'surprise'

            elif self.state == 'surprise':
                rospy.loginfo('[CA] Robot State : {}'.format(self.state))

                action.play_motion_thread("surprise")
                sleep(2)
                self.play_sound('oh_sorry')
                sleep(4)
                action.play_motion("standby")
                self.wait_action()
                self.state = 'hello'

            elif self.state == 'hello':
                rospy.loginfo('[CA] Robot State : {}'.format(self.state))

                action.play_motion_thread("hello")
                sleep(2.5)
                self.play_sound('hello_everyone')
                self.wait_action()
                sleep(1)
                action.play_motion_thread("welcome")
                sleep(1)
                self.play_sound('welcome')
                sleep(3)
                self.wait_action()
                self.state = 'exciting'

            elif self.state == 'exciting':
                rospy.loginfo('[CA] Robot State : {}'.format(self.state))

                action.play_motion_thread("exciting") 
                self.play_sound('but') #line3
                sleep(1)
                self.wait_action()
                self.play_sound('dramatic')
                sleep(6)
                self.state = 'capable'

            elif self.state == 'capable':
                rospy.loginfo('[CA] Robot State : {}'.format(self.state))

                action.play_motion_thread("fisting")
                self.play_sound('dont_believe') 
                sleep(3)
                self.wait_action()
                action.play_motion_thread("standby")
                self.play_sound('lets_see')
                self.wait_action()
                sleep(2)
                self.state = 'matches'

            elif self.state == 'matches':
                rospy.loginfo('[CA] Robot State : {}'.format(self.state))

                self.play_sound('partner') 
                sleep(3)
                action.play_motion("m1")
                self.wait_action()
                sleep(1)
                self.play_sound('place_pen')
                rospy.loginfo('Please click mouse, if pen is installed')
                self.wait_trigger()

                self.play_sound('matches') # backsound
                sleep(2)
                action.play_motion("m2")
                self.wait_action()
                sleep(1)
                action.play_motion("m3")
                self.wait_action()
                sleep(4)
                self.play_sound('basic') #line 5
                sleep(3)
                action.play_motion("m4")
                self.wait_action()
                sleep(1)
                self.play_sound('pick_pen')
                rospy.loginfo('Please click mouse, if pen is removed')
                self.wait_trigger()

                action.play_motion("standby")
                self.wait_action()
                self.state = 'crayon'

            elif self.state == 'crayon':
                rospy.loginfo('[CA] Robot State : {}'.format(self.state))
                
                self.play_sound('next_magic') 
                sleep(3)
                action.play_motion("c1")
                self.wait_action()
                sleep(1)
                self.play_sound('place_crayon') # change line 7
                rospy.loginfo('Please click mouse, if crayon is placed')
                self.wait_trigger()
                
                action.play_motion("c2")
                self.wait_action()
                sleep(3)
                self.play_sound('joker') 
                sleep(2)
                action.play_motion("c3")
                self.wait_action()
                action.play_motion("c4")
                self.wait_action()
                sleep(6)
                self.play_sound('please_applause') # change line 9
                sleep(5)
                self.play_sound('applause_effect')
                sleep(1)

                rospy.loginfo('Please click mouse, if crayon is taken')
                self.wait_trigger()
                action.play_motion("standby")
                self.wait_action()
                self.state = 'volunteer'

            elif self.state == 'volunteer':
                rospy.loginfo('[CA] Robot State : {}'.format(self.state))
                action.play_motion_thread("volunteer")

                sleep(2)
                self.play_sound('volunteer')  # change line 11
                self.wait_action()
                rospy.loginfo('Please click mouse, if got volunteer')
                self.wait_trigger()

                action.play_motion("standby")
                self.wait_action()
                self.state = 'pickup_coin'

            elif self.state == 'pickup_coin':
                rospy.loginfo('[CA] Robot State : {}'.format(self.state))
                action.play_motion_thread("pickup")

                sleep(2)
                self.play_sound('pickup_coin') # change line 13
                self.wait_action()
                rospy.loginfo('Please click mouse, if coin is picked up')
                self.wait_trigger()

                action.play_motion("standby")
                self.wait_action()
                self.play_sound('ready?') 
                rospy.loginfo('Please click mouse, if volunteer is ready')
                self.wait_trigger()
                self.state = 'instructions'

            elif self.state == 'instructions':
                rospy.loginfo('[CA] Robot State : {}'.format(self.state))
                self.face_pub.publish(True)

                self.play_sound('step1') 
                sleep(10)
                self.action.play_motion_thread("arm_front")
                self.play_sound('step2') # change line 15
                self.wait_action()
                sleep(5)
                self.state = 'look_my_eye'

            elif self.state == 'look_my_eye':
                rospy.loginfo('[CA] Robot State : {}'.format(self.state))
                action.play_motion_thread("look")
                sleep(2)
                self.play_sound('look_my_eye')
                sleep(5)
                self.play_sound('i_know') # change line 17
                sleep(5)               
                self.wait_action()
                action.play_motion("standby")
                self.wait_action()
                sleep(1)
                self.state = 'cross_arm'

            elif self.state == 'cross_arm':
                rospy.loginfo('[CA] Robot State : {}'.format(self.state))

                action.play_motion_thread("cross_arm")
                sleep(1)
                self.play_sound('cross_arm') # change line 19
                sleep(0.2)
                self.wait_action()
                sleep(2)
                action.play_motion("standby")
                self.wait_action()
                self.state = 'head_scanning'
            
            elif self.state == 'head_scanning':
                rospy.loginfo('[CA] Robot State : {}'.format(self.state))
                action.set_init_config(0, 0) # set torque & velocity to default 
                self.face_pub.publish(False)

                motion.publisher_(motion.module_control_pub, "head_control_module", latch=True)
                self.play_sound('begin') # change line 21
                motion.publisher_(motion.move_lidar_pub, "start") # scan full head_p
                sleep(1)
                motion.publisher_(motion.override_orig_pos_lidar_pub,  self.init_head_p) # overide original lidar pose

                self.wait_robot(motion, "Finish head joint in order to make pointcloud")
                self.state = 'standby_pointing'

            elif self.state == 'standby_pointing':
                rospy.loginfo('[CA] Robot State : {}'.format(self.state))

                sleep(2)
                while self.arm == None:
                    pass
                kinematics.publisher_(kinematics.module_control_pub, "manipulation_module", latch=True)
                # self.play_sound(self.arm)

                if self.arm == "left_arm":
                    kinematics.set_joint_pos(['l_arm_thumb_p', 'l_arm_middle_p', 'l_arm_finger45_p'], [-60, 180, 180])
                    kinematics.set_kinematics_pose("left_arm", 2.0, **{ 'x': 0.4, 'y': 0.1, 'z': 0.80, 'roll': 0.00, 'pitch': -10.00, 'yaw': -5.00 })
                    self.play_sound('open_left_arm')
                    self.wait_robot(kinematics, "End Left Arm Trajectory")
                
                elif self.arm == "right_arm":
                    kinematics.set_joint_pos(['r_arm_thumb_p', 'r_arm_middle_p', 'r_arm_finger45_p'], [-60, 180, 180])
                    kinematics.set_kinematics_pose("right_arm", 2.0, **{ 'x': 0.4, 'y': -0.1, 'z': 0.80, 'roll': 0.00, 'pitch': -10.00, 'yaw': 5.00 })
                    self.play_sound('open_right_arm')
                    self.wait_robot(kinematics, "End Right Arm Trajectory")

                self.arm = None
                self.state = 'check'

            elif self.state == 'check': # vision
                rospy.loginfo('[CA] Robot State : {}'.format(self.state))

                start_time = time.time()
                while not self.object:
                    diff_time = time.time() - start_time
                    if diff_time > 5:
                        break

                if self.object:
                    action.motor.publisher_(action.motor.module_control_pub, "none", latch=True)
                    self.state = 'success'
                else:
                    if self.fail_counter >= 2: # 2
                        action.motor.publisher_(action.motor.module_control_pub, "none", latch=True)
                        self.state = 'sad'
                    else:
                        self.play_sound('fail1') # change line 23
                        sleep(2)
                        self.fail_counter += 1
                        self.state = 'fail'

            elif self.state == 'fail':
                rospy.loginfo('[CA] Robot State : {}'.format(self.state))

                start_time = time.time()
                while not self.object:
                    diff_time = time.time() - start_time
                    if diff_time > 5:
                        break

                if self.object:
                    self.play_sound('fail2')
                    sleep(7)
                else:
                    self.play_sound('cheated') # change line 25
                    sleep(5)

                action.motor.publisher_(action.motor.module_control_pub, "none", latch=True)
                action.play_motion("standby")
                self.wait_action()
                self.play_sound('retry')
                sleep(5)
                self.state = 'instructions'

            elif self.state == 'success':
                rospy.loginfo('[CA] Robot State : {}'.format(self.state))

                self.play_sound('its_magic')
                action.play_motion("happy")
                self.wait_action()
                action.play_motion("standby")
                self.wait_action()
                sleep(5)
                self.play_sound('proud')
                sleep(8)
                # self.state = 'thanks_volunteer'
                self.state = None

            elif self.state == 'sad':
                rospy.loginfo('[CA] Robot State : {}'.format(self.state))

                self.play_sound('sad_titanic')
                action.play_motion("sad")
                self.wait_action()
                sleep(10)
                action.play_motion("standby")
                self.wait_action()
                self.play_sound('not_working')
                sleep(10)
                self.state = 'thanks_volunteer'

            elif self.state == 'thanks_volunteer':
                rospy.loginfo('[CA] Robot State : {}'.format(self.state))

                self.play_sound('thankful')
                sleep(3)
                action.play_motion("joining")
                self.wait_action()
                self.play_sound('joining_me')
                sleep(5)
                action.play_motion("standby")
                self.wait_action()

                self.state = 'closing'

            elif self.state == 'closing':
                rospy.loginfo('[CA] Robot State : {}'.format(self.state))

                self.play_sound('thank_you')
                sleep(5)

                action.play_motion_thread("thank")
                sleep(4)
                self.play_sound('closing')
                self.wait_action()
                sleep(8)
                action.play_motion("standby")
                self.wait_action()
                self.state = None

            elif self.state == 'trial':
                rospy.loginfo('[CA] Trial Head Scanning')
                action.set_init_config(0, 0) # set torque & velocity to default 
                motion.publisher_(motion.module_control_pub, "head_control_module", latch=True)

                start = input("\nStart Head Scanning (y/n)? ")

                if start == 'y':
                    self.play_sound('begin')
                    motion.publisher_(motion.move_lidar_pub, "start") # scan full head_p
                    sleep(1)
                    motion.publisher_(motion.override_original_pos_lidar_pub,  self.init_head_p) # overide original lidar pose
                    self.wait_robot(motion, "Finish head joint in order to make pointcloud")
                
                else:
                    rospy.loginfo('[CA] Exit trial')
                    break

            else:
                if self.state != None:
                    rospy.loginfo('[CA] Invalid Robot State : {}'.format(self.state))

            self.main_rate.sleep()

if __name__ == '__main__':
    ca = Cross_Arm()
    ca.run()