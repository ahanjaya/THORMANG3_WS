#!/usr/bin/env python3

import os
import sys
import time
import rospy
import threading
import numpy as np
from time import sleep
from pynput import mouse
from pocketsphinx import LiveSpeech
from std_msgs.msg import String, Bool, Int16
from pioneer_motion.action import Action

class Mind_No:
    def __init__(self):
        rospy.init_node('pioneer_mind_no')
        rospy.loginfo("[MN] Pioneer Main Mind No- Running")

        self.action      = Action()
        self.main_rate   = rospy.Rate(10)

        self.state        = None
        self.shutdown     = False
        self.left_clicked = False
        self.thread_rate  = rospy.Rate(10)

        self.volume_value = 80 # 60

        ## Publisher
        self.play_sound_pub = rospy.Publisher('/play_sound_file',         String,  queue_size=10)
        self.set_volume_pub = rospy.Publisher('/set_volume',              Int16,  queue_size=10)
        # self.shutdown_pub   = rospy.Publisher("/pioneer/shutdown_signal", Bool,    queue_size=10)

        ## Subscriber
        rospy.Subscriber("/pioneer/shutdown_signal",           Bool,   self.shutdown_callback)

    def wait_robot(self, obj, msg):
        while obj.status_msg != msg:
            pass # do nothing

    def shutdown_callback(self, msg):
        self.shutdown = msg.data
        rospy.loginfo("[MN] Shutdown time")
        rospy.signal_shutdown('Exit')

    def set_volume(self, volume):
        msg      = Int16()
        msg.data = volume
        self.set_volume_pub.publish(msg)
        rospy.loginfo("[MN] Set PPC volume to : {}".format(msg.data))

    def play_sound(self, name):
        sounds = {  '1st' : "/home/ppc/Music/mind_no_iros_2019/1st.mp3", 
                    '2nd' : "/home/ppc/Music/mind_no_iros_2019/2nd.mp3", 
                    '3rd' : "/home/ppc/Music/mind_no_iros_2019/3rd.mp3",
                    '4th' : "/home/ppc/Music/mind_no_iros_2019/4th.mp3",
                    '5th' : "/home/ppc/Music/mind_no_iros_2019/5th.mp3",
                    '6th' : "/home/ppc/Music/mind_no_iros_2019/6th.mp3",
                    '7th' : "/home/ppc/Music/mind_no_iros_2019/7th.mp3"
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

    def on_click_left(self, x, y, button, pressed):
        if pressed:
            self.left_clicked = True
            print ("Left thread mouse clicked")
            return False

    def wait_trigger(self):
        with mouse.Listener(on_click=self.on_click) as listener:
            listener.join()

    def thread_mouse_click(self):
        with mouse.Listener(on_click=self.on_click_left) as listener:
            listener.join()

    def no_detection(self):
        start_time = time.time()
        print('start time:', start_time)

        while True:
            diff_time = time.time() - start_time
            print('diff:', diff_time)

            if diff_time > 5:
                rospy.logwarn('[MN] Timeout for no')
                break

            if self.left_clicked:
                break

            self.thread_rate.sleep()

    def run(self):
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
            self.state = 'start'
            # self.state = 'asking'
        else:
            self.shutdown = True

        while not rospy.is_shutdown():
            if self.shutdown:
                rospy.loginfo('[MN] Mind No Exit ...')
                break

            if self.state == 'start':
                rospy.loginfo('[MN] Robot State : {}'.format(self.state))

                self.play_sound('1st')
                sleep(8)
                action.play_motion_thread("take_phone")
                sleep(0.5)
                self.play_sound('2nd')
                self.wait_action()
                self.state = 'writing'

            elif self.state == 'writing':
                rospy.loginfo('[MN] Robot State : {}'.format(self.state))

                rospy.loginfo('Please click mouse, if phone in hand')
                self.wait_trigger()
                action.play_motion_thread("write_phone")
                sleep(10)
                self.play_sound('3rd')
                self.wait_action()
                self.state = 'asking'

            elif self.state == 'asking':
                rospy.loginfo('[MN] Robot State : {}'.format(self.state))

                thread1 = threading.Thread(target = self.thread_mouse_click)
                thread1.start()

                self.play_sound('4th')
                sleep(5)
                print('4th sound')
                self.no_detection()

                if self.left_clicked:
                    self.state = 'show'
                else:
                    self.play_sound('5th')
                    sleep(5)
                    print('5th sound')
                    self.no_detection()
                    
                    if self.left_clicked:
                        self.state = 'show'
                    else:
                        self.play_sound('6th')
                        sleep(5)
                        print('6th sound')
                        self.no_detection()

                        if self.left_clicked:
                            self.state = 'show'
                        else:
                            self.play_sound('7th')
                            sleep(4)
                            print('7th sound')
                            self.no_detection()

                            if self.left_clicked:
                                self.state = 'show'

            elif self.state == 'show':
                rospy.loginfo('[MN] Robot State : {}'.format(self.state))

                action.play_motion("show_phone")
                self.wait_action()
                self.state = None

            else:
                if self.state != None:
                    rospy.loginfo('[MN] Invalid Robot State : {}'.format(self.state))

            self.main_rate.sleep()

if __name__ == '__main__':
    mn = Mind_No()
    mn.run()