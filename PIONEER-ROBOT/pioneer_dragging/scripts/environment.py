#! /usr/bin/env python3

import rospy
import threading
import numpy as np
from time import sleep

# Service
from std_srvs.srv import Empty

# Message
from std_msgs.msg import Float32
from sensor_msgs.msg import Imu

# Pioneer
from pioneer_utils.utils import *
from pioneer_walking.walking import Walking

class Env:
    def __init__(self):
        rospy.wait_for_service('/gazebo/reset_world')
        self.reset_world = rospy.ServiceProxy('/gazebo/reset_world', Empty)
        self.walking     = Walking()

        self.main_rate   = rospy.Rate(60)
        self.fall        = False
        self.fall_angle  = 15

        # rqt_plot
        self.imu_roll_pub  = rospy.Publisher('/pioneer/dragging/imu_roll',  Float32,  queue_size=1)
        self.imu_pitch_pub = rospy.Publisher('/pioneer/dragging/imu_pitch', Float32,  queue_size=1)
        self.imu_yaw_pub   = rospy.Publisher('/pioneer/dragging/imu_yaw',   Float32,  queue_size=1)

        # threading
        thread1    = threading.Thread(target = self.thread_check_robot, ) 
        thread1.start()
        self.mutex = threading.Lock()

    def thread_check_robot(self):
        rospy.Subscriber('/robotis/sensor/imu', Imu, self.imu_callback)
        rospy.spin()

    def imu_callback(self, msg):
        self.mutex.acquire()
        euler_rad    = quaternion_to_euler( msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w )
        euler_deg    = np.degrees(euler_rad)
        self.imu_ori = { 'roll': euler_deg[0], 'pitch': euler_deg[1], 'yaw' : euler_deg[2] }

        try:
            if np.absolute(self.imu_ori['pitch']) > self.fall_angle and not self.fall:
                rospy.logwarn('[Env] Robot FALLING - angle: {}'.format(self.imu_ori['pitch']))
                self.fall = True

            self.imu_roll_pub.publish(self.imu_ori['roll'])
            self.imu_pitch_pub.publish(self.imu_ori['pitch'])
            self.imu_yaw_pub.publish(self.imu_ori['yaw'])
        
        except:
            pass
        self.mutex.release()

    def wait_robot(self, obj, msg):
        while obj.status_msg != msg:
            pass # do nothing
        # rospy.loginfo('[Env] Robot: {}'.format(msg))

    def initial(self):
        walking = self.walking

        # set init pose
        walking.publisher_(walking.walking_pub, "ini_pose", latch=True)
        self.wait_robot(walking, "Finish Init Pose")

        # set walking mode
        walking.publisher_(walking.walking_pub, "set_mode")
        self.wait_robot(walking, "Walking_Module_is_enabled")

        # turn on balance
        walking.publisher_(walking.walking_pub, "balance_on")
        self.wait_robot(walking, "Balance_Param_Setting_Finished")
        self.wait_robot(walking, "Joint_FeedBack_Gain_Update_Finished")

        return

    def reset(self):
        walking = self.walking

        # reset environment
        walking.publisher_(walking.walking_pub, "stop")
        self.wait_robot(walking, "Walking_Finished")

        # wait until robot balance
        self.reset_world()
        self.fall = False

        # wait balance
        sleep(0.5)

        # start walking
        walking.walk_command(command="backward", step_num=10, step_time=1.5,\
                             step_length=0.1, side_step_length=0.05, step_angle_deg=5)

        return None

    def step(self, action):
        walking = self.walking

        new_state = None
        reward    = None
        done      = False
        info      = None

        if self.fall or \ 
            walking.status_msg == "Walking_Finished":

            done = True

        return new_state, reward, done, info 

    def close(self):
        walking = self.walking

        # reset environment
        walking.publisher_(walking.walking_pub, "stop")
        self.wait_robot(walking, "Walking_Finished")

        # wait until robot balance
        self.reset_world()
        self.fall = False