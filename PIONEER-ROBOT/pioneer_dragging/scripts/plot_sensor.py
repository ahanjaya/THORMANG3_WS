#! /usr/bin/env python3

import rospy
import numpy as np
import matplotlib.pyplot as plt
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32
from pioneer_utils.utils import *

class Sensor_Util:
    def __init__(self):
       
        # rqt_plot
        self.imu_roll_pub  = rospy.Publisher('/pioneer/dragging/imu_roll',  Float32,  queue_size=1)
        self.imu_pitch_pub = rospy.Publisher('/pioneer/dragging/imu_pitch', Float32,  queue_size=1)
        self.imu_yaw_pub   = rospy.Publisher('/pioneer/dragging/imu_yaw',   Float32,  queue_size=1)
        
        self.lfoot_pub     = rospy.Publisher('/pioneer/dragging/lfoot',     Float32,  queue_size=1)
        self.rfoot_pub     = rospy.Publisher('/pioneer/dragging/rfoot',     Float32,  queue_size=1)

    def imu_callback(self, msg):
        euler_rad    = quaternion_to_euler( msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w )
        euler_deg    = np.degrees(euler_rad)
        self.imu_ori = { 'roll': euler_deg[0], 'pitch': euler_deg[1], 'yaw' : euler_deg[2] }

        try:
            self.imu_roll_pub.publish(self.imu_ori['roll'])
            self.imu_pitch_pub.publish(self.imu_ori['pitch'])
            self.imu_yaw_pub.publish(self.imu_ori['yaw'])
        except:
            pass

    def run(self):
        rospy.Subscriber('/robotis/sensor/imu', Imu, self.imu_callback)
        rospy.spin()

            # # FT Sensor
            # try:
            #     left_foot  = np.array([ sensor.left_torque['x'],  sensor.left_torque['y'],  sensor.left_torque['z']  ])
            #     right_foot = np.array([ sensor.right_torque['x'], sensor.right_torque['y'], sensor.right_torque['z'] ])
            #     origin     = np.array([0, 0, 0])

            #     euc_lfoot  = np.linalg.norm(left_foot - origin)
            #     euc_rfoot  = np.linalg.norm(right_foot - origin)

            #     self.lfoot_pub.publish(euc_lfoot)
            #     self.rfoot_pub.publish(euc_rfoot)
            
            # except:
            #     pass

if __name__ == "__main__":
    rospy.init_node('pioneer_sensor_utils')
    utiliity = Sensor_Util()
    utiliity.run()