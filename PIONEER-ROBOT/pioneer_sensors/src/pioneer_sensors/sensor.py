#!/usr/bin/env python3

import time
import rospy
import ctypes
import struct
import ros_numpy
import threading
import numpy as np
import sensor_msgs.point_cloud2 as pc2

from pioneer_utils.utils import *
from geometry_msgs.msg import WrenchStamped
from thormang3_imu_3dm_gx4.msg import FilterOutput
from sensor_msgs.msg import Imu, LaserScan, PointCloud2

class Sensor:
    def __init__(self, robot_name):
        # rospy.init_node('pionee_sensor', anonymous=False)
        self.robot_name        = robot_name

        # IMU
        self.imu_filter        = True
        self.imu_ori           = {}
        self.imu_ori_cov       = None
        self.imu_ang_vel       = None
        self.imu_ang_vel_cov   = None
        self.imu_lin_accel     = None
        self.imu_lin_accel_cov = None
        self.imu_bias          = None
        self.imu_bias_cov      = None

        # Lidar
        self.lidar_filter      = False
        self.lidar_ranges      = None

        # Foot Sensor
        self.left_torque       = None
        self.right_torque      = None

        # Point Cloud
        self.lidar_pcl         = None
        self.real_sense_pcl    = None

        self.mutex             = threading.Lock()
        self.read_sensor()
        
    def thread_read_IMU(self):
        if self.robot_name  == "Thormang3_Wolf" : # Thormang3 Full size
            if self.imu_filter:
                rospy.Subscriber('/robotis/sensor/imu/filter', FilterOutput, self.imu_filter_callback)
                rospy.spin()
            else:
                rospy.Subscriber('/robotis/sensor/imu/imu', Imu, self.imu_callback)
                rospy.spin()
        elif self.robot_name == "Simulation":
            rospy.Subscriber('/robotis/sensor/imu', Imu, self.imu_callback)
            rospy.spin()

    def thread_read_Lidar(self):
        if self.lidar_filter:
            rospy.Subscriber('/robotis/sensor/scan_filtered', LaserScan, self.lidar_filter_callback)
            rospy.spin()
        else:
            rospy.Subscriber('/robotis/sensor/scan', LaserScan, self.lidar_callback)    
            rospy.spin()

    def thread_read_FTSensor(self):
        rospy.Subscriber('/robotis/sensor/ft_left_foot/scaled',  WrenchStamped, self.left_foot_callback)
        rospy.Subscriber('/robotis/sensor/ft_right_foot/scaled', WrenchStamped, self.right_foot_callback)
        rospy.spin()

    def thread_read_pcl(self):
        rospy.Subscriber('/robotis/sensor/assembled_scan',     PointCloud2,  self.lidar_pcl2_callback)
        rospy.Subscriber('/realsense/depth_registered/points', PointCloud2,  self.realsense_pcl2_callback)
        rospy.spin()

    def read_sensor(self):
        if self.robot_name  == "Thormang3_Wolf" : # Thormang3 Full size
            thread1 = threading.Thread(target = self.thread_read_IMU,     ) 
            thread2 = threading.Thread(target = self.thread_read_FTSensor )
            thread3 = threading.Thread(target = self.thread_read_pcl) 
            
            thread1.start()
            thread2.start()
            thread3.start()
        elif self.robot_name == "Simulation":
            thread1 = threading.Thread(target = self.thread_read_IMU,     ) 
            thread1.start()


        # threadn = threading.Thread(target = self.thread_read_Lidar ) 
        # threadn.start()

    def imu_callback(self, msg):
        self.mutex.acquire()
        euler_rad = quaternion_to_euler( msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w )
        euler_deg = np.degrees(euler_rad)

        self.imu_ori           = { 'roll': euler_deg[0], 'pitch': euler_deg[1], 'yaw' : euler_deg[2] }
        self.imu_ori_cov       = msg.orientation_covariance
        self.imu_ang_vel       = msg.angular_velocity
        self.imu_ang_vel_cov   = msg.angular_velocity_covariance
        self.imu_lin_accel     = msg.linear_acceleration
        self.imu_lin_accel_cov = msg.linear_acceleration_covariance
        # rospy.loginfo('[Sensor] IMU: {}'.format(self.imu_ori))
        # rospy.loginfo('[Sensor] IMU: {}'.format(self.imu_ang_vel))
        self.mutex.release()

    def imu_filter_callback(self, msg):
        self.mutex.acquire()
        euler_rad = quaternion_to_euler( msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w )
        euler_deg = np.degrees(euler_rad)

        self.imu_ori       = { 'roll': euler_deg[0], 'pitch': euler_deg[1], 'yaw' : euler_deg[2] }
        self.imu_ori_cov   = msg.orientation_covariance
        self.imu_bias      = msg.bias
        self.imu_bias_cov  = msg.bias_covariance # **Critical point to consider**
        # rospy.loginfo('[Sensor] IMU ori:          {}'.format(self.imu_ori))
        # rospy.loginfo('[Sensor] IMU ori_cov:      {}'.format(self.imu_ori_cov))
        # rospy.loginfo('[Sensor] IMU imu_bias:     {}'.format(self.imu_bias))
        # rospy.loginfo('[Sensor] IMU imu_bias_cov: {}'.format(self.imu_bias_cov))
        self.mutex.release()

    def lidar_callback(self, msg):
        self.mutex.acquire()
        self.lidar_ranges  = msg.ranges
        self.mutex.release()

    def lidar_filter_callback(self, msg):
        self.mutex.acquire()
        self.lidar_ranges  = msg.ranges
        # resolution = (msg.angle_max - msg.angle_min) / len(msg.ranges)
        # print("Angle[rad] reading resolution:", resolution)
        self.mutex.release()

    def left_foot_callback(self, msg):
        self.mutex.acquire()
        self.left_torque = { 'x': msg.wrench.torque.x, 'y': msg.wrench.torque.y, 'z': msg.wrench.torque.z}
        # rospy.loginfo('[Sensor] Left foot: {}'.format(self.left_torque))
        self.mutex.release()
    
    def right_foot_callback(self, msg):
        self.mutex.acquire()
        self.right_torque = { 'x': msg.wrench.torque.x, 'y': msg.wrench.torque.y, 'z': msg.wrench.torque.z}
        # rospy.loginfo('[Sensor] Right foot: {}'.format(self.right_torque))
        self.mutex.release()

    def lidar_pcl2_callback(self, data):
        self.mutex.acquire()
        pc          = ros_numpy.numpify(data)
        points      = np.zeros((pc.shape[0],4))
        points[:,0] = pc['x']
        points[:,1] = pc['y']
        points[:,2] = pc['z']
        points[:,3] = pc['intensities']
        # print(pc.dtype.names) # ('x', 'y', 'z', 'intensities', 'index')
        self.lidar_pcl = points
        self.mutex.release()

    def realsense_pcl2_callback(self, data):
        self.mutex.acquire()
        try:
            cloud_points  = list(pc2.read_points(data, skip_nans=True, field_names = ("x", "y", "z", "rgb")))
            cloud_points  = np.array(cloud_points)
            points        = cloud_points[:,:3]       # get x,y,z
            points        = points.dot(rotate_X(90))
            points        = points.dot(rotate_Z(90))

            float_rgb     = cloud_points[:,-1]       # get latest column
            color         = []
            
            for data in float_rgb:
                s    = struct.pack('>f', data)
                i    = struct.unpack('>l', s)[0]
                pack = ctypes.c_uint32(i).value
                r    = int((pack & 0x00FF0000) >> 16)
                g    = int((pack & 0x0000FF00) >> 8)
                b    = int((pack & 0x000000FF))
                color.append([r,g,b])
            
            color               = np.array(color)
            self.real_sense_pcl = np.c_[ points, color ]
            # rospy.loginfo('[Sensor] Real sense pcl : {}'.format(self.real_sense_pcl.shape))
        except:
            pass
        self.mutex.release()