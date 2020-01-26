#!/usr/bin/env python3

import pcl
import pptk
import rospy
import rospkg
import ros_numpy
import numpy as np
from time import sleep
from std_msgs.msg import String
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2
from pioneer_motion.motion import Motion

np.set_printoptions(suppress=True)

class Workspace:
    def __init__(self):
        rospy.init_node('pioneer_workspace', anonymous=False)
        rospy.loginfo("[PW] Pioneer Main Workspace- Running")

        rospack         = rospkg.RosPack()
        self.pcl_path = rospack.get_path("pioneer_main") + "/config/thormang3_pcl.npz"
        
        self.motion       = Motion()
        self.scan_offset  = 50
        self.init_head_p  = 30
        self.point_clouds = None

        self.lidar_finish = False
        self.load_pcl     = True
        self.got_data     = False

        ## Subscriber
        rospy.Subscriber('/robotis/sensor/assembled_scan', PointCloud2, self.point_cloud2_callback)
        rospy.Subscriber('/robotis/sensor/move_lidar',     String,      self.lidar_turn_callback)

    def wait_robot(self, msg):
        motion = self.motion
        while motion.status_msg != msg:
            pass # do nothing

    def point_cloud2_callback(self, data):
        pc          = ros_numpy.numpify(data)
        points      = np.zeros((pc.shape[0],4))
        points[:,0] = pc['x']
        points[:,1] = pc['y']
        points[:,2] = pc['z']
        points[:,3] = pc['intensities']
        # print(pc.dtype.names) # ('x', 'y', 'z', 'intensities', 'index')

        # print(len(points))
        self.point_clouds = points

    def lidar_turn_callback(self, msg):
        if msg.data == "end":
            rospy.loginfo("[PW] Scan finished")
            self.lidar_finish = True

    def plot_point_cloud(self, pcl_data, roi=False):
        rospy.loginfo("[PW] Length point cloud : {}".format(pcl_data.shape))
        visual_ptk = pptk.viewer(pcl_data[:,:3])

        if not roi:
            visual_ptk.attributes(pcl_data[:,-1])

        # v.attributes(points[['r', 'g', 'b']] / 255., points['i'])
        # color      = pcl_data.copy()
        # color[:,:] = [255, 0, 0]
        # visual_ptk.attributes(color)
        # visual_ptk.set(point_size=0.001)

    def run(self):
        motion = self.motion

        if not self.load_pcl:
            motion.publisher_(motion.module_control_pub, "head_control_module", latch=True)
            motion.set_head_joint_states(['head_y', 'head_p'], [0, self.init_head_p])
            self.wait_robot("Head movement is finished.")
            rospy.loginfo('[PW] Head Init ...')

            motion.publisher_(motion.move_lidar_pub, "start") # scan full head_p
            # motion.publisher_(motion.move_lidar_range_pub, np.radians(self.scan_offset+1)) # scan head with range

        else:
            data  = np.load(self.pcl_path)
            self.point_clouds = data['pcl']
            self.plot_point_cloud(self.point_clouds)
            self.got_data = True

        while not rospy.is_shutdown():
            if not self.load_pcl:
                if self.lidar_finish:
                    sleep(8)
                    np.savez(self.pcl_path, pcl=self.point_clouds)
                    self.plot_point_cloud(self.point_clouds)
                    self.got_data     = True
                    self.lidar_finish = False

            # if self.got_data:
            #     area_conds    = np.where( (self.point_clouds[:,0] < 0.55) & \
            #                               (self.point_clouds[:,1] < 0.35)  & \
            #                               (self.point_clouds[:,2] < -0.1) & (self.point_clouds[:,2] > -0.3)   ) 
            #     filter_area   = self.point_clouds[area_conds]
            #     self.plot_point_cloud(filter_area)

            #     keyboard_area = np.where( (filter_area[:,3] < 3000) )
            #     keyboard      = filter_area[keyboard_area]
            #     self.plot_point_cloud(keyboard, True)

            #     xmin_keyb_pcl = np.min(keyboard[:,0])
            #     xmax_keyb_pcl = np.max(keyboard[:,0])
            #     ymin_keyb_pcl = np.min(keyboard[:,1])
            #     ymax_keyb_pcl = np.max(keyboard[:,1])
                
            #     rospy.loginfo("PCL Keyboard")
            #     rospy.loginfo("X Min : {} X Max : {}".format(xmin_keyb_pcl, xmax_keyb_pcl))
            #     rospy.loginfo("Y Min : {} Y Max : {}".format(ymin_keyb_pcl, ymax_keyb_pcl))

            #     self.got_data = False

            # pass
            # motion.publisher_(motion.move_lidar_pub, "start") # scan full head_p
            # motion.publisher_(motion.move_lidar_range_pub, np.radians(self.scan_offset+1)) # scan head with range
            # break


        motion.kill_threads()

if __name__ == '__main__':
    ws = Workspace()
    ws.run()