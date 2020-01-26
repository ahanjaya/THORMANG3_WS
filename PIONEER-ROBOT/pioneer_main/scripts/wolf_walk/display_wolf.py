#!/usr/bin/env python3

import os
import pptk
import rospy
import rospkg
import numpy as np

class Wolf_Display:
    def __init__(self):
        rospy.init_node('pioneer_wolf_display') #, disable_signals=True)
        rospy.loginfo("[Wolf] Pioneer Main Wolf Walk - Running")

        rospack           = rospkg.RosPack()
        self.data_path    = rospack.get_path("pioneer_main") + "/data/wolf_walk"
        
        self.main_rate    = rospy.Rate(10)
        self.visual_ptk1  = None

    def plot_point_cloud(self, label, pcl_data, hardware):
        rospy.loginfo("[Wolf] {} - length pcl : {}".format(label, pcl_data.shape))
        visual_ptk = pptk.viewer(pcl_data[:,:3])

        if hardware == "lidar":
            visual_ptk.attributes(pcl_data[:,-1])
            visual_ptk.set(point_size=0.0025)

        elif hardware == "realsense":
            visual_ptk.set(point_size=0.0025)
            color = pcl_data[:, 3:]
            visual_ptk.attributes(color / 255)
        return visual_ptk

    def close_all(self):
        if self.visual_ptk1 is not None:
            self.visual_ptk1.close()

    def run(self):
        folder_number = 2
        file_number   = 15

        # real_sense
        file_name     = "{0}/{1}/wolf_realsense_pcl-{1}-{2}.npz".format(self.data_path, folder_number, file_number )
        pcl_file      = np.load(file_name)
        point_clouds  = pcl_file['pcl']
        rospy.loginfo("[Wolf] wolf_pcl-{}-{}.npz".format(folder_number, file_number))
        visual_ptk1   = self.plot_point_cloud('wolf_realsense', point_clouds, hardware='realsense') # <-- plot

        # lidar
        file_name     = "{0}/{1}/wolf_lidar_pcl-{1}.npz".format(self.data_path, folder_number)
        pcl_file      = np.load(file_name)
        point_clouds  = pcl_file['pcl']
        rospy.loginfo("[Wolf] wolf_lidar_pcl-{}.npz".format(folder_number))
        visual_ptk2   = self.plot_point_cloud('wolf_lidar', point_clouds, hardware='lidar') # <-- plot

        while not rospy.is_shutdown():
            self.main_rate.sleep()

        visual_ptk1.close()

if __name__ == '__main__':
    wolf = Wolf_Display()
    wolf.run()