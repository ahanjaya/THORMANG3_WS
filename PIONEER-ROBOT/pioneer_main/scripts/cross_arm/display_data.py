#!/usr/bin/env python3

import os
import pcl
import pptk
import rospy
import rospkg
import ros_numpy
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from pyntcloud import PyntCloud
from mpl_toolkits.mplot3d import Axes3D

class Display:
    def __init__(self):
        rospy.init_node('pioneer_cross_Display')
        rospy.loginfo("[Display] Pioneer Display Data Cross Arm- Running")

        rospack           = rospkg.RosPack()
        # self.pcl_raw_path = rospack.get_path("pioneer_main") + "/data/cross_arm/raw_pcl/"
        # self.pcl_cam_path = rospack.get_path("pioneer_main") + "/data/cross_arm/cam/"
        self.pcl_raw_path = rospack.get_path("pioneer_main") + "/data/cross_arm/history/raw_pcl/"
        self.pcl_cam_path = rospack.get_path("pioneer_main") + "/data/cross_arm/history/cam/"
        self.point_clouds = None
        self.visual_ptk1  = None
        self.visual_ptk2  = None

        self.show_plt     = True
        self.debug        = True
        self.aug_theta    = np.arange(-45, 50, 5)
        self.aug_theta    = np.delete(self.aug_theta, np.argwhere(self.aug_theta==0))
        self.aug_dist     = np.arange(-1, 2, 1)
        # self.aug_dist     = np.delete(self.aug_dist, np.argwhere(self.aug_dist==0))

    def plot_point_cloud(self, label, pcl_data, big_point=False, color=True):
        rospy.loginfo("[Display] {} - length pcl : {}".format(label, pcl_data.shape))
        visual_ptk = pptk.viewer(pcl_data[:,:3])

        if color:
            visual_ptk.attributes(pcl_data[:,-1])
        if big_point:
            # visual_ptk.set(point_size=0.0025)
            visual_ptk.set(point_size=0.1)

        return visual_ptk

    def plot_2d(self, axes_, x_, y_, legend_=None, xlabel_="", ylabel_="", title_=""):
        axes_.plot(x_, y_, 'o-', label=legend_)
        axes_.set_xlabel(xlabel_)
        axes_.set_ylabel(ylabel_)
        axes_.set_title(title_)
        axes_.grid()

    def plot_pcl(self, ax, x, y, z, title=""):
        ax.scatter(x, y, z, c='green')
        # ax.scatter(x, y, z)
        ax.set_xlabel('x')
        ax.set_ylabel('y')
        ax.set_zlabel('z')
        ax.set_xlim(0, 32)
        ax.set_ylim(0, 32)
        ax.set_zlim(0, 32)
        ax.set_title(title)
        ax.view_init(azim=225)        

    def plot_voxel(self, ax, voxel, title=""):
        # ax.voxels(voxel)
        # filled = np.ones(voxel.shape)
        ax.voxels(voxel, facecolors='green', edgecolor='k')

        ax.set_xlabel('x')
        ax.set_ylabel('y')
        ax.set_zlabel('z')
        ax.set_xlim(0, 32)
        ax.set_ylim(0, 32)
        ax.set_zlim(0, 32)
        ax.set_title(title) 
        ax.view_init(azim=225)

    def filter_raw_data(self, data):
        # Sorting point cloud
        data   = data[np.argsort(data[:,0])] # X axis
        data_x = data[:,0]

        # square scaning
        x_min = np.round( np.min(data_x), 1) - 0.1
        x_max = np.round( np.max(data_x), 1) + 0.1
        x_step_size = 0.025 #0.01
        y_step      = np.arange(x_min, x_max, x_step_size)
        z_list      = []
        z_prev      = None

        for y in range(len(y_step)):
            y_ll  = y_step[y]
            y_ul  = y_step[y] + x_step_size
            layer = data[np.where( (data_x >= y_ll) & (data_x < y_ul) )]

            if layer.size != 0:
                zmax = np.max(layer[:,2])
                z_list.append(zmax)
                if z_prev != None:
                    diff = zmax-z_prev
                    if diff <= -0.5:
                        y_lim = y_ll
                        break
                z_prev = zmax

        # if self.show_plt:
        #     _, axes2D = plt.subplots(nrows=1, ncols=1)
        #     self.plot_2d(axes2D, y_step[:len(z_list)], z_list, legend_=None,\
        #         xlabel_="y-layer", ylabel_="z-height", title_="Filtering Human Body on Point Cloud")
        
        # first filter, by z layer
        human_body = data[np.where( (data_x >= x_min) & (data_x < y_lim) )]

        # second filter by euclidean distance
        x_ref      = np.min(human_body[:,0])
        y_ref      = (np.min(human_body[:,1]) + np.max(human_body[:,1])) / 2
        z_ref      = np.max(human_body[:,2])
        ref_point  = np.array([ [x_ref, y_ref, z_ref] ])
        # rospy.loginfo('[CAL] Ref. Point: {}'.format(ref_point))

        eucl_dist  = np.linalg.norm(ref_point - human_body[:,:3], axis=1)
        human_body = human_body[ np.where( (eucl_dist <= 0.8) )] #0.8
        # return human_body

        pcl_       = pcl.PointCloud(np.array(human_body[:,:3], dtype=np.float32))
        sor        = pcl_.make_voxel_grid_filter()
        # sor.set_leaf_size(0.01, 0.01, 0.01)
        sor.set_leaf_size(0.02, 0.02, 0.02)
        filtered   = sor.filter()
        human_body = np.asarray(filtered) 
        return human_body

    def voxelization_raw_data(self, human_body):
        dataset      = pd.DataFrame({'x': human_body[:,0], 'y': human_body[:,1], 'z': human_body[:,2]})
        cloud        = PyntCloud(dataset)
        voxelgrid_id = cloud.add_structure("voxelgrid", n_x=32, n_y=32, n_z=32)
        voxelgrid    = cloud.structures[voxelgrid_id]
        x_cords      = voxelgrid.voxel_x
        y_cords      = voxelgrid.voxel_y
        z_cords      = voxelgrid.voxel_z
        voxel        = np.zeros((32, 32, 32)).astype(np.bool)

        for x, y, z in zip(x_cords, y_cords, z_cords):
            voxel[x][y][z] = True
        return voxel

    def close_all(self):
        if self.visual_ptk1 is not None:
            self.visual_ptk1.close()

        if self.visual_ptk2 is not None:                
            self.visual_ptk2.close()
        
        plt.close('all')

    def load_data(self, path):
        datasets = np.load(path)
        data     = datasets['data']
        labels   = datasets['labels']
        return data, labels

    def save_data(self, path, data, labels):
        data   = np.array(data)
        labels = np.array(labels)
        rospy.loginfo('[Display] Saving data: {}'.format(path))
        rospy.loginfo('[Display] Total data : {}'.format(data.shape))
        rospy.loginfo('[Display] Total label: {}'.format(labels.shape))
        np.savez(path, data=data, labels=labels)

    def rotate_pcl(self, pcl, theta):
        x      = pcl[:,0]
        y      = pcl[:,1]
        z      = pcl[:,2]
        theta  = np.radians(theta)
        ox, oy = np.mean(x), np.mean(y)

        qx = ((x - ox) * np.cos(theta)) - ((y - oy) * np.sin(theta)) + ox
        qy = ((x - ox) * np.sin(theta)) + ((y - oy) * np.cos(theta)) + oy
        qx = qx.astype(int)
        qy = qy.astype(int)
        return np.stack((qx, qy, z), axis=1)

    def translate_pcl(self, pcl, diff):
        diff      = np.array(diff)
        pcl[:,:2] = pcl[:,:2] + diff 
        return pcl

    def voxelization(self, pcl):
        x_cords = pcl[:,0]
        y_cords = pcl[:,1]
        z_cords = pcl[:,2]
        voxel   = np.zeros((32, 32, 32)).astype(np.bool)
        for x, y, z in zip(x_cords, y_cords, z_cords):
            voxel[x][y][z] = True
        return voxel

    def run(self):
        # raw_files = os.listdir(self.pcl_raw_path)
        # rospy.loginfo('[Display] Raw data : {}'.format(raw_files))
        # print()
        # f = raw_files[100]

        # f = 'right_arm_top-566.npz'
        # f = 'left_arm_top-413.npz'
        f = 'thormang3_cross_arm_pcl-24.npz'
        file_name         = self.pcl_raw_path + f
        pcl_file          = np.load(file_name)
        self.point_clouds = pcl_file['pcl']
        self.visual_ptk1  = self.plot_point_cloud(f, self.point_clouds) # <-- plot

        try:
            human_body   = self.filter_raw_data(self.point_clouds)
        except:
            pass
        # self.visual_ptk2 = self.plot_point_cloud(f, human_body, big_point=True, color=True )

        # voxelization_raw_data
        voxel   = self.voxelization_raw_data(human_body)
        x, y, z = voxel.nonzero()
        pcl     = np.stack((x, y, z), axis=1)

        self.visual_ptk2 = self.plot_point_cloud(f, pcl, big_point=True, color=True )
        # self.visual_ptk2.capture('screenshot.png')

        fig = plt.figure()
        ax  = fig.add_subplot(1,1,1, projection='3d')
        # self.plot_pcl(ax, x, y, z, title='Point Cloud')
        self.plot_pcl(ax, x, y, z)

        fig = plt.figure()
        ax = fig.gca(projection='3d')
        # self.plot_voxel(ax, voxel, title="Voxel")
        self.plot_voxel(ax, voxel)
        
        # plt.show(block=False)
        plt.show()
        self.close_all()

        rospy.loginfo('[Display] Exit code')

if __name__ == '__main__':
    display = Display()
    display.run()