#!/usr/bin/env python3

import os
import sys
import pcl
import pptk
import rospy
import pygame
import rospkg
import ros_numpy
import numpy as np
import pandas as pd
import tensorflow as tf  
import matplotlib.pyplot as plt

from time import sleep
from pyntcloud import PyntCloud
from keras.models import load_model
from std_msgs.msg import String, Bool
from sensor_msgs.msg import PointCloud2

class Deep_Cross_Arm:
    def __init__(self, robot):
        self.run_robot    = self.str_to_bool(robot)
        rospack           = rospkg.RosPack()
        self.pcl_path     = rospack.get_path("pioneer_main") + "/data/cross_arm/raw_pcl/"
        self.rec_pcl_path = rospack.get_path("pioneer_main") + "/data/cross_arm/history/raw_pcl/"

        self.pcl_model    = rospack.get_path("pioneer_main") + "/data/cross_arm/cross_arm_model.h5"
        self.growth_memory() # limit tensorflow to use all of GPU resources
        self.model        = load_model(self.pcl_model)

        self.main_rate    = rospy.Rate(10)
        self.point_clouds = None
        self.scan_finish  = False # callback for read scanning finish
        self.shutdown     = False
        self.show_plt     = False
        self.visual_ptk1  = None
        self.visual_ptk2  = None

        self.display_text = True
        self.save_data    = True

        ## Publisher
        self.final_decision_pub = rospy.Publisher("/pioneer/cross_arm/final_decision",  String, queue_size=1)

        ## Subscriber
        rospy.Subscriber("/pioneer/shutdown_signal",       Bool,        self.shutdown_callback)
        rospy.Subscriber('/robotis/sensor/assembled_scan', PointCloud2, self.point_cloud2_callback)
        rospy.Subscriber('/robotis/sensor/move_lidar',     String,      self.lidar_turn_callback)

    def str_to_bool(self, s):
        if s == 'true':
            return True
        elif s == 'false':
            return False
        else:
            raise ValueError # evil ValueError that doesn't tell you what the wrong value was
    
    def growth_memory(self):
        physical_devices = tf.config.experimental.list_physical_devices('GPU')
        if len(physical_devices) > 0:
            for k in range(len(physical_devices)):
                tf.config.experimental.set_memory_growth(physical_devices[k], True)
                print('memory growth:', tf.config.experimental.get_memory_growth(physical_devices[k]))
        else:
            print("Not enough GPU hardware devices available")

    def shutdown_callback(self, msg):
        self.shutdown = msg.data
        rospy.loginfo("[DCA] Shutdown time")
        rospy.signal_shutdown('Exit')

    def point_cloud2_callback(self, data):
        pc          = ros_numpy.numpify(data)
        points      = np.zeros((pc.shape[0],4))
        points[:,0] = pc['x']
        points[:,1] = pc['y']
        points[:,2] = pc['z']
        points[:,3] = pc['intensities']
        # print(pc.dtype.names) # ('x', 'y', 'z', 'intensities', 'index')
        self.point_clouds = points
        self.visual_ptk1  = self.plot_point_cloud('raw_data', self.point_clouds, big_point=True, color=True )

    def lidar_turn_callback(self, msg):
        if msg.data == "start":
            rospy.loginfo("[DCA] Start scan")
            self.scan_finish = False
            self.close_all()
        elif msg.data == "end":
            rospy.loginfo("[DCA] Finish scan")
            self.scan_finish = True

    def plot_point_cloud(self, label, pcl_data, big_point=False, color=True):
        rospy.loginfo("[Pre.] {} - length pcl : {}".format(label, pcl_data.shape))
        visual_ptk = pptk.viewer(pcl_data[:,:3])

        if color:
            visual_ptk.attributes(pcl_data[:,-1])
        if big_point:
            visual_ptk.set(point_size=0.0025)

        return visual_ptk

    def plot_2d(self, axes_, x_, y_, legend_=None, xlabel_="", ylabel_="", title_=""):
        axes_.plot(x_, y_, 'o-', label=legend_)
        axes_.set_xlabel(xlabel_)
        axes_.set_ylabel(ylabel_)
        axes_.set_title(title_)
        axes_.grid()

    def plot_voxel(self, ax, voxel, title=""):
        ax.voxels(voxel)
        ax.set_xlabel('x')
        ax.set_ylabel('y')
        ax.set_zlabel('z')
        ax.set_xlim(0, 32)
        ax.set_ylim(0, 32)
        ax.set_zlim(0, 32)
        ax.set_title(title) 

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

        if self.show_plt:
            _, axes2D = plt.subplots(nrows=1, ncols=1)
            self.plot_2d(axes2D, y_step[:len(z_list)], z_list, legend_=None,\
                xlabel_="y-layer", ylabel_="z-height", title_="Filtering Human Body on Point Cloud")
        
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

        if self.show_plt:
            fig = plt.figure()
            ax = fig.gca(projection='3d')
            ax.voxels(voxel)
            plt.show(block=False)

        return voxel

    def prediction(self, model, x):
        result = model.predict_classes(x)
        # print(result)

        result = result[0]
        arm    = String()

        if result == 0:
            rospy.loginfo('[DCA] Left arm on Top')
            rospy.loginfo('[DCA] Right arm on Bottom')
            arm.data = 'right_arm'

        elif result == 1:
            rospy.loginfo('[DCA] Right arm on Top')
            rospy.loginfo('[DCA] Left arm on Bottom')
            arm.data = 'left_arm'

        self.final_decision_pub.publish(arm)

        if self.display_text:
            self.render_text(arm.data)
            
    def render_text(self, data):
        pygame.init()
        screen = pygame.display.set_mode((640, 480))
        font   = pygame.font.SysFont("comicsansms", 72)

        if data == 'right_arm':
            text = font.render("right arm on bottom", True, (0, 0, 128))
        elif data == 'left_arm':
            text = font.render("left arm on bottom", True, (0, 128, 0))

        screen.fill((255, 255, 255))
        screen.blit(text, (320 - text.get_width() // 2, 240 - text.get_height() // 2))
        pygame.display.flip()

    def close_all(self):
        if self.visual_ptk1 is not None:
            self.visual_ptk1.close()

        if self.visual_ptk2 is not None:                
            self.visual_ptk2.close()

        if self.display_text:
            pygame.quit()
        
        plt.close('all')
        
    def run(self):
        try:
            while not rospy.is_shutdown():
                if self.run_robot:
                    while not self.scan_finish:
                        if self.shutdown: 
                            break
                        else:
                            pass

                    self.scan_finish = False
                    if self.shutdown:
                        break
                    sleep(1)

                    # save data
                    if self.save_data:
                        counter = len(os.walk(self.rec_pcl_path).__next__()[2])
                        np.savez(self.rec_pcl_path + "thormang3_cross_arm_pcl-" + str(counter) + ".npz", pcl=self.point_clouds)
                        rospy.loginfo('[DCA] save: thormang3_cross_arm_pcl-{}.npz'.format(counter))

                else:
                    counter  = 570
                    arm_data = 'right_arm_top'
                    # arm_data = 'left_arm_top'
                    pcl_file = self.pcl_path + arm_data+ "-" + str(counter) + ".npz"
                    data     = np.load(pcl_file)
                    self.point_clouds = data['pcl']

                # process data
                try:
                    human_body        = self.filter_raw_data (self.point_clouds)
                except:
                    rospy.logwarn('[DCA] Failed to filter human body point cloud')
                    human_body = self.point_clouds
                self.visual_ptk2  = self.plot_point_cloud('human_body', human_body, big_point=True, color=True)

                # voxelization_raw_data
                voxel   = self.voxelization_raw_data(human_body)
                x_input = voxel.reshape(1, 32, 32, 32, 1) # reshaping data

                # deep learning prediction
                self.prediction(self.model, x_input)

                if self.show_plt:
                    plt.show(block=False)
                    input("[DCA] Press [enter] to close.\n")
                    plt.close('all')

                if not self.run_robot:
                    break

                self.main_rate.sleep()

        except KeyboardInterrupt:
            self.shutdown = True

if __name__ == '__main__':
    rospy.init_node('pioneer_deep_cross_arm', disable_signals=True)

    # if using ros launch length of sys.argv is 4
    if len(sys.argv) == 4:
        robot = sys.argv[1]
        rospy.loginfo("[DCA] Pioneer Deep Cross Arm - Running")
        rospy.loginfo("[DCA] Using Robot : {}\n".format(robot))
        dca = Deep_Cross_Arm(robot)
        dca.run()
    else:
        rospy.logerr("[DCA] Exit Argument not fulfilled")


    