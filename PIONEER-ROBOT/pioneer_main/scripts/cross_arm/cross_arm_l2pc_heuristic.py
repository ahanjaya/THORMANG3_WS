#!/usr/bin/env python3

import os
import sys
import pcl
import pptk
import rospy
import pygame
import random
import rospkg
import ros_numpy
import numpy as np
from time import sleep
import matplotlib.pyplot as plt
from std_msgs.msg import String, Bool
from sensor_msgs.msg import PointCloud2

np.set_printoptions(suppress=True)

class Lidar_Cross_Arm:
    def __init__(self, robot):
        self.robot        = self.str_to_bool(robot)
        self.shutdown     = False

        rospack           = rospkg.RosPack()
        self.pcl_path     = rospack.get_path("pioneer_main") + "/data/cross_arm/raw_pcl/"
        self.main_rate    = rospy.Rate(10)
        self.point_clouds = None
        self.lidar_finish = False # callback for read scanning finish
        self.first_edge   = False

        self.debug        = True # showing matplotlib plots
        self.save_data    = True # saving scanning data
        self.flip         = True # flipping data x axes & y axes

        self.display_text = False

        if self.display_text:
            pygame.init()
            self.screen = pygame.display.set_mode((640, 480))
            self.font = pygame.font.SysFont("comicsansms", 72)

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

    def shutdown_callback(self, msg):
        self.shutdown = msg.data
        rospy.loginfo("[CAL] Shutdown time")
        rospy.signal_shutdown('Exit')

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
            rospy.loginfo("[CAL] Scan finished")
            self.lidar_finish = True

    def plot_point_cloud(self, label, pcl_data, big_point=False, color=True):
        rospy.loginfo("[CAL] {} - length pcl : {}".format(label, pcl_data.shape))
        display = True

        if display:
            visual_ptk = pptk.viewer(pcl_data[:,:3])
            if color:
                visual_ptk.attributes(pcl_data[:,-1])
            if big_point:
                visual_ptk.set(point_size=0.0025)
            
            # v.attributes(points[['r', 'g', 'b']] / 255., points['i'])
            # color      = pcl_data.copy()
            # color[:,:] = [255, 0, 0]
            # visual_ptk.attributes(color)

    def plots_(self, axes_, x_, y_, legend_=None, scatter=False, xlabel_="", ylabel_="", title_=""):
        if scatter:
            axes_.scatter(x_, y_, label=legend_)
        else:
            axes_.plot(x_, y_, 'o-', label=legend_)
        
        axes_.set_xlabel(xlabel_)
        axes_.set_ylabel(ylabel_)
        axes_.set_title(title_)
        # axes_.set_xlim([-0.3, 0.3])
        # axes_.set_xscale('linear')
        # axes_.invert_xaxis()
        axes_.grid()  

    def check_consecutive(self, arr):
        arr  = np.array(arr)
        ones = np.where(arr == 1)

        if ones[0].size != 0:
            l = list(ones[0])
            return l == list(range(min(l), max(l)+1))
        else:
            return False

    def middle_consecutive(self, arr):
        arr  = np.array(arr)
        ones = np.where(arr == 1)
        return (ones[0][0] + ones[0][-1]) // 2

    def filtering_raw_data(self, data):
        # plot original data
        if self.debug:
            self.plot_point_cloud('point_clouds', data) # <-- plot

        ####
        # Removing Table <-- this code will be removed on real practice
        if not self.flip:
            data_y = data[:,1] # normal
        else:
            data_y = data[:,0] # flip
        data = data[ np.where( (data_y >= 0.4) )]
        # self.plot_point_cloud('point_clouds', data) # <-- plot
        ####
        
        # flipping data between X Axes & Y Axes
        if not self.flip:
            data_y = data[:,1] # normal
        else:
            data_y = data[:,0] # flip

        # Sorting point cloud
        data = data[np.argsort(data_y)] # Y axis
        if not self.flip:
            data_y = data[:,1] # normal
        else:
            data_y = data[:,0] # flip

        # square scaning
        ymin = np.round( np.min(data_y), 1) - 0.1
        ymax = np.round( np.max(data_y), 1) + 0.1
        y_step_size = 0.025 #0.01
        y_step      = np.arange(ymin, ymax, y_step_size)

        # rospy.loginfo('[CAL] Y Point: {}'.format( (ymin, ymax) ))
        # print(y_step)
        # print()

        z_list = []
        z_prev = None

        for y in range(len(y_step)):
            y_ll = y_step[y]
            y_ul = y_step[y] + y_step_size
            # print('Y : {:.3f}, {:.3f}'.format(y_ll, y_ul) )

            layer = data[np.where( (data_y >= y_ll) & (data_y < y_ul) )]
            if layer.size != 0:
                zmax = np.max(layer[:,2])
                # print('\t\ttotal data: {}, max z : {:.4f}'.format(len(layer), zmax ) )
                z_list.append(zmax)
                if z_prev != None:
                    diff = zmax-z_prev
                    if diff <= -0.5:
                        y_lim = y_ll
                        break
                z_prev = zmax
            # print()

        if self.debug:
            _, axes2D = plt.subplots(nrows=1, ncols=1)
            self.plots_(axes2D, y_step[:len(z_list)], z_list, legend_=None, scatter=False, \
                xlabel_="y-layer", ylabel_="z-height", title_="Filtering Human Body on Point Cloud")
        
        human_body      = data[np.where( (data_y >= ymin) & (data_y < y_lim) )]
        flag_human_body = human_body
        # self.plot_point_cloud('human_body', human_body) # <-- plot

        # Reference point
        if not self.flip:
            x_ref = ( np.min(human_body[:,0]) + np.max(human_body[:,0]) ) / 2
            y_ref = np.min(human_body[:,1])
        else:
            x_ref = np.min(human_body[:,0])
            y_ref = ( np.min(human_body[:,1]) + np.max(human_body[:,1]) ) / 2

        z_ref     = np.max(human_body[:,2])
        ref_point = np.array([ [x_ref, y_ref, z_ref] ])
        # rospy.loginfo('[CAL] Ref. Point: {}'.format(ref_point))

        # Euclidean Distance of 3D Point
        eucl_dist = np.linalg.norm(ref_point - human_body[:,:3], axis=1)

        try:
            # Filter euclidean distance
            human_body = human_body[ np.where( (eucl_dist <= 0.8) )] #0.8
            # self.plot_point_cloud('human_body', human_body, big_point=True, color=True )

            p   = pcl.PointCloud(np.array(human_body[:,:3], dtype=np.float32))
            sor = p.make_voxel_grid_filter()
            sor.set_leaf_size(0.01, 0.01, 0.01)
            filtered = sor.filter()

            filtered_human_body = np.asarray(filtered)
            if self.debug:
                self.plot_point_cloud('filtered_human_body', filtered_human_body) #, big_point=True, color=True )
            return filtered_human_body

        except:
            if self.debug:
                self.plot_point_cloud('human_body', flag_human_body) #, big_point=True, color=True )
            return flag_human_body

    def paw_decision(self, human_body):
        # Reference point
        if not self.flip:
            x_ref = ( np.min(human_body[:,0]) + np.max(human_body[:,0]) ) / 2
            y_ref = np.min(human_body[:,1])
        else:
            x_ref = np.min(human_body[:,0])
            y_ref = ( np.min(human_body[:,1]) + np.max(human_body[:,1]) ) / 2

        z_ref     = np.max(human_body[:,2])
        ref_point = np.array([ [x_ref, y_ref, z_ref] ])

        # 2D square scaning
        if not self.flip:
            data_x = human_body[:,0]
            data_y = human_body[:,1]
        else:
            data_x = human_body[:,1]
            data_y = human_body[:,0]

        xmin = np.round( np.min(data_x), 1) - 0.1
        xmax = np.round( np.max(data_x), 1) + 0.1
        ymin = np.round( np.min(data_y), 1) - 0.1
        ymax = np.round( np.max(data_y), 1) + 0.1

        len_square = 0.025 #0.01
        x_step = np.arange(xmin, xmax, len_square)
        y_step = np.arange(ymin, ymax, len_square)

        edge_point = []
        for y in range(len(y_step)):
            y_ll   = y_step[y]
            y_ul   = y_step[y] + len_square 
            # print('Y : {:.2f}, {:.2f}'.format(y_ll, y_ul) )
            binary = []
            for x in range(len(x_step)):
                x_ll   = x_step[x]
                x_ul   = x_step[x] + len_square
                # print('\t\tX : {:.2f}, {:.2f}'.format(x_ll, x_ul) )

                # small_cube = human_body[np.where( (human_body[:,0] >= x_ll) & (human_body[:,0] < x_ul) & \
                #                                     (human_body[:,1] >= y_ll) & (human_body[:,1] < y_ul) )]
                small_cube = human_body[np.where( (data_x >= x_ll) & (data_x < x_ul) & \
                                                    (data_y >= y_ll) & (data_y < y_ul) )]
                if small_cube.size != 0:  binary.append(1)
                else:                     binary.append(0)
            
            # print('\t', binary)
            edge_point.append(binary)
            if self.check_consecutive(binary):
                if not all(elem == 0 for elem in edge_point[-2]):
                # print('\t\t',edge_point[-2])
                    ymax = y_ul
                    # print(ymax)
                    break

        # show cross arm
        crossed_arm = human_body[np.where( (data_y >= ymin) & (data_y < ymax))]
        if self.debug:
            self.plot_point_cloud('crossed_arm', crossed_arm, big_point=True, color=True )

        if not self.flip:
            data_x_cross_arm = crossed_arm[:,0]
            data_y_cross_arm = crossed_arm[:,1]
        else:
            data_x_cross_arm = crossed_arm[:,1]
            data_y_cross_arm = crossed_arm[:,0]
            
        # seperate cross arm by screen
        x_mid = self.middle_consecutive(edge_point[-1])
        if not self.flip:
            left_screen  = crossed_arm[np.where( (data_x_cross_arm <= x_step[x_mid] ) )]
            right_screen = crossed_arm[np.where( (data_x_cross_arm >  x_step[x_mid] ) )]
        else:
            left_screen  = crossed_arm[np.where( (data_x_cross_arm > x_step[x_mid] ) )]
            right_screen = crossed_arm[np.where( (data_x_cross_arm <=  x_step[x_mid] ) )]

        # plot each arms
        if self.debug:
            self.plot_point_cloud('left_screen',  left_screen,  big_point=True, color=True )
            self.plot_point_cloud('right_screen', right_screen, big_point=True, color=True )

        # calculate average distance of each screen
        left_screen_dist  = np.linalg.norm(ref_point - left_screen[:,:3], axis=1)
        left_screen_dist  = np.mean(left_screen_dist)
        # left_screen_dist   = np.min(left_screen_dist)

        right_screen_dist = np.linalg.norm(ref_point - right_screen[:,:3], axis=1)
        right_screen_dist = np.mean(right_screen_dist)
        # right_screen_dist  = np.min(right_screen_dist)

        # decision
        self.final_decision(left_screen_dist, right_screen_dist)

        if self.debug:
            temp_y = []
            for y in range(len(y_step)):
                ll   = y_step[y]
                ul   = y_step[y] + len_square
                temp = human_body[ np.where( (data_y >= ll) & (data_y < ul))]
                temp_y.append(temp)
            
            _, axes2D = plt.subplots(nrows=1, ncols=1)
            for y in temp_y:
                if not self.flip:
                    self.plots_(axes2D, y[:,0], y[:,1], legend_=None, scatter=True, \
                        xlabel_="x-distance", ylabel_="y-distance", title_="Top View")
                else:
                    self.plots_(axes2D, y[:,1], y[:,0], legend_=None, scatter=True, \
                        xlabel_="x-distance", ylabel_="y-distance", title_="Top View")

            xlim_min, xlim_max = axes2D.get_xlim()
            ylim_min, ylim_max = axes2D.get_ylim()

            _, axes2D = plt.subplots(nrows=1, ncols=1)
            self.plots_(axes2D, data_x_cross_arm, data_y_cross_arm, legend_=None, scatter=True,\
                xlabel_="x-distance", ylabel_="y-distance", title_="2D Crossed Paw")
            axes2D.set_xlim([xlim_min, xlim_max])
            axes2D.set_ylim([ylim_min, ylim_max])

    def intersection_decision(self, human_body):
        if not self.flip:
            data_x = human_body[:,0]
            data_y = human_body[:,1]
        else:
            data_x = human_body[:,1]
            data_y = human_body[:,0]

        # 2D square scaning
        xmin = np.round( np.min(data_x), 1) - 0.1
        xmax = np.round( np.max(data_x), 1) + 0.1
        ymin = np.round( np.min(data_y), 1) - 0.1
        ymax = np.round( np.max(data_y), 1) + 0.1

        len_square = 0.025 #0.01
        x_step = np.arange(xmin, xmax, len_square)
        y_step = np.arange(ymin, ymax, len_square)

        try:
            edge_point = []
            for y in range(len(y_step)):
                y_ll   = y_step[y]
                y_ul   = y_step[y] + len_square 
                # print('Y : {:.2f}, {:.2f}'.format(y_ll, y_ul) )
                binary = []
                for x in range(len(x_step)):
                    x_ll   = x_step[x]
                    x_ul   = x_step[x] + len_square
                    # print('\t\tX : {:.2f}, {:.2f}'.format(x_ll, x_ul) )
                    small_cube = human_body[np.where( (data_x >= x_ll) & (data_x < x_ul) & \
                                                        (data_y >= y_ll) & (data_y < y_ul) )]
                    if small_cube.size != 0:  binary.append(1)
                    else:                     binary.append(0)
                
                # print('\t', binary)
                edge_point.append(binary)
                if self.check_consecutive(binary):
                    if not self.first_edge:
                        if not all(elem == 0 for elem in edge_point[-2]):
                            self.first_edge = True
                else:
                    # check last intersection
                    if self.first_edge:
                        ymin = y_ll
                        break

            # show cross arm
            crossed_arm = human_body[np.where( (data_y >= ymin) & (data_y < ymin + 0.1))] # 0.07
            # self.plot_point_cloud('crossed_arm', crossed_arm, big_point=True, color=True )

            # filter noise
            if not self.flip:
                x_ref = ( np.min(crossed_arm[:,0]) + np.max(crossed_arm[:,0]) ) / 2
                y_ref = np.min(crossed_arm[:,1])
            else:
                x_ref = np.min(crossed_arm[:,0])
                y_ref = ( np.min(crossed_arm[:,1]) + np.max(crossed_arm[:,1]) ) / 2

            z_ref     = np.max(crossed_arm[:,2])
            ref_point = np.array([ [x_ref, y_ref, z_ref] ])

            # Filter euclidean distance
            eucl_dist          = np.linalg.norm(ref_point - crossed_arm[:,:3], axis=1)
            filter_crossed_arm = crossed_arm[ np.where( (eucl_dist <= 0.2) )]
            if self.debug:
                self.plot_point_cloud('filter_crossed_arm', filter_crossed_arm, big_point=True, color=True )
            
            # seperate cross arm by screen
            if not self.flip:
                data_x_cross_arm = filter_crossed_arm[:,0]
                data_y_cross_arm = filter_crossed_arm[:,1]
            else:
                data_x_cross_arm = filter_crossed_arm[:,1]
                data_y_cross_arm = filter_crossed_arm[:,0]

            x_mid  = self.middle_consecutive(edge_point[-2])
            if not self.flip:
                left_screen  = filter_crossed_arm[np.where( (data_x_cross_arm <= x_step[x_mid] ) )]
                right_screen = filter_crossed_arm[np.where( (data_x_cross_arm >  x_step[x_mid] ) )]
            else:
                left_screen  = filter_crossed_arm[np.where( (data_x_cross_arm > x_step[x_mid] ) )]
                right_screen = filter_crossed_arm[np.where( (data_x_cross_arm <=  x_step[x_mid] ) )]
            
            # ploting arm
            if self.debug: 
                self.plot_point_cloud('left_screen',  left_screen,  big_point=True, color=True )
                self.plot_point_cloud('right_screen', right_screen, big_point=True, color=True )

            # calculate average z height each screen
            left_screen_dist  = np.mean(left_screen[:,2])
            right_screen_dist = np.mean(right_screen[:,2])

            # decision
            self.final_decision(left_screen_dist, right_screen_dist)

            if self.debug:
                temp_y = []
                for y in range(len(y_step)):
                    ll   = y_step[y]
                    ul   = y_step[y] + len_square
                    temp = human_body[ np.where( (data_y >= ll) & (data_y < ul))]
                    temp_y.append(temp)
                
                _, axes2D = plt.subplots(nrows=1, ncols=1)
                for y in temp_y:
                    if not self.flip:
                        self.plots_(axes2D, y[:,0], y[:,1], legend_=None, scatter=True, \
                            xlabel_="x-distance", ylabel_="y-distance", title_="2D Human Body")
                    else:
                        self.plots_(axes2D, y[:,1], y[:,0], legend_=None, scatter=True, \
                            xlabel_="x-distance", ylabel_="y-distance", title_="2D Human Body")

                xlim_min, xlim_max = axes2D.get_xlim()
                ylim_min, ylim_max = axes2D.get_ylim()

                _, axes2D = plt.subplots(nrows=1, ncols=1)
                self.plots_(axes2D, data_x_cross_arm, data_y_cross_arm, legend_=None, scatter=True,\
                    xlabel_="x-distance", ylabel_="y-distance", title_="2D Crossed Arm")
                axes2D.set_xlim([xlim_min, xlim_max])
                axes2D.set_ylim([ylim_min, ylim_max])

        except Exception as e:
            print(e)
            rospy.logwarn('[CAL] Random Guess')
            left_screen_dist  = random.random()
            right_screen_dist = random.random()
            self.final_decision(left_screen_dist, right_screen_dist)

    def final_decision(self, left_screen_dist, right_screen_dist):
        print()
        rospy.loginfo('[CAL] Left screen dist : {:.4f}'.format(left_screen_dist))
        rospy.loginfo('[CAL] Right screen dist : {:.4f}'.format(right_screen_dist))

        arm = String()
        if left_screen_dist < right_screen_dist:
            rospy.loginfo('[CAL] Left arm on Top')
            rospy.loginfo('[CAL] Right arm on Bottom')
            arm.data = 'right_arm'
        else:
            rospy.loginfo('[CAL] Right arm on Top')
            rospy.loginfo('[CAL] Left arm on Bottom')
            arm.data = 'left_arm'

        self.final_decision_pub.publish(arm)
        if self.display_text:
            text = self.font.render(arm.data + " on bottom", True, (0, 128, 0))
            self.screen.fill((255, 255, 255))
            self.screen.blit(text, (320 - text.get_width() // 2, 240 - text.get_height() // 2))
            pygame.display.flip()

    def run(self):
        try:
            while not rospy.is_shutdown():
                if self.robot:
                    while not self.lidar_finish:
                        if self.shutdown: 
                            break
                        else:
                            pass

                    if self.shutdown:
                        break

                    self.lidar_finish = False
                    sleep(1)
                    if self.save_data:
                        counter = len(os.walk(self.pcl_path).__next__()[2])
                        # counter = 100 # <-- override
                        np.savez(self.pcl_path + "thormang3_cross_arm_pcl-" + str(counter) + ".npz", pcl=self.point_clouds)
                        rospy.loginfo('[CAL] save: thormang3_cross_arm_pcl-{}.npz'.format(counter))
                else:
                    counter = 0
                    # data    = np.load(self.pcl_path + "thormang3_cross_arm_pcl-" + str(counter) + ".npz")
                    data    = np.load(self.pcl_path + "left_arm_top-" + str(counter) + ".npz")
                    self.point_clouds = data['pcl']
                
                # process data
                human_body = self.filtering_raw_data(self.point_clouds)

                # # decision function
                # self.paw_decision(human_body)
                self.intersection_decision(human_body)
 
                if self.debug:
                    plt.show(block=False)
                    input("[CAL] Press [enter] to close.\n")
                    plt.close('all')

                if not self.robot:
                    break

                self.main_rate.sleep()

        except KeyboardInterrupt:
            self.shutdown = True

if __name__ == '__main__':
    rospy.init_node('pioneer_cross_arm_lidar', disable_signals=True)

    # if using ros launch length of sys.argv is 4
    if len(sys.argv) == 4:
        robot = sys.argv[1]
        rospy.loginfo("[CAL] Pioneer Cross Arm Lidar- Running")
        rospy.loginfo("[CAL] Using Robot : {}\n".format(robot))
        lca = Lidar_Cross_Arm(robot)
        lca.run()
    else:
        rospy.logerr("[CAL] Exit Argument not fulfilled")