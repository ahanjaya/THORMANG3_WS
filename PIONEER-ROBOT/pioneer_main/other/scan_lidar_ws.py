#!/usr/bin/env python3

import json
import rospy
import rospkg
import numpy as np
from time import sleep
from pioneer_motor.motor import Motor
from pioneer_motion.motion import Motion
from pioneer_sensors.sensor import Sensor

import matplotlib
matplotlib.use('TkAgg') 
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

rospack = rospkg.RosPack()
lidar_path     = rospack.get_path("pioneer_main") + "/config/thormang3_lidar.npz"
workspace_path = rospack.get_path("pioneer_main") + "/config/thormang3_workspace.json"
np.set_printoptions(suppress=True)

def scan_workspace(motor, init_head_p, sensor):
    global tilt_rate, tilt_min, tilt_max
    lidar_ranges = []
    velocity     = 1
    rate         = rospy.Rate(20)

    # move head to start position
    head_pitch = tilt_max
    motor.set_joint_states(['head_y', 'head_p'], [0.0, head_pitch], [0, velocity])

    while head_pitch != np.round(motor.joint_position['head_p'],0):
        pass # wait until reaches to start position

    # calculate height robot when facing down, by using middle point from lidar
    # middle_p      = len(sensor.lidar_ranges) // 2
    # offset_m      = 2
    # height_robot  = np.array( sensor.lidar_ranges[ middle_p-offset_m:middle_p+offset_m ] )
    # height_robot  = np.mean( height_robot )
    height_robot = sensor.lidar_ranges
    # rospy.loginfo("Height robot: {0} meter" .format(height_robot))

    # scanning per each degree
    while not rospy.is_shutdown():
        lidar_ranges.append(sensor.lidar_ranges)
        # print(head_pitch)
        head_pitch += tilt_rate

        if head_pitch < tilt_min :		
            head_pitch = tilt_min
            break
        elif head_pitch > tilt_max :	
            head_pitch = tilt_max

        motor.set_joint_states(['head_y', 'head_p'], [0.0, head_pitch])
        rate.sleep()

    # save lidar data
    lidar_ranges = np.array(lidar_ranges)
    # np.savez(lidar_path, height_robot=height_robot, lidar_ranges=lidar_ranges)

    return height_robot, lidar_ranges

def plots_(axes_, x_, y_, legend_=None, xlabel_="", ylabel_="", title_=""):
    axes_.plot(x_, y_, 'o-', label=legend_)
    axes_.set_xlabel(xlabel_)
    axes_.set_ylabel(ylabel_)
    axes_.set_title(title_)
    # axes_.set_xscale('linear')
    axes_.grid()  

def plot_3Dworkspace(height_robot, lidar_ranges):
    global head_pitches
    lidar_points_x = []
    lidar_points_y = []
    lidar_points_z = []
    layers         = []
    
    for idx, points in enumerate(lidar_ranges):
        z             = points * np.cos( np.radians(90 - head_pitches[idx]) )
        height_object = height_robot - z
        lidar_points_z.append(height_object) # acquire lidar data per tilt_rate
        x             = points * np.sin( np.radians(90 - head_pitches[idx]) )
        lidar_points_x.append(x)
        y             = np.arange(len(points))
        lidar_points_y.append(y)

        layers.append(np.mean(height_object))

    # 3D point cloud
    fig3d = plt.figure()
    axes  = fig3d.add_subplot(111, projection='3d')
    X = np.array(lidar_points_y)  # 
    Y = np.array(lidar_points_x)  # x object
    Z = np.array(lidar_points_z)  # z object
    # axes.scatter(X, Y, Z)#, marker='.')
    axes.plot_wireframe(X, Y, Z, rstride=5, cstride=5)
    return layers

def plot_x_edge(layers):
    global head_pitches, tilt_rate 
    x_winsize      = 5
    first_x_win    = True
    dz_list        = []
    hp_list        = []

    for i in range (len(layers)):
        if i % x_winsize == 0 and i != 0:
            start = i-x_winsize
            end   = i
            curr_win = np.array( layers[start:end] )
            if not first_x_win:
                dz = np.power(curr_win - prev_win, 2)
                dz = np.sqrt( np.sum(dz) )
                dz_list.append(dz)
                hp_list.append(head_pitches[i])
            first_x_win = False
            prev_win = curr_win

    # Edge plotting on x axis
    fig2, axes = plt.subplots(nrows=1, ncols=1)
    plots_(axes, hp_list, dz_list, None, "head pitch", "dz", "Windows X Edge")

    # calculate edge on window x
    dz_list        = np.array(dz_list)
    diff_dz_list   = np.absolute( np.diff(dz_list) )
    idx_win_edge_x = np.argmax(diff_dz_list)
    # rospy.loginfo('Head Pitch Window X: {0}'. format(hp_list[idx_win_edge_x]))

    # selected edge on window x
    edge_winx = np.where(head_pitches == hp_list[idx_win_edge_x])
    edge_winx = int(edge_winx[0])
    # print('edge_winx:', edge_winx)
    print('win head_pitches: ', head_pitches[edge_winx])

    # iterating on selected edge window x
    sub_win        = np.array( layers[edge_winx-x_winsize : edge_winx+x_winsize] )
    print('Sub Head Win X: ', head_pitches[edge_winx-x_winsize : edge_winx+x_winsize])
    # print('Sub Win X: ', sub_win)

    # diff_sub       = np.absolute( np.diff(sub_win) )
    # # print(diff_sub)
    # edge_point     = sub_win[ np.argmax(diff_sub) ]
    # # rospy.loginfo('Edge Point: {0}'.format(edge_point)) 

    # diff_sub       = np.absolute( np.diff(sub_win) )
    # print(diff_sub)
    edge_point     = sub_win[ np.argmin(sub_win) ]
    # rospy.loginfo('Edge Point: {0}'.format(edge_point)) 

    idx_res        = layers.index(edge_point)
    head_pitch_max = head_pitches[idx_res]
    rospy.loginfo('Head Pitch Max: {0} deg.'.format(head_pitch_max))  

    fig3, axes = plt.subplots(nrows=1, ncols=1)
    plots_(axes, head_pitches, layers, None, "head pitch", "height object", "X Edge")
    axes.set_xticks(np.arange(max(head_pitches), min(head_pitches)+tilt_rate, tilt_rate*x_winsize))

    config = {}
    config['min_pitch'] = head_pitches[0]
    config['max_pitch'] = head_pitch_max

    with open(workspace_path, 'w') as f:
        json.dump(config, f, sort_keys=True, indent=4)
    
    return head_pitch_max

def calc_edge_y(data, dir):
    step    = len(dir)/10
    angles  = data[dir]    
    win     = np.split(angles, step)
    idx_win = np.split(dir, step)
    dz_list = []

    for i in range (len(win)-1):
        dz = win[i]-win[i+1]
        dz = np.power(dz, 2)
        dz = np.sqrt(np.sum(dz))
        dz_list.append(dz)

    # calculate edge on window
    dz_list    = np.array(dz_list)
    ddz        = np.absolute( np.diff(dz_list) )
    idx_dzl    = np.argmax(ddz)

    # selected 3 windows edge
    edge_win   = win[idx_dzl:idx_dzl+3]
    edge_win   = np.concatenate(edge_win)
    diff_sub   = np.absolute( np.diff(edge_win) )
    edge_idx   = np.argmax(diff_sub)

    # result index
    res_edge_win  = idx_win[idx_dzl:idx_dzl+3]
    res_edge_win  = np.concatenate(res_edge_win)
    
    return res_edge_win[edge_idx]

def plot_y_edge(angles, max_pitch):
    global head_pitches
    i      = 0

    config = None
    with open(workspace_path, 'r') as f:
        config = json.load(f)

    while not rospy.is_shutdown():
        # print(head_pitches[i])
        # fig4, axes = plt.subplots(nrows=1, ncols=1)
        # axes.plot(angles[i], '-o')

        data         = angles[i]
        mid_point    = len(data)//2
        left         = np.arange(mid_point, 0, -1)
        right        = np.arange(mid_point, len(data)-1)
        
        edge_left    = calc_edge_y(data, left)
        edge_right    = calc_edge_y(data, right)
        config[ head_pitches[i] ] = int(edge_left), int(edge_right)

        # print('Edge left: ', edge_left )
        # print('Edge right: ', edge_right )      
        # plt.show(block=False)
        # # plt.pause(1)
        # input("Press [enter] to close.")
        # plt.close()

        if head_pitches[i] == max_pitch:
            with open(workspace_path, 'w') as f:
                json.dump(config, f, indent=4)
            break

        i += 1

def plot_3Dresult(height_robot, lidar_ranges):
    global head_pitches
    lidar_points_x = []
    lidar_points_y = []
    lidar_points_z = []

    i = 0

    with open(workspace_path, 'r') as f:
        config = json.load(f)

    while not rospy.is_shutdown():
        edge_left, edge_right = config[str(head_pitches[i])]

        points                       = np.zeros( len(lidar_ranges[i]) )
        points[edge_left:edge_right] = lidar_ranges[i][edge_left:edge_right]
        points[points == 0]          = np.nan    

        hr                           = np.zeros( len(height_robot) )
        hr[edge_left:edge_right]     = height_robot[edge_left:edge_right]
        hr[hr == 0]                  = np.nan

        z             = points * np.cos( np.radians(90 - head_pitches[i]) )
        height_object = hr - z
        lidar_points_z.append(height_object) # acquire lidar data per tilt_rate
        x             = points * np.sin( np.radians(90 - head_pitches[i]) )
        lidar_points_x.append(x)
        y             = np.arange(len(points))
        lidar_points_y.append(y)
        # print(head_pitches[i])

        if head_pitches[i] == config['max_pitch']:
            break

        i += 1

    # 3D point cloud
    fig3d_1 = plt.figure()
    axes    = fig3d_1.add_subplot(111, projection='3d')
    X = np.array(lidar_points_y)  # 
    Y = np.array(lidar_points_x)  # x object
    Z = np.array(lidar_points_z)  # z object
    # axes.scatter(X, Y, Z)#, marker='.')
    axes.plot_wireframe(X, Y, Z, rstride=5, cstride=5)
    axes.set_zticks(np.arange(-0.1, 0.2, 0.1))

def main():
    rospy.init_node('pioneer_main', anonymous=False)
    rospy.loginfo("Pioneer Main - Running")

    # Create class object
    motor  = Motor()
    sensor = Sensor("Thormang3_Bear")
    # motor.publisher_(motor.module_control_pub, "direct_control_mode", latch=True)
    sleep(1)

    rate       = rospy.Rate(60)
    load_lidar = False

    global tilt_rate, tilt_min, tilt_max, head_pitches
    tilt_rate    = -0.5 # degree
    tilt_min     = 45
    tilt_max     = 90
    head_pitches = np.arange(tilt_max, tilt_min+tilt_rate, tilt_rate)

    if load_lidar:
        data = np.load(lidar_path)
        height_robot = data['height_robot']
        lidar_ranges = data['lidar_ranges']
        max_pitch = 83 # 48.5
    else:
        init_head_p = 30
        height_robot, lidar_ranges = scan_workspace(motor, init_head_p, sensor)

    # layers    = plot_3Dworkspace(height_robot, lidar_ranges)
    # max_pitch = plot_x_edge(layers)
    # motor.set_joint_states(['head_y', 'head_p'], [0.0, max_pitch])

    # plot_y_edge(lidar_ranges, max_pitch)

    # plot_3Dresult(height_robot, lidar_ranges)

    # while not rospy.is_shutdown():
    #     scan_workspace(motor, init_head_p, scan_offset)
    #     rospy.loginfo("scan finished")
    #     rate.sleep()
    #     break
    
    motor.kill_threads()
    sensor.kill_threads()
    print()
    plt.show(block=False)
    input("Press [enter] to close.\n")
    plt.close('all')

if __name__ == '__main__':
    main()