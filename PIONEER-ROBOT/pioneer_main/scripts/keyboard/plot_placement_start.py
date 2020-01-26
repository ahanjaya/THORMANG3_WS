#!/usr/bin/env python3
 
import cv2
import yaml
import rospkg
import numpy as np
import pandas as pd

class Plot():
    def __init__(self):
        rospack        = rospkg.RosPack()
        self.data_path      = rospack.get_path("pioneer_main") + "/data/keyboard_placement/keyboard_placement.xlsx"
        self.ws_config_path = rospack.get_path("pioneer_main") + "/config/thormang3_align_keyboard_ws.yaml"

        self.len_line = 25

    def load_config(self, arm):
        try:
            with open(self.ws_config_path, 'r') as f:
                aruco_ws = yaml.safe_load(f)
        except yaml.YAMLError as exc:
            print(exc)

        try:
            config = aruco_ws[arm]
            pts    = []
            for i in ['P1', 'P2', 'P3', 'P4']:
                pts.append( [config[i]['cx'], config[i]['cy']] )
            pts = np.array( pts, np.int32)
            return pts.reshape((-1,1,2))
        except:
            return None

    def translate_point(self, point):
        half = self.len_line // 2 * -1
        return tuple(np.array(point) + np.array([half, 0]))

    def pixel_to_cm(self, axes, pixel):
        if axes == 'x':
            return np.interp( pixel, [120, 500], [0, 45] )
        elif axes == 'y':
            return np.interp( pixel, [180, 435], [0, 35] )
    
    def calc_position_err(self):
        df = pd.read_excel(self.data_path, sheetname='Sheet1')

        final_pose = list(zip( df['Actual Final (X)'], df['Actual Final (Y)'] ))

        print(final_pose)

        for pose in final_pose:
            position_err_cm  = np.linalg.norm( np.array([ self.pixel_to_cm('x', 318), self.pixel_to_cm('y', 353) ]) - \
                                           np.array([ self.pixel_to_cm('x', pose[0]), self.pixel_to_cm('y', pose[1]) ])) # cm
        
            print(position_err_cm)

    def run(self):
        left_ws  = self.load_config('left_arm')
        right_ws = self.load_config('right_arm')

        df = pd.read_excel(self.data_path, sheetname='Sheet1')
        print("Column headings:", df.columns)
        start_pose = list(zip(df['Actual Start (X)'], df['Actual Start (Y)'], df['Actual Start (Theta)']))
        # start_pose = list(zip(df['Actual Final (X)'], df['Actual Final (Y)'], df['Actual Final (Theta)']))
        # print(start_pose)

        mask = np.full((480, 640, 3), 255, np.uint8)
        for pose in start_pose:
            cv2.circle(mask, pose[:-1], 2, (0,0,0), -1)

            end_x = pose[0] + int( self.len_line * np.cos(np.radians(pose[2])) )
            end_y = pose[1] + int( self.len_line * np.sin(np.radians(pose[2])) )
            # cv2.line(mask, pose[:-1], (end_x, end_y), (255,255,0), 1)

            start = self.translate_point(pose[:-1])
            # cv2.circle(mask, start, 2, (255,0,0), -1)

            end = self.translate_point( (end_x, end_y) )
            # cv2.circle(mask, end, 2, (0,0,255), -1)

            cv2.line(mask, start, end, (0,255,0), 1)


        cv2.polylines(mask,[left_ws],  True, (255,0,0), 2)
        cv2.polylines(mask,[right_ws], True, (0,0,255), 2)
        cv2.circle(mask, (318, 353), 10, (0,0,0), -1)
        cv2.namedWindow("frame")
        cv2.imshow("frame", mask)
        cv2.waitKey()

        # self.calc_position_err()

if __name__ == "__main__":
    plot = Plot()
    plot.run()