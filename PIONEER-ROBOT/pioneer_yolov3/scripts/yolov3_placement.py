#!/usr/bin/env python3

import sys
import rospy
import rospkg
rospack          = rospkg.RosPack()
yolo_config_path = rospack.get_path("pioneer_yolov3") + "/config/"

from utils import utils
from sort import *
from models import *
from utils.utils import *

import cv2
import glob
import yaml
import torch
import numpy as np
import os, sys, time, datetime, random

from PIL import Image
from time import sleep
from torch.autograd import Variable
from torch.utils.data import DataLoader
from torchvision import datasets, transforms

# ROS
from geometry_msgs.msg import Pose2D
from geometry_msgs.msg import Point32
from pioneer_vision.camera import Camera
from pioneer_utils.export_excel_keyboard import Excel
from std_msgs.msg import Bool
from std_srvs.srv import Trigger, TriggerResponse

rospy.init_node('pioneer_yolov3_placement', anonymous=False)
rospy.loginfo("[Yolo] Pioneer YoloV3 Placement")

# Publisher
left_arm_pos_pub      = rospy.Publisher("/pioneer/placement/left_arm_point",    Point32, queue_size=1)
right_arm_pos_pub     = rospy.Publisher("/pioneer/placement/right_arm_point",   Point32, queue_size=1)
approach_keyboard_pub = rospy.Publisher("/pioneer/placement/approach_keyboard", Bool,    queue_size=1)
placement_trigger_pub = rospy.Publisher("/pioneer/placement/placement_trigger", Bool,    queue_size=1)
grip_key_pub          = rospy.Publisher("/pioneer/placement/grasp_keyboard",    Bool,    queue_size=1)
ini_pose_pub          = rospy.Publisher("/pioneer/placement/init_pose",         Bool,    queue_size=1)
keyboard_pos_pub      = rospy.Publisher("/pioneer/placement/keyboard_position", Pose2D,  queue_size=1)
save_frame_pub        = rospy.Publisher("/pioneer/placement/save_frame",        Bool,    queue_size=1)
shutdown_pub          = rospy.Publisher("/pioneer/placement/shutdown_signal",   Bool,    queue_size=1)

ws_config_path        = rospack.get_path("pioneer_main") + "/config/thormang3_align_keyboard_ws.yaml"
data_path             = rospack.get_path("pioneer_main") + "/data/keyboard_placement/"
interupt_data_path    = rospack.get_path("pioneer_main") + "/data/"

save_data  = rospy.get_param("/pioneer/placement/save_data")
counter_start    = 0
counter_finish   = 0
save_start_frame = False
save_final_frame = False

if save_data:
    if not os.path.exists(data_path):
        os.mkdir(data_path)

    excel = Excel(data_path + 'keyboard_placement.xlsx')

    if not os.path.exists(data_path + "picts_actual_start"):
        os.mkdir(data_path + "picts_actual_start")

    if not os.path.exists(data_path + "picts_actual_finish"):
        os.mkdir(data_path + "picts_actual_finish")

    if not os.path.exists(data_path + "picts_simulation_finish"):
        os.mkdir(data_path + "picts_simulation_finish")
    
    counter_start  = len(os.walk(data_path + "picts_actual_start").__next__()[2])
    counter_finish = len(os.walk(data_path + "picts_actual_start").__next__()[2])

def pixel_to_cm(axes, pixel):
    if axes == 'x':
        return np.interp( pixel, [120, 500], [0, 45] )
    elif axes == 'y':
        return np.interp( pixel, [180, 435], [0, 35] )

keyboard_start_pose = {'x': None, 'y': None, 'theta': None}
good = fail = 0
def finish_placement_callback(msg):
    global counter_finish, simul_keyboard, frame, save_final_frame
    global good, fail
    
    if msg.data == True:
        print()
        counter_finish += 1
        rospy.loginfo('[Yolo] Finish placement: {}'.format(counter_finish))

        # actual start keyboard pose
        rospy.loginfo('[Yolo] Real keyboard START pose:  X: {},  Y: {}, Theta: {:.2f}'.format(keyboard_start_pose['x'], keyboard_start_pose['y'], keyboard_start_pose['theta']))

        # actual final keyboard pose
        rospy.loginfo('[Yolo] Real keyboard FINAL pose:  X: {},  Y: {}, Theta: {:.2f}'.format(keyboard.x, keyboard.y, keyboard.theta))
        # simulation keyboard pose
        rospy.loginfo('[Yolo] Simulation keyboard FINAL pose:  X: {},  Y: {}, Theta: {:.2f}'.format(simul_keyboard.x, simul_keyboard.y, simul_keyboard.theta))

        # actual position error
        position_err_pixel = np.linalg.norm(np.array([318, 353]) - np.array([keyboard.x, keyboard.y])) # pixel
        position_err_cm    = np.linalg.norm( np.array([ pixel_to_cm('x', 318), pixel_to_cm('y', 353) ]) - \
                                             np.array([ pixel_to_cm('x', keyboard.x), pixel_to_cm('y', keyboard.y) ])) # cm

        rospy.loginfo('[Yolo] Actual position error (Pixel): {:.2f}'.format(position_err_pixel))
        rospy.loginfo('[Yolo] Actual position error (Cm):    {:.2f}'.format(position_err_cm))
            
        # actual orientation error
        actual_ori_err = keyboard.theta
        rospy.loginfo('[Yolo] Actual orientation error: {:.2f}'.format(actual_ori_err))

        # orientation error actual vs simulation
        map_ori_err = np.absolute( keyboard.theta - simul_keyboard.theta)
        rospy.loginfo('[Yolo] Map orientation error: {:.2f}'.format(map_ori_err))

        if save_data:
            rospy.loginfo('[Yolo] Saving data..')

            status = input("OK (y/n)? ")
            if status == 'y':
                status = 'ok'
                good += 1
            elif status == 'n':
                status = 'fail'
                fail += 1

            rospy.loginfo('[Yolo] Total Good: {} Total Fail: {}'.format(good, fail))
            
            # save frame
            save_final_frame = True
            save_frame_pub.publish(True)
            excel.add_data(no=counter_finish, \
                            x_start_actual=keyboard_start_pose['x'], y_start_actual=keyboard_start_pose['y'], theta_start_actual=keyboard_start_pose['theta'], \
                            x_final_actual=keyboard.x, y_final_actual=keyboard.y, theta_final_actual=keyboard.theta, \
                            x_simula=simul_keyboard.x, y_simula=simul_keyboard.y, theta_simula=simul_keyboard.theta, \
                            pos_err_pixel=position_err_pixel, pos_err_cm=position_err_cm, theta_err=actual_ori_err,  map_theta_err=map_ori_err, status=status )

        reset_robot()

def final_pose_callback(msg):
    global simul_keyboard
    simul_keyboard = msg

# Subscriber
rospy.Subscriber("/pioneer/placement/finish_placement",     Bool,   finish_placement_callback)
rospy.Subscriber("/pioneer/placement/final_pose",           Pose2D, final_pose_callback)

def pub_point(topic, point):
    tar_pos   = Point32()
    tar_pos.x = point[0]
    tar_pos.y = point[1]
    topic.publish(tar_pos)

def load_config(arm):
    try:
        with open(ws_config_path, 'r') as f:
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

# load workspace
left_ws          = load_config('left_arm')
right_ws         = load_config('right_arm')
send_point       = False
rsync_send_point = False
l_point, r_point = None, None

def check_roi(arm, points, area):
    global l_point, r_point
    area = area.reshape((4,2))

    if arm == 'left_arm':
        x_min_b  = area[0][0]
        x_min_a  = area[1][0]
        x_max_a  = area[2][0]
        x_max_b  = area[3][0]
    elif arm == 'right_arm':
        x_max_b  = area[0][0]
        x_max_a  = area[1][0]
        x_min_a  = area[2][0]
        x_min_b  = area[3][0]

    y_frame_min = area[1][1]
    y_frame_max = area[0][1]
    x_frame_min = np.interp( points[1], [y_frame_min, y_frame_max], [x_min_a, x_min_b] )
    x_frame_min = int( np.round(x_frame_min, 0) )
    x_frame_max = np.interp( points[1], [y_frame_min, y_frame_max], [x_max_a, x_max_b] )
    x_frame_max = int( np.round(x_frame_max, 0) )

    # Y Check
    if points[1] >= y_frame_min and points[1] <= y_frame_max:
        # X Check
        if points[0] >= x_frame_min and points[0] <= x_frame_max:
            if arm == 'left_arm':
                l_point = (points[0], points[1])
            elif arm == 'right_arm':
                r_point = (points[0], points[1])
        else:
            l_point, r_point = None, None
    else:
        l_point, r_point = None, None

def reset_robot():
    global left_cnt, right_cnt, l_point, r_point
    ini_pose_pub.publish(True)
    left_cnt = right_cnt = 0
    l_point  = r_point   = None

left_cnt, right_cnt = 0, 0
save_interupt    = False
counter_interupt = 0
def mouse_event(event, x, y, flags, param):
    global left_cnt, right_cnt, send_point
    global l_point, r_point, save_interupt

    if event == cv2.EVENT_LBUTTONDOWN:
        if l_point and r_point:
            left_cnt += 1
            if left_cnt == 1:
                print()
                rospy.loginfo('[Yolo] Step 1: Send Keyboard Coordinate')
                send_point = True
            # elif left_cnt == 2:
            #     rospy.loginfo('[Yolo] Step 2: Grip Keyboard')
            #     grip_key_pub.publish(True)

    elif event == cv2.EVENT_LBUTTONUP:
        if left_cnt == 1:
            approach_keyboard_pub.publish(True)
            sleep(2)
            rospy.loginfo('[Yolo] Step 2: Grip Keyboard')
            grip_key_pub.publish(True)

    elif event == cv2.EVENT_RBUTTONDOWN:
        if left_cnt >= 1:
            rospy.loginfo('[Yolo] Step 3: Placement trajectory movement')
            placement_trigger_pub.publish(True)

    elif event == cv2.EVENT_MOUSEWHEEL:
        reset_robot()

    elif event == cv2.EVENT_MBUTTONDOWN:
        save_interupt = True
    

# load weights and set defaults
cfg_path      = yolo_config_path + 'yolov3.cfg'     #'config/yolov3.cfg'
weights_path  = yolo_config_path + 'yolov3.weights' #'config/yolov3.weights'
class_path    = yolo_config_path + 'coco.names'     #'config/coco.names'
img_size      = 416
conf_thres    = 0.8 # 0.8
nms_thres     = 0.4 # 0.4
 
# load model and put into eval mode
model  = Darknet(cfg_path, img_size=img_size)

try:
    model.load_weights(weights_path)
except:
    rospy.loginfo('[Yolo] Downloading yolov3.weights')
    os.system("cd ~/catkin_ws/src/PIONEER-ROBOT/pioneer_yolov3/config; ./download_weights.sh;")
    model.load_weights(weights_path)

device = torch.device('cuda:0' if torch.cuda.is_available() else 'cpu')
model.to(device) # auto select
model.eval()

classes = utils.load_classes(class_path)
Tensor  = torch.cuda.FloatTensor if torch.cuda.is_available() else torch.FloatTensor

def detect_image(img):
    # scale and pad image
    ratio = min(img_size/img.size[0], img_size/img.size[1])
    imw = round(img.size[0] * ratio)
    imh = round(img.size[1] * ratio)
    img_transforms = transforms.Compose([ transforms.Resize((imh, imw)),
         transforms.Pad((max(int((imh-imw)/2),0), max(int((imw-imh)/2),0), max(int((imh-imw)/2),0), max(int((imw-imh)/2),0)),
                        (128,128,128)),
         transforms.ToTensor(), ])
    # convert image to Tensor
    image_tensor = img_transforms(img).float()
    image_tensor = image_tensor.unsqueeze_(0)
    input_img = Variable(image_tensor.type(Tensor))
    # run inference on the model and get detections
    with torch.no_grad():
        detections = model(input_img)
        detections = utils.non_max_suppression(detections, 80, conf_thres, nms_thres)
    return detections[0]

colors       = [(255,0,0),(0,255,0),(0,0,255),(255,0,255),(128,0,0),(0,128,0),(0,0,128),(128,0,128),(128,128,0),(0,128,128)]
mot_tracker  = Sort() 
camera       = Camera()
frame        = camera.source_image.copy()
# frame_width  = frame.shape[0]
# frame_height = frame.shape[1]
frame_height = frame.shape[0]
frame_width  = frame.shape[1]

slope_deg       = 0
keyboard        = Pose2D()
show_keyb_frame = False
keyb_top_left, keyb_bottom_right = (-1, -1), (-1, -1)

cv2.namedWindow("frame")
cv2.setMouseCallback("frame", mouse_event)

def processing_keyboard(x1, y1, x2, y2, obj_id):
    global slope_deg, send_point, keyb_frame, save_start_frame
    global keyb_top_left, keyb_bottom_right

    box_h = int(((y2 - y1) / unpad_h) * img.shape[0])
    box_w = int(((x2 - x1) / unpad_w) * img.shape[1])
    y1    = int(((y1 - pad_y // 2) / unpad_h) * img.shape[0])
    x1    = int(((x1 - pad_x // 2) / unpad_w) * img.shape[1])
    color = colors[int(obj_id) % len(colors)]
    cls   = 'object'

    keyb_top_left     = (x1, y1)
    keyb_bottom_right = (x1+box_w, y1+box_h)

    cv2.rectangle(frame, (x1, y1), (x1+box_w, y1+box_h), color, 2)
    label = cls + " ({:.2f} deg)".format(slope_deg)
    cv2.putText(frame, label, (x1, y1 - 10), cv2.FONT_HERSHEY_TRIPLEX, 1, color, lineType=cv2.LINE_AA)
    keyb_frame  = src_frame[ y1:y1 + box_h, x1:x1 + box_w]

    if keyb_frame.size != 0:
        imgray      = cv2.cvtColor(keyb_frame,cv2.COLOR_BGR2GRAY)
        # ret,thresh  = cv2.threshold(imgray,127,255,0)
        ret,thresh  = cv2.threshold(imgray,100,255,0)
        thresh      = cv2.bitwise_not(thresh)
        # cv2.imshow('thresh', thresh)

        _,contours,_ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        max_contours = max(contours, key=cv2.contourArea)
        # hull         = cv2.convexHull(max_contours)
        hull         = max_contours
        # cv2.drawContours(keyb_frame, [hull], 0, (255,255,0), 2, offset=(0,0))
        rect         = cv2.minAreaRect(hull)
        box          = cv2.boxPoints(rect)
        box_d        = np.int0(box)

        if show_keyb_frame:
            cv2.drawContours(keyb_frame, [box_d], 0, (0,255,0), 1)

        # Sort 2D numpy array by 1nd Column (X)
        sorted_boxd = box_d[box_d[:,0].argsort()]
        left        = sorted_boxd[0:2]
        right       = sorted_boxd[2:4]

        P1 = left [ np.argmin( left [:,1]) ] + np.array([x1, y1])
        P3 = left [ np.argmax( left [:,1]) ] + np.array([x1, y1])
        P2 = right[ np.argmin( right[:,1]) ] + np.array([x1, y1])
        P4 = right[ np.argmax( right[:,1]) ] + np.array([x1, y1])

        cv2.circle(frame, tuple(P1), 5, (255,0,0),   -1) # B
        cv2.circle(frame, tuple(P2), 5, (0,255,0),   -1) # G
        cv2.circle(frame, tuple(P3), 5, (0,0,255),   -1) # R
        cv2.circle(frame, tuple(P4), 5, (0,255,255), -1) # Y

        left_mid  = ((P1 + np.array(P3)) // 2) #+ np.array([0, 25])
        right_mid = ((P2 + np.array(P4)) // 2) #+ np.array([0, 25])
        # cv2.line(frame,   tuple(left_mid),  tuple(right_mid), (255,255,255), 2)

        h_keyb, w_keyb  = thresh.shape
        keyb_mid        = w_keyb // 2
        offset_mid      = int( keyb_mid * 0.10 )
        
        xl_vline = keyb_mid-offset_mid 
        l_vline  = thresh[:,xl_vline] 
        yl_upper = np.where(l_vline==255)[0][0]
        xr_vline = keyb_mid+offset_mid 
        r_vline  = thresh[:,xr_vline] 
        yr_upper = np.where(r_vline==255)[0][0]

        m_upper  = (yr_upper-yl_upper) / (xr_vline-xl_vline)
        b_upper  = yl_upper - (m_upper*xl_vline)
        l_upper  = ( 0, int(b_upper) )
        r_upper  = ( w_keyb, int(m_upper * w_keyb + b_upper)  )

        if show_keyb_frame:
            cv2.line(keyb_frame, l_upper, r_upper, (255,255,255), 2)
            cv2.circle(keyb_frame, (xl_vline, yl_upper), 5, (255,0,0),   -1)
            cv2.circle(keyb_frame, (xr_vline, yr_upper), 5, (0,0,255),   -1)
        slope_deg = -1 * np.degrees(m_upper)#.astype(int)

        check_roi('left_arm',  left_mid,  left_ws)
        check_roi('right_arm', right_mid, right_ws)
    
    if l_point and r_point:
        cv2.circle(frame, l_point, 5, (255,0,0), -1) # Left arm point
        cv2.circle(frame, r_point, 5, (0,0,255), -1) # Right arm point

        keyboard.x     = (l_point[0]  + r_point[0]) // 2
        keyboard.y     = (l_point[1]  + r_point[1]) // 2
        keyboard.theta = slope_deg
        cv2.putText(frame, str(keyboard.x) + ", " + str(keyboard.y), (keyboard.x, keyboard.y-10), cv2.FONT_HERSHEY_TRIPLEX, 0.7, color, lineType=cv2.LINE_AA)
        cv2.circle(frame, (keyboard.x, keyboard.y), 10, (255,255,255), -1) # Middle keyboard point

        if send_point:
            keyboard_start_pose['x']     = keyboard.x
            keyboard_start_pose['y']     = keyboard.y
            keyboard_start_pose['theta'] = keyboard.theta
            save_start_frame = True

            pub_point(left_arm_pos_pub,  l_point)
            pub_point(right_arm_pos_pub, r_point)

            keyboard.y  = frame_height - keyboard.y
            keyboard_pos_pub.publish(keyboard)
            send_point = False

def myhook():
    shutdown_pub.publish(True)
    rospy.loginfo("[Yolo] Shutdown time")

while(True):
    frame      = camera.source_image.copy()
    src_frame  = frame.copy()
    frame      = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    pilimg     = Image.fromarray(frame)
    detections = detect_image(pilimg)

    frame      = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
    img        = np.array(pilimg)
    pad_x      = max(img.shape[0] - img.shape[1], 0) * (img_size / max(img.shape))
    pad_y      = max(img.shape[1] - img.shape[0], 0) * (img_size / max(img.shape))
    unpad_h    = img_size - pad_y
    unpad_w    = img_size - pad_x
    keyb_frame = np.zeros_like(frame)

    if detections is not None:
        tracked_objects = mot_tracker.update(detections.cpu())
        unique_labels   = detections[:, -1].cpu().unique()

        # for x1, y1, x2, y2, obj_id, cls_pred in tracked_objects:
        #     box_h = int(((y2 - y1) / unpad_h) * img.shape[0])
        #     box_w = int(((x2 - x1) / unpad_w) * img.shape[1])
        #     y1 = int(((y1 - pad_y // 2) / unpad_h) * img.shape[0])
        #     x1 = int(((x1 - pad_x // 2) / unpad_w) * img.shape[1])
        #     color = colors[int(obj_id) % len(colors)]
        #     cls = classes[int(cls_pred)]
        #     cv2.rectangle(frame, (x1, y1), (x1+box_w, y1+box_h), color, 4)
        #     cv2.rectangle(frame, (x1, y1-35), (x1+len(cls)*19+80, y1), color, -1)
        #     cv2.putText(frame, cls + "-" + str(int(obj_id)), (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 1, (255,255,255), 3)

        if  tracked_objects.size != 0:
            object_class = tracked_objects[:,5].astype(int)
            object_class = [ classes[i] for i in object_class ]

        keyboard_found = False
        keyb_top_left = keyb_bottom_right = (-1, -1)

        for x1, y1, x2, y2, obj_id, cls_pred in tracked_objects:
            cls = classes[int(cls_pred)]

            if cls == 'keyboard' and not keyboard_found:
                keyboard_found = True
                processing_keyboard(x1, y1, x2, y2, obj_id)
            elif  cls == 'laptop' and not keyboard_found:
                keyboard_found = True
                processing_keyboard(x1, y1, x2, y2, obj_id)

    # draw workspace
    cv2.polylines(frame,[left_ws],  True, (255,0,0), 2)
    cv2.polylines(frame,[right_ws], True, (0,0,255), 2)

    # # measuring distance mapping from pixel to cm
    # # x lines
    # cv2.line(frame, (120,353), (500,353), (255,255,255), 2) # 45cm
    # # y lines
    # cv2.line(frame, (318,180), (318,435), (255,255,255), 2) # 35cm
    # # ref point
    # cv2.circle(frame, (318, 353), 5, (255,255,255), -1)

    key = cv2.waitKey(1)
    cv2.imshow('frame', frame)

    if save_interupt:
        counter_interupt += 1
        cv2.imwrite(interupt_data_path + "-" + str(counter_interupt) + ".jpg", src_frame) # original
        print(interupt_data_path + "-" + str(counter_interupt) + ".jpg")
        save_interupt = False

    if save_data:
        if save_start_frame:
            counter_start += 1
            cv2.imwrite(data_path + "picts_actual_start/actual_start-" + str(counter_start) + ".jpg", frame)
            save_start_frame = False

        if save_final_frame:
            cv2.imwrite(data_path + "picts_actual_finish/actual_finish-" + str(counter_finish) + ".jpg", frame)
            # cv2.imwrite(data_path + "pictu    res/actual/ori-" + str(counter_finish) + ".jpg", src_frame) # original
            save_final_frame = False

    if show_keyb_frame:
        if keyb_frame.size != 0:
            cv2.imshow('keyboard', keyb_frame)

    if key == 27:  # esc
        break

rospy.on_shutdown(myhook)
camera.kill_threads()
cv2.destroyAllWindows()