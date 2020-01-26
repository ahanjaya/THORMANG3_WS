#!/usr/bin/env python3

from models import *
from utils import *
from sort import *

import os, sys, time, datetime, random
import torch, cv2
import rospkg
from torch.autograd import Variable
from torch.utils.data import DataLoader
from torchvision import datasets, transforms
from PIL import Image
from geometry_msgs.msg import Pose2D


class YoloV3:
    def __init__(self):
        rospack   = rospkg.RosPack()
        cfg_path  = rospack.get_path("pioneer_yolov3") + "/config/"
        self.data_path = rospack.get_path("pioneer_yolov3") + "/data/"

        # load weights and set defaults
        config_path  = cfg_path + 'yolov3.cfg'
        weights_path = cfg_path + 'yolov3.weights'
        class_path   = cfg_path + 'coco.names'
        self.img_size   = 416
        self.conf_thres = 0.8
        self.nms_thres  = 0.4

        # load self.model and put into eval mode
        self.model = Darknet(config_path, img_size=self.img_size)
        self.model.load_weights(weights_path)
        self.model.cuda()
        self.model.eval()

        self.classes = utils.load_classes(class_path)
        self.Tensor  = torch.cuda.FloatTensor

    def detect_image(self, img):
        # scale and pad image
        ratio = min(self.img_size/img.size[0], self.img_size/img.size[1])
        imw = round(img.size[0] * ratio)
        imh = round(img.size[1] * ratio)
        img_transforms = transforms.Compose([ transforms.Resize((imh, imw)),
            transforms.Pad((max(int((imh-imw)/2),0), max(int((imw-imh)/2),0), max(int((imh-imw)/2),0), max(int((imw-imh)/2),0)),
                            (128,128,128)), transforms.ToTensor(), ])
        # convert image to self.Tensor
        image_tensor = img_transforms(img).float()
        image_tensor = image_tensor.unsqueeze_(0)
        input_img = Variable(image_tensor.type(self.Tensor))
        # run inference on the self.model and get detections
        with torch.no_grad():
            detections = self.model(input_img)
            detections = utils.non_max_suppression(detections, 80, self.conf_thres, self.nms_thres)
        return detections[0]

    def run(self):
        colors       = [(255,0,0),(0,255,0),(0,0,255),(255,0,255),(128,0,0),(0,128,0),(0,0,128),(128,0,128),(128,128,0),(0,128,128)]
        # cap          = cv2.VideoCapture(0)
        # ret, frame   = cap.read()

        # frame        = cv2.imread(self.data_path + "keyboard_10.jpg")
        frame        = cv2.imread(self.data_path + "my_photo-2.jpg")
        frame_width  = frame.shape[1]
        frame_height = frame.shape[0]
        print ("Video size", frame_width,frame_height)

        mot_tracker  = Sort() 
        frames       = 0
        start_time   = time.time()

        while(True):
            src_frame  = frame.copy()
            frame      = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            pilimg     = Image.fromarray(frame)
            detections = self.detect_image(pilimg)

            frame   = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
            img     = np.array(pilimg)
            pad_x   = max(img.shape[0] - img.shape[1], 0) * (self.img_size / max(img.shape))
            pad_y   = max(img.shape[1] - img.shape[0], 0) * (self.img_size / max(img.shape))
            unpad_h = self.img_size - pad_y
            unpad_w = self.img_size - pad_x

            slope_deg = 0

            if detections is not None:
                tracked_objects = mot_tracker.update(detections.cpu())

                unique_labels = detections[:, -1].cpu().unique()
                n_cls_preds = len(unique_labels)
                for x1, y1, x2, y2, obj_id, cls_pred in tracked_objects:
                    box_h = int(((y2 - y1) / unpad_h) * img.shape[0])
                    box_w = int(((x2 - x1) / unpad_w) * img.shape[1])
                    y1 = int(((y1 - pad_y // 2) / unpad_h) * img.shape[0])
                    x1 = int(((x1 - pad_x // 2) / unpad_w) * img.shape[1])
                    color = colors[int(obj_id) % len(colors)]

                    cls = self.classes[int(cls_pred)]
                    if cls == 'keyboard':
                        # cv2.rectangle(frame, (x1, y1), (x1+box_w, y1+box_h), color, 4)
                        # cv2.rectangle(frame, (x1, y1-35), (x1+len(cls)*19+80, y1), color, -1)
                        # cv2.putText(frame, cls + "-" + str(int(obj_id)), (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 1, (255,255,255), 3)

                        # # keyb_top_left     = (x1, y1)
                        # # keyb_bottom_right = (x1+box_w, y1+box_h)

                        cv2.rectangle(frame, (x1, y1), (x1+box_w, y1+box_h), color, 2)
                        # label = cls + " ({:.2f} deg)".format(slope_deg)
                        # cv2.putText(frame, label, (x1, y1 - 10), cv2.FONT_HERSHEY_TRIPLEX, 1, color, lineType=cv2.LINE_AA)
                        keyb_frame  = src_frame[ y1:y1 + box_h, x1:x1 + box_w]

                        if keyb_frame.size != 0:
                            imgray      = cv2.cvtColor(keyb_frame,cv2.COLOR_BGR2GRAY)
                            # ret,thresh  = cv2.threshold(imgray,127,255,0)
                            ret,thresh  = cv2.threshold(imgray,100,255,0)
                            thresh      = cv2.bitwise_not(thresh)
                            cv2.imshow('thresh', thresh)

                            _,contours,_ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                            max_contours = max(contours, key=cv2.contourArea)
                            # hull         = cv2.convexHull(max_contours)
                            hull         = max_contours
                            # cv2.drawContours(keyb_frame, [hull], 0, (255,255,0), 2, offset=(0,0))
                            rect         = cv2.minAreaRect(hull)
                            box          = cv2.boxPoints(rect)
                            box_d        = np.int0(box)

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

                            P1 = left [ np.argmin( left [:,1]) ]
                            P3 = left [ np.argmax( left [:,1]) ]
                            P2 = right[ np.argmin( right[:,1]) ]
                            P4 = right[ np.argmax( right[:,1]) ]

                            cv2.circle(keyb_frame, tuple(P1), 5, (255,0,0),   -1) # B
                            cv2.circle(keyb_frame, tuple(P2), 5, (0,255,0),   -1) # G
                            cv2.circle(keyb_frame, tuple(P3), 5, (0,0,255),   -1) # R
                            cv2.circle(keyb_frame, tuple(P4), 5, (0,255,255), -1) # Y

                            left_mid  = ((P1 + np.array(P3)) // 2)
                            right_mid = ((P2 + np.array(P4)) // 2)

                            cv2.line(keyb_frame,   tuple(left_mid),  tuple(right_mid), (255,0,0), 2)
                            cv2.circle(keyb_frame, tuple(left_mid),  5, (255,255,255),   -1)
                            cv2.circle(keyb_frame, tuple(right_mid), 5, (255,255,255),   -1)


                            keyboard       = Pose2D()
                            keyboard.x     = (left_mid[0]  + right_mid[0]) // 2
                            keyboard.y     = (left_mid[1]  + right_mid[1]) // 2
                            keyboard.theta = slope_deg
                            cv2.putText(frame, str(keyboard.x) + ", " + str(keyboard.y), (keyboard.x, keyboard.y-10), cv2.FONT_HERSHEY_TRIPLEX, 0.7, color, lineType=cv2.LINE_AA)
                            cv2.circle(keyb_frame, (keyboard.x, keyboard.y), 10, (255,255,255), -1) # Middle keyboard point

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

                            slope_deg = -1 * np.degrees(m_upper)#.astype(int)
                            label = cls + " ({:.2f} deg)".format(slope_deg)
                            cv2.putText(frame, label, (x1, y1 - 10), cv2.FONT_HERSHEY_TRIPLEX, 1, color, lineType=cv2.LINE_AA)



            cv2.imshow('Stream',   frame)
            cv2.imshow('keyboard', keyb_frame)

            ch = 0xFF & cv2.waitKey(0)
            if ch == 27:
                break

        cv2.destroyAllWindows()

if __name__ == "__main__":
    yolo = YoloV3()
    yolo.run()
