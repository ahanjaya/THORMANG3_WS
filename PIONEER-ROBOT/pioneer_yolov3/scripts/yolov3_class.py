#!/usr/bin/env python3

import sys
import rospy
import rospkg
from pioneer_vision.camera import Camera
from std_msgs.msg import Bool, Int32MultiArray

from models import *
from utils import *
from sort import *

import os, sys, time, datetime, random
import torch, cv2
import numpy as np
from torch.autograd import Variable
from torch.utils.data import DataLoader
from torchvision import datasets, transforms
from PIL import Image

class YoloV3:
    def __init__(self):
        rospack  = rospkg.RosPack()
        cfg_path = rospack.get_path("pioneer_yolov3") + "/config/"

        rospy.init_node('pioneer_yolov3', anonymous=False)
        self.shutdown_pub       = rospy.Publisher("/pioneer/shutdown_signal",             Bool,    queue_size=1)

        # load weights and set defaults
        config_path  = cfg_path + 'yolov3.cfg'
        weights_path = cfg_path + 'yolov3.weights'
        class_path   = cfg_path + 'coco.names'
        self.img_size   = 416
        self.conf_thres = 0.8 # 0.8
        self.nms_thres  = 0.4 # 0.4

        # load self.model and put into eval mode
        self.model = Darknet(config_path, img_size=self.img_size)
        try:
            self.model.load_weights(weights_path)
        except:
            rospy.loginfo('[Yolo] Downloading yolov3.weights')
            os.system("cd " + cfg_path + "; sh download_weights.sh;")
            self.model.load_weights(weights_path)

        device = torch.device('cuda:0' if torch.cuda.is_available() else 'cpu')
        self.model.to(device) # auto select
        self.model.eval()
        rospy.loginfo('[Yolo] Device: {}'.format(device))

        self.classes = utils.load_classes(class_path)
        self.Tensor  = torch.cuda.FloatTensor

    def mouse_event(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            rospy.loginfo('left clicked')

    def myhook(self):
        self.shutdown_pub.publish(True)
        rospy.loginfo("[Yolo] Shutdown time")

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
        camera       = Camera()
        frame        = camera.source_image.copy()
        frame_width  = frame.shape[0]
        frame_height = frame.shape[1]
        rospy.loginfo("[Yolo] Video size: {}, {}".format(frame_width,frame_height) )

        mot_tracker  = Sort() 
        frames       = 0
        start_time   = time.time()

        cv2.namedWindow("Stream")
        cv2.setMouseCallback("Stream", self.mouse_event)

        while(True):
            frame = camera.source_image.copy()

            frames     += 1
            frame      = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            pilimg     = Image.fromarray(frame)
            detections = self.detect_image(pilimg)

            frame   = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
            img     = np.array(pilimg)
            pad_x   = max(img.shape[0] - img.shape[1], 0) * (self.img_size / max(img.shape))
            pad_y   = max(img.shape[1] - img.shape[0], 0) * (self.img_size / max(img.shape))
            unpad_h = self.img_size - pad_y
            unpad_w = self.img_size - pad_x

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
                    cv2.rectangle(frame, (x1, y1), (x1+box_w, y1+box_h), color, 4)
                    cv2.rectangle(frame, (x1, y1-35), (x1+len(cls)*19+80, y1), color, -1)
                    cv2.putText(frame, cls + "-" + str(int(obj_id)), (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 1, (255,255,255), 3)

            cv2.imshow('Stream', frame)
            ch = 0xFF & cv2.waitKey(1)
            if ch == 27:
                break

        total_time = time.time()-start_time
        rospy.loginfo("[Yolo] {} frames {:.4f} s/frame".format(frames, total_time/frames))
        rospy.on_shutdown(self.myhook)
        cv2.destroyAllWindows()
        camera.kill_threads()

if __name__ == "__main__":
    yolo = YoloV3()
    yolo.run()