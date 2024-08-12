#!/usr/bin/env python
import rclpy
from rclpy.node import Node
import cv2
import numpy as np
import torch
import os
import std_msgs
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray
from vision_msgs.msg import Detection2D
from cv_bridge import CvBridge
from yolo_ros2.tool import handle_preds
from yolo_ros2.module.detector import Detector
from rclpy.qos import HistoryPolicy, QoSProfile

target_classes = ["person"]


path_curr = os.path.dirname(__file__)
img_topic = "/camera/color/image_raw"
device = "cuda:0"
weight = "weights/weight_AP05:0.253207_280-epoch.pth"
class_names = "config/coco.names"
thresh = 0.65

class yolo_detector(Node):

    def __init__(self):
        super().__init__("yolo_detector_node")
        print("[onboardDetector]: yolo detector init...")

        self.img_received = False
        self.img_detected = False


        # init and load
        self.model = Detector(80, True).to(device)
        self.model.load_state_dict(torch.load(os.path.join(path_curr, weight), map_location=device))
        self.model.eval()

        # subscriber
        self.br = CvBridge()
        qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=10)
        self.img_sub = self.create_subscription(Image,img_topic,self.image_callback,qos_profile=qos)
        # publisher
        self.img_pub = self.create_publisher(Image,"yolo_detector/detected_image",  10)
        self.bbox_pub = self.create_publisher(Detection2DArray,"yolo_detector/detected_bounding_boxes",  10)
        self.time_pub = self.create_publisher(std_msgs.msg.Float32,"yolo_detector/yolo_time",  1)

        # timer
        #self.create_timer(0.033, self.detect_callback)
        #self.create_timer(0.033, self.vis_callback)
        #self.create_timer(0.033, self.bbox_callback)
    
    def image_callback(self, msg):
        self.img= self.br.imgmsg_to_cv2(msg, "bgr8")
        self.detect_callback()
        self.vis_callback()
        self.bbox_callback()
    

    def detect_callback(self):
        startTime = self.get_clock().now().nanoseconds
        output = self.inference(self.img)
        self.detected_img, self.detected_bboxes = self.postprocess(self.img, output)
        self.img_detected = True
        endTime = self.get_clock().now().nanoseconds
        time_msg = std_msgs.msg.Float32()
        time_msg.data = float(endTime - startTime)
        self.time_pub.publish(time_msg)
        
        

    def vis_callback(self):
        if (self.img_detected == True):
            self.img_pub.publish(self.br.cv2_to_imgmsg(self.detected_img, "bgr8"))

    def bbox_callback(self):
        if (self.img_detected == True):
            bboxes_msg = Detection2DArray()
            for detected_box in self.detected_bboxes:
                if (detected_box[4] in target_classes):
                    bbox_msg = Detection2D()
                    bbox_msg.bbox.center.position.x = float((detected_box[0]+detected_box[2])/2)
                    bbox_msg.bbox.center.position.y = float((detected_box[1]+detected_box[3])/2)
                    bbox_msg.bbox.size_x = float(abs(detected_box[2] - detected_box[0]))
                    bbox_msg.bbox.size_y = float(abs(detected_box[3] - detected_box[1]))

                    bboxes_msg.detections.append(bbox_msg)
                bboxes_msg.header.stamp = self.get_clock().now().to_msg()
            self.bbox_pub.publish(bboxes_msg)

    def inference(self, ori_img):
        # image pre-processing
        res_img = cv2.resize(ori_img, (352, 352), interpolation = cv2.INTER_LINEAR) 
        img = res_img.reshape(1, 352, 352, 3)
        img = torch.from_numpy(img.transpose(0, 3, 1, 2))
        img = img.to(device).float() / 255.0    

        # inference
        preds = self.model(img)
        output = handle_preds(preds, device, thresh)
        return output

    def postprocess(self, ori_img, output):
        LABEL_NAMES = []
        with open(os.path.join(path_curr, class_names), 'r') as f:
            for line in f.readlines():
                LABEL_NAMES.append(line.strip())
        
        H, W, _ = ori_img.shape
        scale_h, scale_w = H / 352., W / 352.

        detected_boxes = []
        for box in output[0]:
            box = box.tolist()
           
            obj_score = box[4]
            category = LABEL_NAMES[int(box[5])]
            if category in target_classes:
                x1, y1 = int(box[0] * W), int(box[1] * H)
                x2, y2 = int(box[2] * W), int(box[3] * H)
                detected_box = [x1, y1, x2, y2, category]
                detected_boxes.append(detected_box)

                cv2.rectangle(ori_img, (x1, y1), (x2, y2), (255, 255, 0), 2)
                cv2.putText(ori_img, '%.2f' % obj_score, (x1, y1 - 5), 0, 0.7, (0, 255, 0), 2)  
                cv2.putText(ori_img, category, (x1, y1 - 25), 0, 0.7, (0, 255, 0), 2)
        return ori_img, detected_boxes
    