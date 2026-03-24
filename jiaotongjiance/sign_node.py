#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
import cv2
import torch
from sensor_msgs.msg import Image
from std_msgs.msg import Int32
from cv_bridge import CvBridge
import os
os.environ['PYTORCH_CUDA_ALLOC_CONF'] = 'max_split_size_mb:64'

class SignDetector:
    def __init__(self):
        rospy.init_node('sign_detector_node')
        self.bridge = CvBridge()
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        self.model = torch.load('sign_model.pth').to(self.device)
        self.model.eval()

        self.image_sub = rospy.Subscriber("/usb_cam/image_raw", Image, self.callback)
        self.sign_pub = rospy.Publisher("/traffic_sign", Int32, queue_size=1)

    def callback(self, data):
        cv_img = self.bridge.imgmsg_to_cv2(data, "bgr8")
        # 预处理与推理
        img_input = cv2.resize(cv_img, (64, 64))  # 假设输入64x64
        img_tensor = torch.from_numpy(img_input.transpose(2, 0, 1)).float().unsqueeze(0).to(self.device)

        with torch.no_grad():
            output = self.model(img_tensor)
            _, pred = torch.max(output, 1)
            self.sign_pub.publish(int(pred.item()))


if __name__ == '__main__':
    SignDetector()
    rospy.spin()