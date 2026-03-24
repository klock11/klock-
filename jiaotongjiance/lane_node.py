#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
import cv2
import torch
import numpy as np
from sensor_msgs.msg import Image
from std_msgs.msg import Float32
from cv_bridge import CvBridge
import os
os.environ['PYTORCH_CUDA_ALLOC_CONF'] = 'max_split_size_mb:64'

class LaneDetector:
    def __init__(self):
        rospy.init_node('lane_detector_node')
        self.bridge = CvBridge()
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")

        # 加载模型 (请确保模型类定义已导入或使用 torch.jit)
        self.model = torch.load('lane_model.pth').to(self.device)
        self.model.eval()

        self.image_sub = rospy.Subscriber("/usb_cam/image_raw", Image, self.callback)
        self.lane_pub = rospy.Publisher("/lane_offset", Float32, queue_size=1)

        # 标定参数：图像中心横坐标，以及像素到米的比例
        self.img_width = 640
        self.pixel_to_meter = 0.0015  # 示例值：1像素=1.5毫米，需根据实际安装高度调整

    def callback(self, data):
        try:
            cv_img = self.bridge.imgmsg_to_cv2(data, "bgr8")
            input_img = cv2.resize(cv_img, (224, 224))  # 假设模型输入224x224

            # 预处理
            img_tensor = input_img.transpose(2, 0, 1).astype('float32') / 255.0
            img_tensor = torch.from_numpy(img_tensor).unsqueeze(0).to(self.device)

            with torch.no_grad():
                output = self.model(img_tensor)  # 假设输出为 [1, 1, 224, 224]
                mask = torch.sigmoid(output).squeeze().cpu().numpy()

            # 二值化提取线条
            _, binary_lane = cv2.threshold(mask, 0.5, 255, cv2.THRESH_BINARY)

            # 提取图像下部 1/3 区域的平均中点（关注近处车道）
            h, w = binary_lane.shape
            roi = binary_lane[int(h * 2 / 3):, :]
            M = cv2.moments(roi)

            if M["m00"] != 0:
                lane_x = int(M["m10"] / M["m00"])  # 计算重心 X
                # 计算相对于图像中心线的偏差 (单位：米)
                offset_pixels = lane_x - (w / 2)
                offset_meters = offset_pixels * self.pixel_to_meter
                self.lane_pub.publish(offset_meters)
            else:
                # 没检测到线时可以发一个特殊值或保持上一帧
                rospy.logwarn("Lane lost!")
        except Exception as e:
            rospy.logerr(e)


if __name__ == '__main__':
    LaneDetector()
    rospy.spin()