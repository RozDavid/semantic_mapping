#!/usr/bin/env python
import sys
import argparse

import rospy
from cv_bridge import CvBridge, CvBridgeError
import message_filters
from sensor_msgs.msg import Image, CameraInfo
import numpy as np

import cv2
import matplotlib.pyplot as plt
import torch
import torch.nn.functional as F

from src.args import ArgumentParserRGBDSegmentation
from src.build_model import build_model
from src.prepare_data import prepare_data

class ESAInference(object):

    def __init__(self):

        # HYPEPARAMS
        self.image_encoding = '16SC1'
        self.depth_scale = 1.0
        self.ckpt_path = './trained_models/nyuv2/r34_NBt1D.pth'

        self.init_model()
        self.init_ros()

    def init_model(self):
        parser = ArgumentParserRGBDSegmentation(
            description='Efficient RGBD Indoor Sematic Segmentation (Inference)',
            formatter_class=argparse.ArgumentDefaultsHelpFormatter)
        parser.set_common_args()
        parser.add_argument('--ckpt_path', type=str, default=self.ckpt_path)
        parser.add_argument('--depth_scale', type=float,
                            default=self.depth_scale,
                            help='Additional depth scaling factor to apply.')
        self.args = parser.parse_args()
        self.args.pretrained_on_imagenet = False

        self.dataset, self.preprocessor = prepare_data(self.args, with_input_orig=True)
        n_classes = self.dataset.n_classes_without_void

        # model and checkpoint loading
        self.model, self.device = build_model(self.args, n_classes=n_classes)
        self.checkpoint = torch.load(self.args.ckpt_path,
                                     map_location=lambda storage, loc: storage)
        self.model.load_state_dict(self.checkpoint['state_dict'])
        print('Loaded checkpoint from {}'.format(self.args.ckpt_path))

        self.model.eval()
        self.model.to(self.device)

    def init_ros(self):

        rospy.init_node('esa_inference_node', anonymous=True)

        image_topic_name = rospy.get_param('esa_inference_node/image_topic_name',
                                           '/zedm/zed_node/rgb_raw/image_raw_color')
        depth_topic_name = rospy.get_param('esa_inference_node/depth_topic_name',
                                            '/zedm/zed_node/depth/depth_registered')
        semantic_topic_name = rospy.get_param('esa_inference_node/semantic_topic_name', '/esanet/semantic_image')

        self.bridge = CvBridge()
        self.image_pub = rospy.Publisher(semantic_topic_name, Image, queue_size=1)

        image_sub = message_filters.Subscriber(image_topic_name, Image, queue_size=10)
        depth_sub = message_filters.Subscriber(depth_topic_name, CameraInfo, queue_size=10)
        ts = message_filters.TimeSynchronizer([image_sub, depth_sub], 10)
        ts.registerCallback(self.inference_callback)

        rospy.loginfo('ESANet for inference with input images from: %s \nAnd depth from %s to topic %s' % (image_topic_name, depth_topic_name, semantic_topic_name))

        rospy.spin()

    def inference_callback(self, rgb_image, depth_image):

        try:

            img_rgb = self.bridge.imgmsg_to_cv2(rgb_image)
            img_depth = self.bridge.imgmsg_to_cv2(depth_image).astype('float32') * self.args.depth_scale
            h, w, _ = img_rgb.shape

            # preprocess sample
            sample = self.preprocessor({'image': img_rgb, 'depth': img_depth})

            # add batch axis and copy to device
            image = sample['image'][None].to(self.device)
            depth = sample['depth'][None].to(self.device)

            # apply network
            pred = self.model(image, depth)
            pred = F.interpolate(pred, (h, w),
                                 mode='bilinear', align_corners=False)
            pred = torch.argmax(pred, dim=1)
            pred = pred.cpu().numpy().squeeze().astype(np.uint8)

            # show result
            pred_colored = self.dataset.color_label(pred, with_void=False)

            image_message = self.bridge.cv2_to_imgmsg(pred_colored, self.image_encoding)
            image_message.header = rgb_image.header

            self.image_pub.publish(image_message)

        except CvBridgeError as e:
            print(e)


if __name__ == '__main__':

    ESAInference()
