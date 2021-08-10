#!/usr/bin/env python3
import argparse

import rospy
from cv_bridge import CvBridge, CvBridgeError
import message_filters
from sensor_msgs.msg import Image
import numpy as np

import cv2
import torch
import torch.nn.functional as F

from src.args import ArgumentParserRGBDSegmentation
from src.build_model import build_model
from src.prepare_data import prepare_data
from utils.color_segmentation import ColorSegmenter

class ESAInference(object):

    def __init__(self):

        parser = ArgumentParserRGBDSegmentation(
            description='Efficient RGBD Indoor Sematic Segmentation (Inference)',
            formatter_class=argparse.ArgumentDefaultsHelpFormatter)
        parser.set_common_args()

        parser.add_argument("--segmentation_mode", type=int, default=0,
                            help='Choose what to use from the implemented modes: 0 - Semantic Segmentation and Color;'
                                 '1 - Only Semantic segmentation; 2 - only color segmentation')

        parser.add_argument('--ckpt_path', type=str, default='./trained_models/sunrgbd/r34_NBt1D.pth',
                            help='The (relative) path where the trained model checkpoint is saved')
        parser.add_argument('--depth_scale', type=float,
                            default=1.,
                            help='Additional depth scaling factor to apply.')

        parser.add_argument("--image_topic_name", default="/hp_laptop/color/image_color",
                            help="The streaming RGB image topic")
        parser.add_argument("--depth_topic_name", default="/hp_laptop/aligned_depth_to_color/image_raw",
                            help="The streaming depth image topic")
        parser.add_argument("--cv_show_image", default=False, type=parser.str2bool, nargs='?',
                            help="If we want to open an CV image windows for the semantic images (high load)")

        parser.add_argument("--use_prediction_confidence_filtering",  default=False, type=parser.str2bool, nargs='?',
                            help="Mask images with black labels if prediction confidence is bellow a given threshold")
        parser.add_argument("--prediction_confidence_treshold", type=float,
                            default=2.,
                            help="Threshold value for masking unsure predictions and replacing "
                                 "with void label and black color")

        parser.add_argument("--color_segmentation_label_path", default='./config/eit_clc_segmentation_classes.csv',
                            help="Filepath for the CSv, where we store the target colors,"
                                 " labels and HSV tresholds for different classes")

        parser.add_argument("--semantic_topic_name", default='/semantics/semantic_image',
                            help="The output topic name for the semantically segmented image")


        self.args = parser.parse_args()
        self.args.pretrained_on_imagenet = False

        # Setup semantic/color segmentation mode
        mode = self.args.segmentation_mode
        if mode == 0:
            self.use_color_segmentation = True
            self.use_semantic_segmentation = True
        elif mode == 1:
            self.use_color_segmentation = False
            self.use_semantic_segmentation = True
        elif mode == 2:
            self.use_color_segmentation = True
            self.use_semantic_segmentation = False
        else:
            raise Exception("Unsupported segmentation mode:", mode, " should be in [0-2]")

        self.cv_show_image = self.args.cv_show_image

        if self.use_semantic_segmentation:
            self.depth_scale = self.args.depth_scale
            self.init_model(self.args.ckpt_path)

        if self.use_color_segmentation:
            self.color_labels_filepath = self.args.color_segmentation_label_path
            self.colorSegmenter = ColorSegmenter(self.color_labels_filepath)

        # Publisher and ROS params
        self.semantic_topic_name = self.args.semantic_topic_name
        self.init_ros(self.args.image_topic_name,
                         self.args.depth_topic_name)

    def init_model(self, ckpt_path):

        self.ckpt_path = ckpt_path
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

    def init_ros(self, image_topic_name, depth_topic_name):

        rospy.init_node('esa_inference_node', anonymous=False)

        # Subscriber params
        self.image_topic_name = image_topic_name
        self.depth_topic_name = depth_topic_name

        image_topic_name = rospy.get_param('esa_inference_node/image_topic_name', self.image_topic_name)
        depth_topic_name = rospy.get_param('esa_inference_node/depth_topic_name', self.depth_topic_name)

        semantic_topic_name = rospy.get_param('esa_inference_node/semantic_topic_name', self.semantic_topic_name)

        self.bridge = CvBridge()
        self.semantic_image_pub = rospy.Publisher(semantic_topic_name, Image, queue_size=1)


        image_sub = message_filters.Subscriber(image_topic_name, Image, queue_size=150)
        depth_sub = message_filters.Subscriber(depth_topic_name, Image, queue_size=150)

        ts = message_filters.ApproximateTimeSynchronizer([image_sub, depth_sub], 150, slop=0.1)

        ts.registerCallback(self.inference_callback)

        rospy.loginfo('ESANet for inference with input images from: %s \n'
                      'And depth from %s to topic %s' %
                      (image_topic_name, depth_topic_name, semantic_topic_name))

        rospy.spin()

    def apply_color_segmentation(self, orig_img, semantic_prediction, is_rgb=False):

        color_result = self.colorSegmenter.mask_image(orig_img, is_rgb=is_rgb)
        color_mask = cv2.cvtColor(color_result, cv2.COLOR_BGR2GRAY)
        semantic_prediction[color_mask > 0] = color_result[color_mask > 0]

        return semantic_prediction


    def model_inference(self, img_rgb, img_depth):

        ### Preprocessing
        h, w, c = img_rgb.shape

        #img_depth *= 6550. / np.max(img_depth)

        # preprocess sample
        sample = self.preprocessor({'image': img_rgb, 'depth': img_depth})

        # add batch axis and copy to device
        image = sample['image'][None].to(self.device)
        depth = sample['depth'][None].to(self.device)

        # Inference on model
        pred = self.model(image, depth)
        pred = F.interpolate(pred, (h, w),
                             mode='bilinear', align_corners=False)
        # pred_softmax = pred.cpu().detach().numpy().squeeze()
        pred = torch.argmax(pred, dim=1)
        pred = pred.cpu().numpy().squeeze().astype(np.uint8)

        ### Prediction Filtering
        if self.args.use_prediction_confidence_filtering:
            bellow_confidence_inds = pred < self.args.prediction_confidence_treshold
            pred[bellow_confidence_inds] = 0

        # Color the semantic masks
        pred_colored = self.dataset.color_label(pred, with_void=False)

        return pred_colored


    def inference_callback(self, rgb_msg, depth_msg):

        try:
            ### Convert ROS msg to CV2 msg type
            img_rgb = self.bridge.imgmsg_to_cv2(rgb_msg, desired_encoding='rgb8')
            img_depth = self.bridge.imgmsg_to_cv2(depth_msg).astype(np.float)

            if self.use_semantic_segmentation and self.depth_scale != 1.:
                img_depth = img_depth * self.depth_scale

            # Apply semantic segmentation if requested
            if self.use_semantic_segmentation:
                pred_colored = self.model_inference(img_rgb, img_depth)
            else:
                pred_colored = np.zeros(img_rgb.shape, np.uint8)

            # Apply color segmentation if requested
            if self.use_color_segmentation:
                pred_colored = self.apply_color_segmentation(img_rgb, pred_colored)

            ### Visualization and publishing
            if self.cv_show_image:
                dst = cv2.addWeighted(pred_colored, 0.5, img_rgb, 0.5, 0)
                self.show_image(dst)

            semantic_image_msg = self.bridge.cv2_to_imgmsg(pred_colored, encoding='rgb8')
            semantic_image_msg.header = depth_msg.header

            self.semantic_image_pub.publish(semantic_image_msg)

        except CvBridgeError as e:
            print(e)

        except Exception as exc:
            print(exc)

    def show_image(self, img):
        cv2.imshow("Prediction image", cv2.cvtColor(img, cv2.COLOR_RGB2BGR))
        cv2.waitKey(3)


if __name__ == '__main__':
    ESAInference()
