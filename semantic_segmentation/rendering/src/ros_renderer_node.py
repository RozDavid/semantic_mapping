
import rospy
import cv2
import tf
from sensor_msgs.msg import Image
import message_filters
from cv_bridge import CvBridge, CvBridgeError
import yaml
import argparse
import numpy as np

from src.CameraConfig import CameraConfig
from src.meshrenderer import MeshRenderer
from utils import *


class RendererNode(object):

    def __init__(self):

        self.bridge = CvBridge()
        self.listener = tf.TransformListener()

        parser = argparse.ArgumentParser()
        self.args = self.parse_arguments(parser)
        self.sensor_frame = self.args.sensor_frame
        self.parent_frame = self.args.parent_frame
        self.image_topic = self.args.image_topic
        self.depth_topic = self.args.depth_topic
        self.rendered_topic = self.args.rendered_topic
        self.rendered_overlayed_topic = self.args.rendered_overlayed_topic
        self.rendered_depth_topic = self.args.rendered_depth_topic

        self.meshrenderer = MeshRenderer(self.args)

        image_sub = message_filters.Subscriber(self.image_topic, Image, queue_size=10)
        depth_sub = message_filters.Subscriber(self.depth_topic, Image, queue_size=10)
        ts = message_filters.ApproximateTimeSynchronizer([image_sub, depth_sub], 10, slop=0.1)
        ts.registerCallback(self.img_callback)

        self.rendered_image_pub = rospy.Publisher(self.rendered_topic, Image, queue_size=5)
        self.overlayed_image_pub = rospy.Publisher(self.rendered_overlayed_topic, Image, queue_size=5)
        self.rendered_depth_pub = rospy.Publisher(self.rendered_depth_topic, Image, queue_size=5)

    def img_callback(self, img_msg, depth_msg):

        try:

            cv_image = self.bridge.imgmsg_to_cv2(img_msg, "bgr8")
            cv_depth = self.bridge.imgmsg_to_cv2(img_msg, "passthrough")

        except CvBridgeError as e:
            print("CvBridge parse exception: ", e)
            return

        try:
            self.listener.waitForTransform(self.parent_frame,
                                           self.sensor_frame,
                                           img_msg.header.stamp, rospy.Duration(10))
            (trans, rot) = self.listener.lookupTransform(self.parent_frame,
                                                         self.sensor_frame,
                                                         img_msg.header.stamp)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            print("Transform exception: ", e)
            return

        rendered_img, rendered_depth = self.meshrenderer.render_ros_transform(trans, rot, img_msg.header.stamp.secs, depth=True)

        try:
            rendered_msg = self.bridge.cv2_to_imgmsg(rendered_img, "bgr8")
            rendered_msg.header = img_msg.header
            self.rendered_image_pub.publish(rendered_msg)

            rendered_depth_msg = self.bridge.cv2_to_imgmsg(rendered_depth)
            rendered_depth_msg.header = img_msg.header
            self.rendered_depth_pub.publish(rendered_depth_msg)

            if self.args.visualization:

                overlay  = cv2.addWeighted(rendered_img, 0.5, cv_image, 0.5, 0)
                rendered_overlayed_msg = self.bridge.cv2_to_imgmsg(overlay)
                rendered_overlayed_msg.header = img_msg.header
                self.overlayed_image_pub.publish(rendered_overlayed_msg)


        except CvBridgeError as e:
            print("CvBridge publish exception: ", e)


    def parse_arguments(self, parser):

        # Parse arguments for global params
        parser.add_argument('--mesh_path', type=str, default='../meshes/living_room_full.ply',
                            help='The semantic or RGB mesh, that provides the labels for the views')
        parser.add_argument('--recordings_path', type=str, default="./data",
                            help='Under this directory should be stored images and and poses'
                                 ' in the \'imgs\' and \'poses\' folders accordingly')
        parser.add_argument('--camera_config', type=str, default='../config/calib_k4a.yml',
                            help='The camera parameters of the virtual camera that renders the image')
        parser.add_argument("--visualization", default=True, type=str2bool, nargs='?',
                            help="If we would like to open a viewer with the loaded model first")
        parser.add_argument('--sensor_frame', type=str, default='hp_laptop_camera_base',
                            help='The frame of the robot sensor, where the camera pose is tracked')
        parser.add_argument('--parent_frame', type=str, default="hp_laptop_odometry_frame",
                            help='The world or the odometry frame, where we get the sensor frame. '
                                 'Should be aligned to the mesh')
        parser.add_argument('--image_topic', type=str, default="rgb/image_raw",
                            help='Where the streaming RGB images are received over ROS')
        parser.add_argument('--depth_topic', type=str, default="depth_to_rgb/image_raw",
                            help='The topic where the robot depth images are published')
        parser.add_argument('--rendered_topic', type=str, default="semantics/rendered",
                            help='We are going to publish the rendered images over this topic')
        parser.add_argument('--rendered_overlayed_topic', type=str, default="semantics/rendered_overlayed",
                            help='We are going to publish the rendered stacked on the RGB images over this topic')
        parser.add_argument('--rendered_depth_topic', type=str, default="semantics/rendered_depth",
                            help='We are going to publish the rendered depth images over this topic')

        args = parser.parse_args()

        return args


if __name__ == '__main__':

    rospy.init_node("mesh_renderer_node")
    node_name = rospy.get_name()

    rospy.loginfo("%s started" % node_name)
    print("Started", node_name)

    render_node = RendererNode()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down", node_name)

