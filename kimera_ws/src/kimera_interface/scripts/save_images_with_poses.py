#!/usr/bin/env python

import rospy
import cv2
import tf
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import yaml

class ImagePoseRecorder:

    def __init__(self):

        self.bridge = CvBridge()
        self.listener = tf.TransformListener()

        # Reading the params
        self.image_topic = rospy.get_param("~image_topic", "/rgb/image_raw")
        self.image_encoding = rospy.get_param("~image_encoding", "bgr8")
        self.sensor_frame = rospy.get_param("~sensor_frame", "hp_laptop_link")
        self.parent_frame = rospy.get_param("~parent_frame", "hp_laptop_odometry_frame")
        self.dataset_path = rospy.get_param("~dataset_path", "../records/")
        self.throttle_msgs = rospy.get_param("~throttle_msgs", 5)

        self.image_sub = rospy.Subscriber(self.image_topic, Image, self.img_callback)

        self.throttle_iterator = 0

    def img_callback(self, data):

        img_msg = Image

        if float(self.throttle_iterator) / float(self.throttle_msgs) < 1.0:
            self.throttle_iterator += 1.0
            return

        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")

        except CvBridgeError as e:
            print(e)
            return

        try:
            (trans, rot) = self.listener.lookupTransform(self.sensor_frame, self.parent_frame,
                                                         rospy.Time(img_msg.header.stamp))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            print("Transform exception: ", e)
            return

        image_name = self.dataset_path + 'imgs/' + str(img_msg.header.stamp.secs) + str(
            img_msg.header.stamp.nsecs) + '.png'
        pose_name = self.dataset_path + 'imgs/' + str(img_msg.header.stamp.secs) + str(
            img_msg.header.stamp.nsecs) + '.yml'

        pose = {'trans': trans, 'rot': rot}
        with open(pose_name, 'w') as outfile:
            yaml.dump(pose, outfile, default_flow_style=False)

        cv2.imwrite(image_name, cv_image)

        print("Saved image with pose at {} to file". format(img_msg.header.stamp.secs))

if __name__ == '__main__':
    rospy.init_node("image_pose_recorder_node")
    node_name = rospy.get_name()

    rospy.loginfo("%s started" % node_name)
    print("%s started", node_name)

    recorder = ImagePoseRecorder()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
