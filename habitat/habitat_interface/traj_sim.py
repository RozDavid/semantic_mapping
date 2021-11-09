import math
import os
import random
import sys
import argparse
import time

import git
import imageio
import magnum as mn
import numpy as np
import cv2
import pickle
import copy
import os.path

from matplotlib import pyplot as plt
from skimage.filters import gaussian

# function to display the topdown map
from PIL import Image

import habitat_sim
from habitat_sim.utils import common as utils
from habitat_sim.utils import viz_utils as vut
from tkinter import * # Import tkinter

import rosbag
from cv_bridge import CvBridge
from sensor_msgs.msg import  Image as rosImage
from sensor_msgs.msg import  CameraInfo
from sensor_msgs.msg import CompressedImage
import geometry_msgs.msg as rosgeo
import rospy
import tf2_msgs.msg as tf2msg
import tf2_ros

from utils import *
from config import *
from ros import *

class InteractiveSimulator(object):

    def __init__(self):
        parser = argparse.ArgumentParser()
        self.args = self.parse_arguments(parser)

        self.init_simulator(self.args)
        self.init_ros(self.args)

        self.agent_pose = []
        self.camera_pose = []

        self.run_simulation(compressed=self.compressed)

    def parse_arguments(self, parser):

        # Parse arguments for global params
        parser.add_argument('--mesh_path', type=str, default='/media/cuda/ssd/semantic_mapping/Replica-Dataset/dataset/frl_apartment_4/habitat/mesh_semantic.ply',
                            help='The Replica mesh path mesh, that provides the model for simulation')
        parser.add_argument('--camera_config', type=str, default='../config/calib_k4a.yml',
                            help='The camera parameters of the virtual camera that simulates the image')
        parser.add_argument('--robot_frame', type=str, default='habitat_robot_base',
                            help='The frame of the robot sensor, where the camera pose is tracked')
        parser.add_argument('--parent_frame', type=str, default="habitat_odometry_frame",
                            help='The world or the odometry frame, where we get the sensor frame. '
                                 'Should be aligned to the mesh')
        parser.add_argument('--image_topic_name', type=str, default='/habitat/rgb/image_raw',
                            help='The images will be savd under this topic')
        parser.add_argument('--depth_topic_name', type=str, default="/habitat/depth/image_raw",
                            help='The depth images will be saved under this topic')
        parser.add_argument('--semantic_topic_name', type=str, default='/habitat/semantics/image_raw',
                            help='The semantic imaes will be saved under this topic')
        
        parser.add_argument('--compressed_image_topic_name', type=str, default='/habitat/rgb/image_raw/compressed',
                            help='The compressed images will be savd under this topic')
        parser.add_argument('--compressed_depth_topic_name', type=str, default="/habitat/depth/image_raw/compressedDepth",
                            help='The compressed depth images will be saved under this topic')
        parser.add_argument('--compressed_semantic_topic_name', type=str, default='/habitat/semantics/image_raw/compressed',
                            help='The compressed semantic images will be saved under this topic')

        parser.add_argument('--output_bag_name', type=str, default="../data/output.bag",
                            help='The name and relative path of the output bag file')
        parser.add_argument('--output_agent_pose_name', type=str, default="../data/agent_states.npy",
                            help='The name and relative path of the output agent pose file')
        parser.add_argument('--target_fps', type=int, default=5,
                            help='The number of frames to render per second')
        parser.add_argument('--compressed', type=bool, default=False,
                            help='To compress images saved to rosbag')
        parser.add_argument('--replay_mode', type=bool, default=False,
                            help='To replay recorded trajectory from numpy array of poses')
        parser.add_argument('--gaussian_sigma', type=int, default=0.5,
                            help='Sigma of the Gaussian blur')
        parser.add_argument('--motion_blur_weight', type=int, default=0.1,
                            help='Weighting of the motion blur')

        return parser.parse_args()

    def init_simulator(self, args):

        random.seed(sim_settings["seed"])

        self.cfg = make_cfg(sim_settings, args.mesh_path)

        self.sim = habitat_sim.Simulator(self.cfg)
        self.sim.seed(sim_settings["seed"])
        self.sim.config.sim_cfg.allow_sliding = True

        # set new initial state
        self.sim.initialize_agent(agent_id=0)
        self.agent = self.sim.agents[0]
        
        ##
        self.replay_mode = args.replay_mode
        self.target_fps = args.target_fps

        self.instance_id_to_name = self._generate_label_map(self.sim.semantic_scene)
        self.object_name_dict = self._generate_object_dict(self.sim.semantic_scene)
        self.map_to_class_labels = np.vectorize(
            lambda x: self.object_name_dict.get(self.instance_id_to_name.get(x, 0), CLASS_DICT["objects"])
        )

        self.gaussian_sigma = args.gaussian_sigma
        self.motion_blur_weight = args.motion_blur_weight
        self.norm_factor = self.normalize(self.motion_blur_weight)
        self.prev_rgb = []
        self.prev_depth = []
        self.prev_semantic = []

    def init_ros(self, args):

        self.compressed = args.compressed
        self.output_agent_pose_name = args.output_agent_pose_name
        self.bag = rosbag.Bag(args.output_bag_name, 'w')
        self.bridge = CvBridge()
        self.image_topic_name = args.image_topic_name
        self.compressed_image_topic_name = args.compressed_image_topic_name
        self.image_info_topic_name = self.image_topic_name.rsplit('/', 1)[0] + '/camera_info'
        self.depth_topic_name = args.depth_topic_name
        self.compressed_depth_topic_name = args.compressed_depth_topic_name
        self.depth_info_topic_name = self.depth_topic_name.rsplit('/', 1)[0] + '/camera_info'
        self.semantic_topic_name = args.semantic_topic_name
        self.compressed_semantic_topic_name = args.compressed_semantic_topic_name
        self.semantic_info_topic_name = self.semantic_topic_name.rsplit('/', 1)[0] + '/camera_info'
        self.world_frame =  args.parent_frame
        self.robot_frame = args.robot_frame

        hfov = float(self.agent.agent_config.sensor_specifications[0].hfov) * np.pi / 180.
        self.height = self.agent.agent_config.sensor_specifications[0].resolution[0]
        self.width = self.agent.agent_config.sensor_specifications[0].resolution[1]
        focal_length = (self.width / 2) / np.tan(hfov / 2.0)

        camera_info_msg = CameraInfo()
        camera_info_msg.header.frame_id = "habitat_color_frame"
        camera_info_msg.width = sim_settings["width"]
        camera_info_msg.height = sim_settings["height"]
        camera_info_msg.K = [focal_length, 0, self.width / 2.0, 0, focal_length, self.height/2.0, 0, 0, 1]
        camera_info_msg.D = [0, 0, 0, 0, 0]
        camera_info_msg.R = [1.0, 0, 0, 0, 1.0, 0, 0, 0, 1.0]
        camera_info_msg.P = [focal_length, 0, self.width / 2.0, 0, 0, focal_length, self.height/2.0, 0, 0, 0, 1.0, 0]
        camera_info_msg.distortion_model = 'plumb_bob'

        self.rgb_info_msg = copy.deepcopy(camera_info_msg)

        self.depth_info_msg = copy.deepcopy(camera_info_msg)
        self.depth_info_msg.header.frame_id = "habitat_depth_frame"

        # Publishers
        self.rgb_pub = rospy.Publisher(self.image_topic_name, rosImage, queue_size=10)
        self.depth_pub = rospy.Publisher(self.depth_topic_name, rosImage, queue_size=10)
        self.semantic_pub = rospy.Publisher(self.semantic_topic_name, rosImage, queue_size=10)

        self.rgb_info_pub = rospy.Publisher(self.image_info_topic_name, CameraInfo, queue_size=10)
        self.semantic_info_pub = rospy.Publisher(self.semantic_info_topic_name, CameraInfo, queue_size=10)
        self.depth_info_pub = rospy.Publisher(self.depth_info_topic_name, CameraInfo, queue_size=10)

    def run_simulation(self, compressed=False):

        # Initialization for motion blur
        self.init_motion_blur()

        # Run simulation until it is cancelled
        window = Tk()  # Create a window
        window.title("Arrow Keys")  # Set a title

        self.canvas = Canvas(window, bg="white", width=300, height=300)
        self.canvas.pack()

        # Bind canvas with key events
        self.canvas.bind("<Up>", self.action_up)
        self.canvas.bind("<Down>", self.action_down)
        self.canvas.bind("<Left>", self.action_left)
        self.canvas.bind("<Right>", self.action_right)
        self.canvas.bind('<Escape>', self.action_exit)
        self.canvas.bind('<space>', self.action_stay)
        self.canvas.bind('<Control-Up>', self.action_look_up)
        self.canvas.bind('<Control-Down>', self.action_look_down)
        self.canvas.bind('<Shift-Up>', self.action_move_up)
        self.canvas.bind('<Shift-Down>', self.action_move_down)
        self.canvas.focus_set()

        self.LOOP_ACTIVE = True
        self.action = 'stay'

        rate = rospy.Rate(self.target_fps)

        self.load_poses()

        if self.replay_mode:
            for state in self.replay_states:
                self.agent.set_state(state)
                self.render(compressed)

            with open(self.output_agent_pose_name, "wb") as fp:
                pickle.dump(self.agent_pose, fp)
            self.bag.close()
            self.sim.reset()

        else:
            while self.LOOP_ACTIVE:

                window.update()
                self.sim.step(self.action)
                self.render(compressed)

    def render(self, compressed=False):

        # render observation
        observation = self.sim.get_sensor_observations()

        # write to rosbag
        rgb = observation["color_sensor"]  # self.agent.get_state().sensor_states['color_sensor'].
        semantic = observation["semantic_sensor"]

        semantic = np.asarray(self.map_to_class_labels(semantic))
        semantic = np.stack((semantic[0], semantic[1], semantic[2]), axis=2)
        semantic = semantic.astype(np.uint8)

        depth = observation["depth_sensor"]
        depth = np.float32(depth)
        # depth = np.uint16(depth)

        rgb, depth, semantic = self.motion_blur(rgb, depth, semantic)

        try:
            self.write_to_bag(rgb, semantic, depth, compressed=compressed)
        except Exception as e:
            #print(e)
            return

        bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR) / 255.
        semantic_bgr = cv2.cvtColor(semantic , cv2.COLOR_RGB2BGR) / 255.
        depth3 = cv2.cvtColor(depth / 10.0, cv2.COLOR_GRAY2BGR)  ## to scale under float 1.0
        dst = np.hstack((bgr, depth3, semantic_bgr))

        cv2.imshow("Rendered sensors", cv2.resize(dst, (0, 0), fx=0.5, fy=0.5))
        cv2.waitKey(1)


    def write_to_bag(self, rgb, semantic, depth, compressed=False):

        timestamp = rospy.Time.now()

        image_color = self.bridge.cv2_to_imgmsg(rgb[:, :, :3], "rgb8")
        image_color.header.frame_id = self.rgb_info_msg.header.frame_id
        image_depth = self.bridge.cv2_to_imgmsg(depth)
        image_depth.header.frame_id = self.rgb_info_msg.header.frame_id
        image_semantic = self.bridge.cv2_to_imgmsg(semantic, "rgb8")
        image_semantic.header.frame_id = self.rgb_info_msg.header.frame_id

        image_color.header.stamp = timestamp
        image_depth.header.stamp = timestamp
        image_semantic.header.stamp = timestamp

        tf_msg = self.create_tf_msg(timestamp)

        rgb_info = self.rgb_info_msg
        rgb_info.header.stamp = timestamp
        depth_info = self.depth_info_msg
        depth_info.header.stamp = timestamp

        self.publish_msgs(image_color, image_semantic, image_depth, tf_msg, timestamp)

        if compressed:
            image_color = CompressedImage()
            image_color.format = "jpeg"
            image_color.data = np.array(cv2.imencode('.jpg', rgb[:, :, :3])[1]).tostring()
        #
            image_depth = CompressedImage()
            image_depth.format = ""
            image_depth.data = np.array(cv2.imencode('.jpg', depth)[1]).tostring()
        #
            image_semantic = CompressedImage()
            image_semantic.format = "jpeg"
            image_semantic.data = np.array(cv2.imencode('.jpg', semantic)[1]).tostring()
        #
            self.bag.write(self.compressed_image_topic_name, image_color)
            self.bag.write(self.image_info_topic_name, rgb_info)
            self.bag.write(self.compressed_depth_topic_name, image_depth)
            self.bag.write(self.depth_info_topic_name, depth_info)
            self.bag.write(self.compressed_semantic_topic_name, image_semantic)
        else:
            self.bag.write(self.image_topic_name, image_color)
            self.bag.write(self.image_info_topic_name, rgb_info)
            self.bag.write(self.depth_topic_name, image_depth)
            self.bag.write(self.depth_info_topic_name, depth_info)
            self.bag.write(self.semantic_topic_name, image_semantic)
        #
        # self.bag.write('/tf', tf_msg)

    def publish_msgs(self, color_msg, semantic_msg, depth_msg, tf_msg, timestamp):

        br = tf2_ros.TransformBroadcaster()

        # for transform in tf_msg.transforms:
        #      br.sendTransform(transform)

        rgb_info = self.rgb_info_msg
        rgb_info.header.stamp = timestamp
        semantic_info = rgb_info
        depth_info = self.depth_info_msg
        depth_info.header.stamp = timestamp

        self.rgb_pub.publish(color_msg)
        self.semantic_pub.publish(semantic_msg)
        self.depth_pub.publish(depth_msg)
        self.rgb_info_pub.publish(rgb_info)
        self.depth_info_pub.publish(depth_info)
        self.semantic_info_pub.publish(semantic_info)

    def create_tf_msg(self, timestamp):

        tf_msg = tf2msg.TFMessage()
        agent_state = self.agent.get_state()

        ### BASE FRAME TRANSFORMATION
        tf_msg.transforms.append(rosgeo.TransformStamped())
        tf_msg.transforms[0].header.stamp = timestamp
        tf_msg.transforms[0].header.frame_id = self.world_frame
        tf_msg.transforms[0].child_frame_id = self.robot_frame

        tf_msg.transforms[0].transform.translation.x = float(agent_state.position[0])
        tf_msg.transforms[0].transform.translation.y = float(agent_state.position[1])
        tf_msg.transforms[0].transform.translation.z = float(agent_state.position[2])
        tf_msg.transforms[0].transform.rotation.x = agent_state.rotation.x
        tf_msg.transforms[0].transform.rotation.y = agent_state.rotation.y
        tf_msg.transforms[0].transform.rotation.z = agent_state.rotation.z
        tf_msg.transforms[0].transform.rotation.w = agent_state.rotation.w
        tf_msg.transforms[0] = transform_habitat_to_ROS(tf_msg.transforms[0])

        ### COLOR TRANSFORMATION
        tf_msg.transforms.append(rosgeo.TransformStamped())
        tf_msg.transforms[1].header.stamp = timestamp #rospy.Time.now()
        tf_msg.transforms[1].header.frame_id = self.robot_frame
        tf_msg.transforms[1].child_frame_id = 'habitat_rgb_frame'

        color_trans = agent_state.sensor_states['color_sensor'].position - agent_state.position
        color_rot = agent_state.sensor_states['color_sensor'].rotation * agent_state.rotation.inverse()

        tf_msg.transforms[1].transform.translation.x = float(color_trans[0])
        tf_msg.transforms[1].transform.translation.y = float(color_trans[1])
        tf_msg.transforms[1].transform.translation.z = float(color_trans[2])
        tf_msg.transforms[1].transform.rotation.x = color_rot.x
        tf_msg.transforms[1].transform.rotation.y = color_rot.y
        tf_msg.transforms[1].transform.rotation.z = color_rot.z
        tf_msg.transforms[1].transform.rotation.w = color_rot.w
        tf_msg.transforms[1] = transform_habitat_to_ROS(tf_msg.transforms[1])

        ### DEPTH TRANSFORMATION
        tf_msg.transforms.append(rosgeo.TransformStamped())
        tf_msg.transforms[2].header.stamp = timestamp
        tf_msg.transforms[2].header.frame_id = self.robot_frame
        tf_msg.transforms[2].child_frame_id = 'habitat_depth_frame'

        depth_trans = agent_state.sensor_states['depth_sensor'].position - agent_state.position
        depth_rot = agent_state.sensor_states['depth_sensor'].rotation * agent_state.rotation.inverse()

        tf_msg.transforms[2].transform.translation.x = float(depth_trans[0])
        tf_msg.transforms[2].transform.translation.y = float(depth_trans[1])
        tf_msg.transforms[2].transform.translation.z = float(depth_trans[2])
        tf_msg.transforms[2].transform.rotation.x = depth_rot.x
        tf_msg.transforms[2].transform.rotation.y = depth_rot.y
        tf_msg.transforms[2].transform.rotation.z = depth_rot.z
        tf_msg.transforms[2].transform.rotation.w = depth_rot.w
        tf_msg.transforms[2] = transform_habitat_to_ROS(tf_msg.transforms[2])

        ### SEMANTIC TRANSFORMATION
        tf_msg.transforms.append(rosgeo.TransformStamped())
        tf_msg.transforms[3].header.stamp = timestamp
        tf_msg.transforms[3].header.frame_id = self.robot_frame
        tf_msg.transforms[3].child_frame_id = 'habitat_semantic_frame'

        semantic_trans = agent_state.sensor_states['semantic_sensor'].position - agent_state.position
        semantic_rot = agent_state.sensor_states['semantic_sensor'].rotation * agent_state.rotation.inverse()

        tf_msg.transforms[3].transform.translation.x = float(semantic_trans[0])
        tf_msg.transforms[3].transform.translation.y = float(semantic_trans[1])
        tf_msg.transforms[3].transform.translation.z = float(semantic_trans[2])
        tf_msg.transforms[3].transform.rotation.x = semantic_rot.x
        tf_msg.transforms[3].transform.rotation.y = semantic_rot.y
        tf_msg.transforms[3].transform.rotation.z = semantic_rot.z
        tf_msg.transforms[3].transform.rotation.w = semantic_rot.w
        tf_msg.transforms[3] = transform_habitat_to_ROS(tf_msg.transforms[3])

        self.agent_pose.append(agent_state)

        return tf_msg


    def action_up(self, event):
        self.action = 'move_forward'

    def action_down(self, event):
        self.action = 'move_backward'

    def action_left(self, event):
        self.action = 'turn_left'

    def action_stay(self, event):
        self.action = 'stay'

    def action_right(self, event):
        self.action = 'turn_right'

    def action_exit(self, event):
        self.LOOP_ACTIVE = False
        with open(self.output_agent_pose_name, "wb") as fp:
            pickle.dump(self.agent_pose, fp)
        self.bag.close()
        self.sim.reset()

    def action_look_up(self, event):
        self.action = 'look_up'

    def action_look_down(self, event):
        self.action = 'look_down'

    def action_move_up(self, event):
        self.action = 'move_up'

    def action_move_down(self, event):
        self.action = 'move_down'

    def load_poses(self):

        if os.path.isfile(self.output_agent_pose_name):
            self.replay_states = np.load(self.output_agent_pose_name, allow_pickle=True)
        else:
            self.replay_states = []

    def _generate_label_map(self, scene):

        instance_id_to_name = {}
        for obj in scene.objects:
            if obj and obj.category:
                obj_id = int(obj.id.split("_")[-1])
                instance_id_to_name[obj_id] = obj.category.name()

        return instance_id_to_name

    def _generate_object_dict(self, scene):

        object_dict = CLASS_DICT.copy()

        object_dict['base-cabinet'] = CLASS_DICT['furniture']
        object_dict['cabinet'] = CLASS_DICT['furniture']
        object_dict['cooktop'] = CLASS_DICT['furniture']
        object_dict['countertop'] = CLASS_DICT['furniture']
        object_dict['desk'] = CLASS_DICT['furniture']
        object_dict['nightstand'] = CLASS_DICT['furniture']
        object_dict['shelf'] = CLASS_DICT['furniture']
        object_dict['wall-cabinet'] = CLASS_DICT['furniture']
        object_dict['wardrobe'] = CLASS_DICT['furniture']
        object_dict['small-appliance'] = CLASS_DICT['objects']
        object_dict['major-appliance-appliance'] = CLASS_DICT['objects']
        object_dict['blinds'] = CLASS_DICT['window']
        object_dict['shower-stall'] = CLASS_DICT['wall']
        
        #print(object_dict)
        return object_dict

    def motion_blur(self, rgb, depth, semantic):

        self.prev_rgb.append(gaussian(rgb, sigma=self.gaussian_sigma, multichannel=True, preserve_range=True))
        self.prev_depth.append(gaussian(depth, sigma=self.gaussian_sigma, multichannel=True, preserve_range=True))
        self.prev_semantic.append(gaussian(semantic, sigma=self.gaussian_sigma, multichannel=True, preserve_range=True))

        self.prev_rgb = self.prev_rgb[-10:]
        self.prev_depth = self.prev_depth[-10:]
        self.prev_semantic = self.prev_semantic[-10:]

        _rgb = (sum([1 / self.motion_blur_weight ** i * self.prev_rgb[i] for i in range(9, -1, -1)]) / self.norm_factor).astype(np.uint8)
        _depth = sum([1 / self.motion_blur_weight ** i * self.prev_depth[i] for i in range(9, -1, -1)]) / self.norm_factor
        _semantic = (sum([1 / self.motion_blur_weight ** i * self.prev_semantic[i] for i in range(9, -1, -1)]) / self.norm_factor).astype(np.uint8)

        return _rgb, _depth, _semantic

    def normalize(self, c):
        return sum([1/c**i for i in range(10)]) 

    def init_motion_blur(self):
        for i in range(10):
            self.sim.step("stay")
            # render observation
            observation = self.sim.get_sensor_observations()
            rgb = observation["color_sensor"]
            semantic = observation["semantic_sensor"]

            semantic = np.asarray(self.map_to_class_labels(semantic))
            semantic = np.stack((semantic[0], semantic[1], semantic[2]), axis=2)
            semantic = semantic.astype(np.uint8)

            depth = observation["depth_sensor"]

            self.prev_rgb.append(gaussian(rgb, sigma=self.gaussian_sigma, multichannel=True, preserve_range=True))
            self.prev_depth.append(gaussian(depth, sigma=self.gaussian_sigma, multichannel=True, preserve_range=True))
            self.prev_semantic.append(gaussian(semantic, sigma=self.gaussian_sigma, multichannel=True, preserve_range=True))




if __name__ == "__main__":

    rospy.init_node("habitat_node")
    InteractiveSimulator()
