import math
import os
import random
import sys

import git
import imageio
import magnum as mn
import numpy as np

from matplotlib import pyplot as plt

# function to display the topdown map
from PIL import Image

import habitat_sim
from habitat_sim.utils import common as utils
from habitat_sim.utils import viz_utils as vut
from habitat_sim.utils.common import (
    quat_from_angle_axis,
)

import rosbag
from cv_bridge import CvBridge
import sensor_msgs.msg as rossen
import geometry_msgs.msg as rosgeo
import tf2_msgs.msg as tf2msg
import rospy
#import tf
#import tf2_ros

from utils import *
from config import *
from ros import *

def simple_nav(display=True):
    cfg = make_cfg(sim_settings)
    try:  # Got to make initialization idiot proof
        sim.close()
    except NameError:
        pass
    sim = habitat_sim.Simulator(cfg)

    # the randomness is needed when choosing the actions
    random.seed(sim_settings["seed"])
    sim.seed(sim_settings["seed"])

    # Set agent state
    agent = sim.initialize_agent(sim_settings["default_agent"])
    agent_state = habitat_sim.AgentState()
    agent_state.position = np.array([-0.6, 0.0, 0.0])  # world space
    agent.set_state(agent_state)

    # Get agent state
    agent_state = agent.get_state()
    print("agent_state: position", agent_state.position, "rotation", agent_state.rotation)

    total_frames = 0
    action_names = list(cfg.agents[sim_settings["default_agent"]].action_space.keys())

    max_frames = 2

    bag = rosbag.Bag('test.bag', 'w')
    bridge = CvBridge()
    img_msg = rossen.Image()
    # rostime = rospy.Time()
    #broadcaster = tf2_ros.StaticTransformBroadcaster()
    #tf_msg.TFMessage
    rospy.init_node("habitat")
    try:
        while total_frames < max_frames:
            action = random.choice(action_names)
            print("action", action)

            agent_state = agent.get_state()
            agent_state.rotation = quat_from_angle_axis(30.0, np.array([1.0, 0, 0]))
            # need to move the sensors too
            for sensor in agent_state.sensor_states:
                agent_state.sensor_states[sensor].rotation = agent_state.rotation
                agent_state.sensor_states[
                    sensor
                ].position = agent_state.position + np.array([0, 1.5, 0])
            agent.set_state(agent_state)
            

            observations = sim.step(action)
            observations = sim.get_sensor_observations()
            
            print("agent_state: position", agent_state.position, "rotation", agent_state.rotation)

            rgb = observations["color_sensor"]
            semantic = observations["semantic_sensor"]
            depth = observations["depth_sensor"]

            timestamp = rospy.Time.now()

            image_color = bridge.cv2_to_imgmsg(rgb[:, :, :3], "bgr8")
            image_color.header.stamp = timestamp

            image_depth = bridge.cv2_to_imgmsg(depth)
            image_depth.header.stamp = timestamp

            semantic = semantic.astype(np.uint8)

            image_semantic = bridge.cv2_to_imgmsg(semantic)
            image_semantic.header.stamp = timestamp

            bag.write('/habitat/image_color', image_color)
            bag.write('/habitat/image_depth', image_depth)
            bag.write('/habitat/image_semantic', image_semantic)

            tf_msg = tf2msg.TFMessage()
            
            tf_msg.transforms.append(rosgeo.TransformStamped())
            tf_msg.transforms[0].header.stamp = timestamp
            tf_msg.transforms[0].header.frame_id = 'habitat_odometry'
            tf_msg.transforms[0].child_frame_id = 'camera'

            print(type(agent_state.rotation))
            #tf_msg.transform.translation = float(agent_state.position)

            tf_msg.transforms[0].transform.translation.x = float(agent_state.position[0])
            tf_msg.transforms[0].transform.translation.y = float(agent_state.position[1])
            tf_msg.transforms[0].transform.translation.z = float(agent_state.position[2])

            tf_msg.transforms[0].transform.rotation = agent_state.rotation
            #tf_msg.transform.rotation.x = agent_state.rotation[0]
            #tf_msg.transform.rotation.y = agent_state.rotation[1]
            #tf_msg.transform.rotation.z = agent_state.rotation[2]
            #tf_msg.transform.rotation.w = agent_state.rotation[3]
            #broadcaster.sendTransform(tf_msg)
            bag.write('/tf', tf_msg)

            if display:
                display_sample(rgb, semantic, depth)

            total_frames += 1
    finally:
        bag.close()

if __name__ == "__main__":

    simple_nav(True)
