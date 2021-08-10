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

from matplotlib import pyplot as plt

# function to display the topdown map
from PIL import Image

import habitat_sim
from habitat_sim.utils import common as utils
from habitat_sim.utils import viz_utils as vut
from tkinter import * # Import tkinter

import rosbag
from cv_bridge import CvBridge
import sensor_msgs.msg as rosmsg
import rospy

from utils import *
from config import *
from ros import *

def continuous_nav(do_make_video=True, show_video=False, display=True):
    seed = 7
    use_current_scene = False

    sim_settings["seed"] = seed
    if not use_current_scene:
        # reload a default nav scene
        sim_settings["scene"] = test_scene
        cfg = make_cfg(sim_settings)
        try:  # make initialization idiot proof
            sim.close()
        except NameError:
            pass
        sim = habitat_sim.Simulator(cfg)
    random.seed(sim_settings["seed"])
    sim.seed(sim_settings["seed"])
    # set new initial state
    sim.initialize_agent(agent_id=0)
    agent = sim.agents[0]

    bag = rosbag.Bag('test.bag', 'w')
    bridge = CvBridge()
    img_msg = rosmsg.Image()
    rospy.init_node("habitat")

    # Seconds to simulate:
    sim_time = 10

    # Optional continuous action space parameters:
    continuous_nav = True

    # defaults for discrete control
    # control frequency (actions/sec):
    control_frequency = 3
    # observation/integration frames per action
    frame_skip = 1
    if continuous_nav:
        control_frequency = 5
        frame_skip = 12

    fps = control_frequency * frame_skip
    print("fps = " + str(fps))
    control_sequence = []
    for _action in range(int(sim_time * control_frequency)):
        if continuous_nav:
            # allow forward velocity and y rotation to vary
            control_sequence.append(
                {
                    "forward_velocity": random.random() * 2.0,  # [0,2)
                    "rotation_velocity": (random.random() - 0.5) * 2.0,  # [-1,1)
                }
            )
        else:
            control_sequence.append(random.choice(action_names))

    # create and configure a new VelocityControl structure
    vel_control = habitat_sim.physics.VelocityControl()
    vel_control.controlling_lin_vel = True
    vel_control.lin_vel_is_local = True
    vel_control.controlling_ang_vel = True
    vel_control.ang_vel_is_local = True

    # try 2 variations of the control experiment
    for iteration in range(1):
        # reset observations and robot state
        observations = []

        video_prefix = "nav_sliding"
        sim.config.sim_cfg.allow_sliding = True
        # turn sliding off for the 2nd pass - WE DONT USE THIS
        if iteration == 1:
            sim.config.sim_cfg.allow_sliding = False
            video_prefix = "nav_no_sliding"

        print(video_prefix)

        # manually control the object's kinematic state via velocity integration
        time_step = 1.0 / (frame_skip * control_frequency)
        print("time_step = " + str(time_step))
        for action in control_sequence:

            # apply actions
            if continuous_nav:
                # update the velocity control
                # local forward is -z
                vel_control.linear_velocity = np.array([0, 0, -action["forward_velocity"]])
                # local up is y
                vel_control.angular_velocity = np.array([0, action["rotation_velocity"], 0])

            else:  # discrete action navigation
                discrete_action = agent.agent_config.action_space[action]

                did_collide = False
                if agent.controls.is_body_action(discrete_action.name):
                    did_collide = agent.controls.action(
                        agent.scene_node,
                        discrete_action.name,
                        discrete_action.actuation,
                        apply_filter=True,
                    )
                else:
                    for _, v in agent._sensors.items():
                        habitat_sim.errors.assert_obj_valid(v)
                        agent.controls.action(
                            v.object,
                            discrete_action.name,
                            discrete_action.actuation,
                            apply_filter=False,
                        )

            # simulate and collect frames
            for _frame in range(frame_skip):
                if continuous_nav:
                    # Integrate the velocity and apply the transform.
                    # Note: this can be done at a higher frequency for more accuracy
                    agent_state = agent.state
                    previous_rigid_state = habitat_sim.RigidState(
                        utils.quat_to_magnum(agent_state.rotation), agent_state.position
                    )

                    # manually integrate the rigid state
                    target_rigid_state = vel_control.integrate_transform(
                        time_step, previous_rigid_state
                    )

                    # snap rigid state to navmesh and set state to object/agent
                    # calls pathfinder.try_step or self.pathfinder.try_step_no_sliding
                    end_pos = sim.step_filter(
                        previous_rigid_state.translation, target_rigid_state.translation
                    )

                    # set the computed state
                    agent_state.position = end_pos
                    agent_state.rotation = utils.quat_from_magnum(
                        target_rigid_state.rotation
                    )
                    agent.set_state(agent_state)

                    # Check if a collision occured
                    dist_moved_before_filter = (
                            target_rigid_state.translation - previous_rigid_state.translation
                    ).dot()
                    dist_moved_after_filter = (
                            end_pos - previous_rigid_state.translation
                    ).dot()

                    # NB: There are some cases where ||filter_end - end_pos|| > 0 when a
                    # collision _didn't_ happen. One such case is going up stairs.  Instead,
                    # we check to see if the the amount moved after the application of the filter
                    # is _less_ than the amount moved before the application of the filter
                    EPS = 1e-5
                    collided = (dist_moved_after_filter + EPS) < dist_moved_before_filter

                # run any dynamics simulation
                sim.step_physics(time_step)

                # render observation
                observation = sim.get_sensor_observations()

                # write to rosbag
                rgb = observation["color_sensor"]
                semantic = observation["semantic_sensor"]
                depth = observation["depth_sensor"]

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

                observations.append(observation)

                if display:
                    display_sample(rgb, semantic, depth)

                # print(observations[0]["semantic_sensor"])

        if do_make_video:
            # use the vieo utility to render the observations
            vut.make_video(
                observations=observations,
                primary_obs="color_sensor",
                primary_obs_type="color",
                video_file=output_directory + "continuous_nav",
                fps=fps,
                open_vid=show_video,
            )

        bag.close()
        sim.reset()

    # [/embodied_agent_navmesh]