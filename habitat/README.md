# AiHabitat simulated experiments

Testing our pipeline on a simulation with ground truth dataset and virtual sensors.
We make use of [Habitat-Sim](https://github.com/facebookresearch/habitat-sim) for Rendering sensor measurement of the Replica dataset and [Habitat-Lab](https://github.com/facebookresearch/habitat-lab) for generated noise models.

## Installation

We provide a setup script for the new conda environment that includes the Habitat simulation engines and interfacing libraries wit other ROS components. Be sure that you have all sbmodules initialized and updated, then execute:

```
source setup.sh
```

We are going to use the same catkin workspace for communication with ROS that we have previouslz created in kimera_ws. 
Here the Python path is modified to use Python 3. 

## Downloading the Replica Dataset

1. Download the Replica dataset as described [here](https://github.com/facebookresearch/Replica-Dataset). 
   To work with the Replica dataset, you need a file called `sorted_faces.bin` for each model. 
   Such files (1 file per model), along with a convenient setup script can be downloaded from here: [sorted_faces.zip](http://dl.fbaipublicfiles.com/habitat/sorted_faces.zip). You need:

- Download the zip file from the above link - *Note: Expect aroung 80Gb of Data.... :(*
- Unzip it to any location with given path and locate the *sorted_faces* folder.
- Here run  `source copy_to_folders <Replica root directory. e.g ~/models/replica/>` which copies semantic description to the scene folders

## Running the experiment

Start roscore in opened terminal
`roscore`

Setup path and conda env:

```
cd <semantic_mapping root>
cd scripts
source init_workspace.sh
cd ../habitat/habitat_interface/
conda activate habitat
```

Finally run the python interface node with

```
python traj_sim.py 
```

## Parameters to set

Change parameters for semantic mesh, topic names and etc in file `traj_sim.py`. 
All argument possible given to the python script are listed here with default values


```
--mesh_path  default=/home/RozDavid/data/Replica/frl_apartment_4/habitat/mesh_semantic.ply,  help=The Replica mesh path mesh, that provides the model for simulation
--camera_config default=../config/calib_k4a.yml,
                    help=The camera parameters of the virtual camera that simulates the image
--robot_frame default=habitat_robot_base,
                    help=The frame of the robot sensor, where the camera pose is tracked
--parent_frame default="habitat_odometry_frame",
                    help=The world or the odometry frame, where we get the sensor frame. 
                         Should be aligned to the mesh
--image_topic_name default=/habitat/rgb/image_raw,
                    help=The images will be savd under this topic
--depth_topic_name default="/habitat/depth/image_raw",
                    help=The depth images will be saved under this topic
--semantic_topic_name default=/habitat/semantics/image_raw,
                    help=The semantic imaes will be saved under this topic

--compressed_image_topic_name default=/habitat/rgb/image_raw/compressed,
                    help=The compressed images will be savd under this topic
--compressed_depth_topic_name default="/habitat/depth/image_raw/compressedDepth",
                    help=The compressed depth images will be saved under this topic
--compressed_semantic_topic_name default=/habitat/semantics/image_raw/compressed,
                    help=The compressed semantic images will be saved under this topic
--output_bag_name default="../data/output.bag",
                    help=The name and relative path of the output bag file
--output_agent_pose_name default="../data/agent_states.npy",
                    help=The name and relative path of the output agent pose file
--target_fps, default=5,
                    help=The number of frames to render per second
--compressed,  default=False,
                    help=To compress images saved to rosbag
--replay_mode, default=False,
                    help=To replay recorded trajectory from numpy array of poses
--gaussian_sigma, default=0.5,
                    help=Sigma of the Gaussian blur
--motion_blur_weight,  default=1,
                    help=Weighting of the motion blur
```

