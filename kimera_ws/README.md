# General
This repository contains a semantic mapping pipeline built on a modified Kimera pipeline by David Rozenberszki and Gabor Soros in 2021.


# Prerequisites
Tested on ubuntu 18.04 and ROS Melodic

## CUDA
Follow the description [here](https://medium.com/@exesse/cuda-10-1-installation-on-ubuntu-18-04-lts-d04f89287130)

1. Start terminal and remove any NVIDIA traces you may have on your machine.
```
sudo rm /etc/apt/sources.list.d/cuda*
sudo apt remove --autoremove nvidia-cuda-toolkit
sudo apt remove --autoremove nvidia-*
```

2. Setup the correct CUDA PPA on your system
```
sudo apt update
sudo add-apt-repository ppa:graphics-drivers
sudo apt-key adv --fetch-keys  http://developer.download.nvidia.com/compute/cuda/repos/ubuntu1804/x86_64/7fa2af80.pub
sudo bash -c 'echo "deb http://developer.download.nvidia.com/compute/cuda/repos/ubuntu1804/x86_64 /" > /etc/apt/sources.list.d/cuda.list'
sudo bash -c 'echo "deb http://developer.download.nvidia.com/compute/machine-learning/repos/ubuntu1804/x86_64 /" > /etc/apt/sources.list.d/cuda_learn.list'
```

3. Install CUDA 10.2 packages
```
sudo apt update
sudo apt install cuda-10-2
sudo apt install libcudnn7
```

4. As the last step one need to specify PATH to CUDA in ‘.profile’ file. Open the file by running:
```
sudo vi ~/.profile
```
And add the following lines at the end of the file:
```
# set PATH for cuda 10.2 installation
if [ -d "/usr/local/cuda-10.2/bin/" ]; then
    export PATH=/usr/local/cuda-10.2/bin${PATH:+:${PATH}}
    export LD_LIBRARY_PATH=/usr/local/cuda-10.2/lib64${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}
fi
```
5. Restart and check the versions for the installation.

 - CUDA: `nvcc --version`
 - NVIDIA Driver: `nvidia-smi`
 - libcudnn: `/sbin/ldconfig -N -v $(sed ‘s/:/ /’ <<< $LD_LIBRARY_PATH) 2>/dev/null | grep libcudnn`

## Realsense SDK
Install the realsense SDK as described [here](https://github.com/IntelRealSense/librealsense/blob/master/doc/distribution_linux.md#installing-the-packages).

```
sudo apt-key adv --keyserver keyserver.ubuntu.com --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE || sudo apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE
sudo add-apt-repository "deb https://librealsense.intel.com/Debian/apt-repo $(lsb_release -cs) main" -u
sudo apt-get install librealsense2-dkms
sudo apt-get install librealsense2-utils
sudo apt-get install librealsense2-dev
```


## Zed SDK
You can download the Zed SDK from [this](https://www.stereolabs.com/developers/release/) link. At the time of writing, the latest version is 3.4.0. Beware the CUDA version. Download the full package and not the standard. 
[This link](https://download.stereolabs.com/zedsdk/3.4/cu102/ubuntu18_full) is for Ubuntu 18.04, Cuda10.2, full package


## Kinect4Azure SDK
Follow the description [here](https://docs.microsoft.com/hu-hu/azure/kinect-dk/sensor-sdk-download)
First, add the Microsoft package repository (Note 18.04 ubuntu version in the URL):
```
curl -sSL https://packages.microsoft.com/keys/microsoft.asc | sudo apt-key add -
sudo apt-add-repository https://packages.microsoft.com/ubuntu/18.04/multiarch/prod
sudo apt update
```

Next, install the driver and tools package:
```
sudo apt install k4a-tools
```

Next, you need to clone and build the Azure Kinect Sensor SDK as described [here](https://github.com/microsoft/Azure-Kinect-Sensor-SDK)
OR install it directly via
```
sudo apt install libk4a1.4-dev
```

If you have built the library yourself, then you need to copy the `libdepthengine.so.2.0` file to your installation folder. The original ones comes with k4a-tools and is located in `/usr/lib/x86_64-linux-gnu/libk4a1.4/libdepthengine.so.2.0`. If you have not installed the tools, you can obtain the depth engine as described [here](https://github.com/microsoft/Azure-Kinect-Sensor-SDK/blob/develop/docs/depthengine.md)
```
cd <folder of your own libk4a SDK installation>
/usr/lib/x86_64-linux-gnu/libk4a1.4/libdepthengine.so.2.0
```

Finally, you need to add udev rules so that non-root users are able to use the camera, as described [here](https://github.com/microsoft/Azure-Kinect-Sensor-SDK/blob/develop/docs/usage.md). From the SDK source repository, copy 'scripts/99-k4a.rules' into '/etc/udev/rules.d/'.
```
cd ~/Downloads
wget https://raw.githubusercontent.com/microsoft/Azure-Kinect-Sensor-SDK/develop/scripts/99-k4a.rules
sudo cp ./99-k4a.rules /etc/udev/rules.d
```

## ROS-specific prerequisites

Add ROS distro, if not already:
```
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu `lsb_release -sc` main" > /etc/apt/sources.list.d/ros-latest.list'
wget http://packages.ros.org/ros.key -O - | sudo apt-key add -
```

Install the following packages (change 'melodic' below to your own ROS distro name if different):
``` 
sudo apt-get update
sudo apt-get install python-wstool python-catkin-tools protobuf-compiler autoconf
sudo apt-get install ros-melodic-cmake-modules 
sudo apt-get install ros-melodic-image-geometry ros-melodic-pcl-ros ros-melodic-cv-bridge \
ros-melodic-rviz-visual-tools ros-melodic-visualization-msgs ros-melodic-camera-info-manager \
ros-melodic-jsk-recognition-msgs ros-melodic-jsk-rviz-plugins ros-melodic-move-base \
ros-melodic-image-proc ros-melodic-stereo-msgs ros-melodic-tf2-bullet ros-melodic-depth-image-proc \
ros-melodic-ddynamic-reconfigure ros-melodic-librealsense2 ros-melodic-image-transport-plugins
```

```
sudo apt-get install -y --no-install-recommends apt-utils
sudo apt-get install -y \
      cmake build-essential unzip pkg-config autoconf \
      libboost-all-dev \
      libjpeg-dev libpng-dev libtiff-dev \
      libvtk6-dev libgtk-3-dev \
      libatlas-base-dev gfortran \
      libparmetis-dev \
      python-wstool python-catkin-tools \
```


## Python 3 prerequisites
Some ROS nodes (primarily the semantic segmentation) require python. If you do not intend to use semantic segmentation (perhaps you don't have an nvidia GPU) then you can see below which python packages need to be installed. It is not strictly necessary to use a conda envionment, the few basic catkin packages cak be installed in you system python or your base environment if you prefer so.


Install Anaconda. Downloads are available [here](https://www.anaconda.com/products/individual#Downloads).

```
sudo apt-get install curl
cd ~/Downloads
curl –O https://repo.anaconda.com/archive/Anaconda3-2021.05-Linux-x86_64.sh
bash Anaconda3-2021.05-Linux-x86_64.sh
```

```
export SEMANTIC_MAPPING_DIR=<the root folder of this project>
cd SEMANTIC_MAPPING_DIR
```

Create Anaconda environment with Python 3.6/3.7 (NOTE: the version matters! Python 3.10 did not work for us(). We will use this environment for catkin/ROS package building and for running the semantic segmentation algorithms. Our Anaconda environment descriptor can be found in the `config` folder. Here you can choose whether you want an environment just for building (on machines without an nvidia GPU) or for both building and semantic segmentation (on machines with nvidia GPU)

If you only need catkin in the environment, create the environment as follows:
```
conda env create --name semantic_mapping python=3.7
conda activate semantic_mapping
conda install -c conda-forge catkin_pkg
conda install -c conda-forge empy pyyaml
conda install -c conda-forge rospkg
conda deactivate
```

If you want to use the semantic segmentation modules, create the environment as follows:
```
cd config
conda env create -f semantic_mapping_conda.yaml
conda activate semantic_mapping
pip install cython
pip install eigency
conda deactivate
```

Note: The last two packages need to be installed line-by-line as they are dependent on each other, pip fails for them.

It is important to completely deactivate the all environments before continuing with catkin
```
conda deactivate
```

## UcoSLAM pose estimator
We replaced Kimera-VIO with [UcoSLAM](http://www.uco.es/investiga/grupos/ava/node/62) to get better pose estimates and complete maps. We extended it with a thin ROS wrapper to be able to communicate with the rest of the pipeline. This component receives images and returns camera poses. You can use any other pose estimator instead of this component, for example [ORB-SLAM2_ROS](https://github.com/ethz-asl/orb_slam_2_ros)

Contact us if you want to use UcoSLAM and need the ROS wrapper for it.



# Building
We initialize the catkin workspace in a **!non-conda!** environment. Please make sure to exit the *(base)* Conda environment as well, in case it is activated.
```
conda deactivate
source /opt/ros/melodic/setup.bash
cd $SEMANTIC_MAPPING_DIR/kimera_ws
catkin init
```

Next, download the src dependencies with wstool:
```
cd $SEMANTIC_MAPPING_DIR/kimera_ws/src
wstool init
wstool merge kimera_interface/install/kimera_interface.rosinstall
wstool update
```

For the package `vision_opencv`, at the time of commit `fc782bb`, we need to apply a patch to be compatible with ROS melodic.
```
cd $SEMANTIC_MAPPING_DIR/kimera_ws/src
cd vision_opencv
git checkout fc782bb
git apply ../vision_opencv_fc782bb.patch
cd ..
```

For the package `gtsam`, we need to roll back to an earlier commit due to incompatibility with `Kimera-RPGO`.
```
cd $SEMANTIC_MAPPING_DIR/kimera_ws/src
cd gtsam
git checkout 59d9027
cd ..
```


Now we can configure our Catkin workspace. Go back to he `kimera_ws` folder. Configure that we want to extend the ROS workspace:
```
catkin config --extend /opt/ros/melodic
```

Configure that we want a merged devel space (each catkin package will be built into a single merged devel space):
```
catkin config --merge-devel
```

Next, we set up our Python Path. Note that `anaconda3` is the _full path_ (relative paths and environment variables do not seem to work) of your conda installation folder and `semantic_mapping` is the name of our conda environment.
```
export MY_CONDA_PATH=$HOME/dev/anaconda3 
# or $HOME/miniconda3 or whatever you have
export MY_CONDA_ENV_NAME=semantic_mapping
```

In the command below, make sure to use the correct python version (3.7 in this example)
```
catkin config --cmake-args -DCMAKE_CXX_STANDARD=14 -DCMAKE_BUILD_TYPE=Release \
-DPYTHON_EXECUTABLE=$MY_CONDA_PATH/envs/$MY_CONDA_ENV_NAME/bin/python3 \
-DPYTHON_INCLUDE_DIR=$MY_CONDA_PATH/envs/$MY_CONDA_ENV_NAME/include/python3.7m \
-DPYTHON_LIBRARY=$MY_CONDA_PATH/envs/$MY_CONDA_ENV_NAME/lib/libpython3.7m.so
```

Next, we need to blacklist the Python2-related packages
```
catkin config -a --blacklist minkindr_python numpy_eigen iclcv_catkin iclcv_segmentation
```

If you do not have the ZED SDK installed, you will need to blacklist the packages related to ZED cameras as well:
```
catkin config -a --blacklist zed_cpu_ros zed_interfaces zed_nodelets zed_ros zed_wrapper
```

Finally, start building and go for a coffee, it will take a while
```
cd $SEMANTIC_MAPPING_DIR/kimera_ws
source /opt/ros/melodic/setup.bash
source ./devel/setup.bash
catkin build opencv3_catkin
catkin build
```



# Running the semantic reconstruction

## Roscore
Start Roscore in one terminal
```
roscore
```


## Realsense camera source
The usage of the realsense ROS wrapper and advertised topics are descried [here](https://github.com/IntelRealSense/realsense-ros).

Start the camera source in a terminal
```
cd scripts
source run_source_rsd435.sh
```


## Zed camera source
You can start the zed_wrapper node with the following command in a terminal:
```
cd scripts
source init_workspace.sh
source run_source_zed2.sh or run_source_zed_mini.sh
```

Note 2: Zed cameras can be also used without CUDA and the ZED SDK as standard stereo cameras. A discussion for that can be found [here](https://github.com/stereolabs/zed-ros-wrapper/issues/580).

Note: If you haven't used UcoSLAM before with Zed camera, you need to create the config file first.
For this on a standard Ubuntu Zed SDK installation:
```
cd /usr/local/zed/settings/
cat <the config file you have for your camera>
```
Copy the parameters from the LEFT_CAM_HD Settings (HD720 is the default zed_wrapper resolution) to the UcoSLAM config file, that you include as an argument for the source script. 


## Pose estimation
We recommend to create a SLAM map first, and in a second run, load that map and localize in it. This way, the poses will be more accurate and we can avoid camera jumps during the reconstruction.

### Realsense
Next, start UcoSLAM in another terminal
```
source run_ucoslam_rsd435.sh
```
### Zed
Now you can run UcoSLAM with Zed mini camera in another terminal:
```
cd ucoslam-open/scripts
source run_ros_slam_zedm.sh
```
### Kinect4Azure


## Kimera Mapping:
Feel free to contact us for an example rosbag to run the demonstation with default arguments.

The reconstruction can be started with launch fles of the kimera_interface node. 
```
cd kimera_ws
roslaunch kimera_interface ucoslam_realsense_semantic.launch
```
We also provided some example scripts `run_mapping_xxx.sh`. You can set with the `metric_semantic_reconstruction` parameter whether you want to use semantic labels or pixel colors in the reconstruction. If you do not have the semantic segmentation node running, set this parameter to false.

Here is a list of the arguments that you might need to modify either in the run scipts or in the launch file directly
- ```--sensor_frame```  The RGB camera frame of the streaming measurement. This need to be the optical frame
- ```--odometry_frame``` The map or odometry frame where the pose is relative
- ```--left_cam_info_topic``` The camera info topic that corresponds to the RGB images
- ```--left_cam_topic``` The RGB image topic, no need for undistortion. This is only used, when `metric_semantic_reconstruction` is false
- ```--left_cam_depth_topic``` The depth images are converted to semantic pointclouds with known frame and semantic mapping. This needs to be aligned to the RGB frame
- ```--left_cam_segmentation_topic``` The colored images streaming with semantic labels. Under the RGB frames and timestamps same as depth images. 


## Semantic Segmentation
First download the trained model `sunrgbd_r34_NBt1D.tar.gz` from [here](https://drive.google.com/u/0/uc?export=download&confirm=6Y6q&id=1tviMAEOr-6lJphpluGvdhBDA_FetIR14)
(links to other models can be found in the Readme file of ESANet)
and extract it to `semantic_mapping/semantic_segmentation/ESANet/trained_models/sunrgbd/r34_NBt1D.pth`

Run the segmentation python ROS node:
```
cd scripts
source init_workspace.sh
cd semantic_segmentation/ESANet
conda activate semantic_mapping
python ros_inference.py --dataset sunrgbd --raw_depth --cv_show_image True
```

The arguments that you might need to provide as additional arguments:

- ```--ckpt_path```  The (relative) path where the trained model checkpoint is saved
- ```--depth_filtering``` If we want to get rid of low confidence measurements and apply median filter for hole filling
- ```--image_topic_name``` The streaming RGB image topic
- ```--depth_topic_name``` The streaming depth image topic
- ```--confidence_topic_name``` The confidence topic only on Zed cameras if filtering is enabled
- ```--cv_show_image``` If we want to open an CV image windows for the semantic images (high load)




# Troubleshooting

Random tales of suffering and solutions, so that you don't have to.

### GTSAM build error
I noticed in 2021 October that the latest gtsam master cannot be built in the workspace. I rolled back manually to an earlier commit 59d9027.

The error message was this:
```
(...)
/src/Kimera-RPGO/src/RobustSolver.cpp:16:10: fatal error: gtsam/nonlinear/GncOptimizer.h: No such file or directory
 #include <gtsam/nonlinear/GncOptimizer.h>
          ^~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
compilation terminated.
```


### Protobuf mismatch error

If you get the following protobuf related error:

```
(...)
File "/home/ubuntu/ss/semantic_mapping/kimera_ws/devel/lib/python3/dist-packages/google/protobuf/internal/python_message.py", line 848
except struct.error, e:
```

 - Go to the folder semantic_mapping/kimera_ws/devel/lib/python3/dist-packages and check if there is a "google" folder.
 - Rename folder "google" to "google_whatever" (this will work around the version mismatch)

### MaskRCNN cuDNN failure

When running maskrcnn semantic segmentation you encounter the following error message:

```
File "/home/ubuntu/miniconda3/envs/semantic_mapping/lib/python3.7/site-packages/tensorflow/python/client/session.py", line 1458, in __call__
run_metadata_ptr)
tensorflow.python.framework.errors_impl.UnknownError: 2 root error(s) found.
(0) Unknown: Failed to get convolution algorithm. This is probably because cuDNN failed to initialize, so try looking to see if a warning log message was printed above.
[[{{node conv1/convolution}}]]
[[mrcnn_detection/map/while/Switch_2/_4249]]
(1) Unknown: Failed to get convolution algorithm. This is probably because cuDNN failed to initialize, so try looking to see if a warning log message was printed above.
[[{{node conv1/convolution}}]]
0 successful operations.
0 derived errors ignored.
```

 - Make sure you have `TF_FORCE_GPU_ALLOW_GROWTH=true` in your environment variables
   - e.g. `export TF_FORCE_GPU_ALLOW_GROWTH=true` 
