# Semantic Segmentation modules

This is the place, where we are going to store the semantic segmentation components of the 3D reconstruction pipeline

Mostly it is separated from the ROS workspace, to use python3 easier, but ROS interface is already given or will be implemented

## ESANet

This is a light, still state-of-the-art network on the NYU-v2 and SUN-RGBD benchmarks for semantic segmentation. The goal is to use both modality of the RGBD streams for the best segmentation performance, and use these images for the reconstruction labels. 

### Installation

Please follow the README in the directory,
or import the conda environtment from the config folder of the root repository. 

Note: When creating the Anaconda/Miniconda environment pay attention to use the same one, for which we insalled the catkin dependencies in the `kimera_ws`. 


### Contributions

Created a ros interface for the pretrained networks. 

Here we subscribe for synced RGB and Depth images over ROS topics, run the inference on GPU and publish the filtered depth with the semantic image over ros topics again. 

We can use this with Kimera to color the meshes with the semantic labels and maintain a confusion matrix for the voxels. 


**Parameters and arguments**

- ```--ckpt_path```  The path were we downloaded the pretrained model
- ```--depth_scale``` Additional depth scaling factor to apply.
- ```--dataset``` Either "sunrgbd" or "nyu-v2" for pretrained networks
- ```--raw_depth``` Either to use or not hole filling for the depth 

The rostopics needs to be set in the parameter server or changed in source code with their defaults

**Rosparams**
- ```esa_inference_node/image_topic_name``` defauls to : `'/zedm/zed_node/rgb_raw/image_raw_color'`
- ```esa_inference_node/depth_topic_name```  defauls to : ```'/zedm/zed_node/depth/depth_registered'```
- ```esa_inference_node/confidence_topic_name```  defauls to : ```'/zedm/zed_node/confidence/confidence_map'```
- ```esa_inference_node/filtered_depth_topic_name```  defauls to : ```'/semantics/filtered_depth'```
- ```esa_inference_node/semantic_topic_name```  defauls to : ```'/semantics/semantic_image'```

### Running the node

```
cd <semantic_mapping_root>/scripts
source run_ESA_segmentation_<k4a,rsd,zedm>.sh
```


Then run the Kimera reconstruction launch file in the `kimera_interface` package, with the segmentation topic name set.


## Mask RCNN

### Installation

We used a modification of the original mask_rcnn package with ROS interfacing.
The original repository can be found [here](https://github.com/ethz-asl/mask_rcnn_ros).
Here one can also find the instruction on the installation,
but it is enough to use the import the provided conda environment again.

Additionally, we have to download the chekcpoints on the different models. We test with Mask-RCNN models trained on CoCo, 
Nyu-V2 and SunRGBD datasets, but found to be the best the SunRGBD. 

For running in inference mode download the checkpoints from [here](https://ikelte-my.sharepoint.com/:f:/g/personal/oicn4a_inf_elte_hu/EpjwmlEKyq1BhP65hiT13b8BzSnGnro2Z9q45WYyLP9dTA?e=El1xpH)
and save it under the models folder. 


Finally, running the ROS nodes take the following arguments

- ```input_rgb_topic``` RGB images to run inference on, defauls to : `'/hp_laptop/color/image_color'`
- ```model_path```  MASK RCNN checkpoint path - download from the matterport github repo, defauls to : ```'../models/mask_rcnn_coco.h5'```
- ```dataset_name``` Either 'coco' or 'sunrgbd' training set and output labels defauls to : ```'coco'```
- ```visualization```  If we would like to visualize the results with masks, names and confidence defauls to : ```True```
- ```semantic_topic_name``` The output topic name for the semantically segmented image, defauls to : ```'/semantics/semantic_image'```


And we can run the segmentation with

```
cd <semantic_mapping_root>/scripts
source run_maskrcnn_segmentation_<k4a,rsd,zedm>.sh
```

# Rendering the labels

## The idea explained

Semantic segmentation and undertsanding is crucial, but requires large amount of annotated data. Such dataset exists alreadz,
but is is really expensive to annotate. Most existing datasets are annotated from human perspective images, but home robots
are observing the world from low perspective. 
This is a challenge, as label transformation from top to bottom is not easy, and robots are not recognizing objects with models
trained on different perspective

Our proposed pipeline:
- Create SLAM map with human perspective quality RGBD sensor,
- Do the semantic reconstruction with the components in this repo
- Share the semantically labelled 3D map with the low perspective robot
- Navigate with the robot with low perspective in the same SLAM map 
Note: This is our current challenge, to localize with UcoSLAM with distant keyframes
- Publish the joint images and poses from the robot to the server
- Render images from the semantic mesh from the robot perspective given the robot pose and robot camera model
- Return the semantic annotation as rendered images overlayed on the original robot image over ROS

## Installation

We are using [Pyrender](https://github.com/mmatl/pyrender) for the rendering, that is wrapped
in a ROS node connected to the remote robot and local Kimera reconstuction pipline. 


We provide a conde environment for this task as well, load the conda environment from [rendering/conda](rendering/conda).


## The rendering node
Options to be implemented:
-Online - subscribe to the published mesh from `kimera_semantics_node`
-Offline - load the semantic mesh from file

TODO:
start python script as node and start automatic rendering