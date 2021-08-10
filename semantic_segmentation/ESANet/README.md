## Mixed inference with semantic and color segmentation

This node is created to run with the semantic_mapping module, will be used to provide the semantic masks, 
that the Kimera-based algorithm can project onto the metric pointcloud and merge into the 3D semantic mesh


### Usage

For usage we provide a conda environment with requirements for the semantic modules (detailed in the [ESA_README](ESA_README.md) file),
and the additional ROS related components and libs that is necessary for the ROS-based communication and functionaliies

This conda can be installed as detaled in the [ROS README](../kimera_ws/README.md). 

### The ROS Node

The inference parameters passed as command line arguments are the following for the [ros_inference.py](ros_inference.py) node.

- `segmentation_mode`  - Choose what to use from the implemented modes: 0 - Semantic Segmentation and Color;
                                 '1 - Only Semantic segmentation; 2 - only color segmentation
- `ckpt_path` -  The (relative) path where the trained model checkpoint is saved
- `depth_scale` - Additional depth scaling factor to apply.
- `image_topic_name` - The streaming RGB image topic over ROS
- `depth_topic_name` - The streaming depth image topic over ROS
- `cv_show_image` -  If we want to open an CV image windows for the semantic images (high load, sometimes freezes), you can see the masked images in rviz instead over the semantic image topic
- `use_prediction_confidence_filtering` - Mask images with black labels if prediction confidence is bellow a given threshold
- `prediction_confidence_treshold` - Threshold value for masking unsure predictions and replacing with void label and black color
- `color_segmentation_label_path` - Filepath for the CSv, where we store the target colors,  labels and HSV tresholds for different classes. A few examples are stored in the config folder


