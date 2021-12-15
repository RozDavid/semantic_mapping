source ./init_workspace.sh

roslaunch \
	azure_kinect_ros_driver driver.launch \
	tf_prefix:=${HOSTNAME}_ \
	color_resolution:=720P \
	point_cloud:=false \
	rgb_point_cloud:=false \
	depth_enabled:=true \
	depth_mode:=NFOV_UNBINNED \
	depth_unit:=16UC1 \
	fps:=15

# WARNING: depth in 16UC1 format is in millimeters, but depth in 32FC1 format is in meters! 
# Make sure that the depth scale must be consistent in your pipeline.
# see https://github.com/microsoft/Azure_Kinect_ROS_Driver/commit/ea70ccf
