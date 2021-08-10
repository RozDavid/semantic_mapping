source ./init_workspace.sh

roslaunch \
	azure_kinect_ros_driver driver.launch \
	tf_prefix:=${HOSTNAME}_ \
	color_resolution:=720P \
	point_cloud:=true \
	rgb_point_cloud:=true \
	depth_mode:=NFOV_UNBINNED \
	fps:=15

	
