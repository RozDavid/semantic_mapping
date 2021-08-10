source ./init_workspace.sh

roslaunch zed_cpu_ros rectified_zed.launch \
	camera_namespace:=${HOSTNAME}
