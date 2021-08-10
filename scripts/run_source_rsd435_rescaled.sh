source ./init_workspace.sh

roslaunch \
	kimera_interface rescale_camera.launch \
	camera_ns:=${HOSTNAME} \
	param_file:=k4a_camera_config.yaml

