source ./init_workspace.sh

roslaunch \
	kimera_interface custom_zed_launch.launch \
	camera_name:=${HOSTNAME} \
	node_name:=${HOSTNAME}_zed_node \
	camera_model:=zedm \
	base_frame:=${HOSTNAME}_link \
	enable_sync:=true \
	align_depth:=true \
	pub_frame_rate:=5.0 \
	point_cloud_freq:=5.0 \
	grab_frame_rate:=15 \
	resolution:=3
	


