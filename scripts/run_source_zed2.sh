source ./init_workspace.sh

roslaunch \
	zed_wrapper zed_no_tf.launch \
	camera_name:=${HOSTNAME} \
	node_name:=${HOSTNAME}_zed_node \
	camera_model:=zed2 \
	base_frame:=${HOSTNAME}_link
