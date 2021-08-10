source ./init_workspace.sh

roslaunch \
	zed_wrapper zed_no_tf.launch \
	camera_name:=${HOSTNAME} \
	node_name:=${HOSTNAME}_zed_node \
	camera_model:=zedm \
	base_frame:=${HOSTNAME}_link
