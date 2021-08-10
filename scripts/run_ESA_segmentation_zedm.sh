source ./init_workspace.sh

source ./init_conda.sh
conda activate semantic_mapping

cd ../semantic_segmentation/ESANet
python ros_inference.py \
	--dataset sunrgbd \
	--raw_depth \
	--cv_show_image False \
	--segmentation_mode 1 \
	--image_topic_name /${HOSTNAME}_zed_node/left/image_rect_color \
	--depth_topic_name /${HOSTNAME}_zed_node/depth/depth_registered \
	--semantic_topic_name /${CAMERA_NAME}/semantics/semantic_image 
	


