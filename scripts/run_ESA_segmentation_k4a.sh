source ./init_workspace.sh

source ./init_conda.sh
conda activate semantic_mapping

cd ../semantic_segmentation/ESANet


CAMERA_NAME=${HOSTNAME}
if [ "$#" -ne 1 ]; then
    CAMERA_NAME=${HOSTNAME}
    echo "Running inference on $CAMERA_NAME"
else
    CAMERA_NAME=$1
    echo "Running inference from remote camera $CAMERA_NAME"
fi


python ros_inference.py \
	--image_topic_name /rgb/image_raw \
	--depth_topic_name /depth_to_rgb/image_raw \
	--semantic_topic_name /semantics/semantic_image \
	--segmentation_mode 1 \
	--dataset sunrgbd \
	--raw_depth \
	--cv_show_image False \
	--depth_scale 1.0


