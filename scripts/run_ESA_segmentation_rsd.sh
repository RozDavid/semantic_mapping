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
	--image_topic_name /${CAMERA_NAME}/color/image_raw \
	--depth_topic_name /${CAMERA_NAME}/aligned_depth_to_color/image_raw \
	--semantic_topic_name /${CAMERA_NAME}/semantics/semantic_image \
	--segmentation_mode 1 \
	--dataset sunrgbd \
	--raw_depth \
	--cv_show_image False \
	--prediction_confidence_treshold -2.0 \


