source ./init_workspace.sh

source ./init_conda.sh
conda activate semantic_mapping

SCRIPT_PATH="../kimera_ws/src/mask_rcnn_ros"

CAMERA_NAME=${HOSTNAME}
if [ "$#" -ne 1 ]; then
    CAMERA_NAME=${HOSTNAME}
    echo "Running inference on $CAMERA_NAME"
else
    CAMERA_NAME=$1
    echo "Running inference from remote camera $CAMERA_NAME"
fi


python ${SCRIPT_PATH}/src/mask_rcnn_node.py \
	--input_rgb_topic /${CAMERA_NAME}/color/image_raw \
	--visualization False \
	--model_path ${SCRIPT_PATH}/models/mask_rcnn_sun.h5 \
	--dataset_name sunrgbd 




