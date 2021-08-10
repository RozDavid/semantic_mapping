# For distorted input images it is necessary to first undistort them with given camera_info
# additionally for stereo camera pairs it can be done similarly with the image_proc node

# Information and documentation regarding the usage can be found here:
# http://wiki.ros.org/image_proc

# This node will rename <hostanem>/color/image_raw topic names to
# <hostanem>/color/image_color or image_rect_color that can later be used for the pipeline as new input

source ./init_workspace.sh

CAMERA_NAME=${HOSTNAME}
if [ "$#" -ne 1 ]; then
    CAMERA_NAME=${HOSTNAME}
    echo "Running image undistortion on $CAMERA_NAME"
else
    CAMERA_NAME=$1
    echo "Running image undistortion for remote camera $CAMERA_NAME"
fi

ROS_NAMESPACE=${CAMERA_NAME} rosrun image_proc image_proc


