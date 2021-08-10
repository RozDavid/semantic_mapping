source ./init_workspace.sh

ROBOT_NAME=${HOSTNAME}
if [ "$#" -ne 1 ]; then
    ROBOT_NAME=${HOSTNAME}
    echo "Running semantic mapping from source robot: $ROBOT_NAME"
else
    ROBOT_NAME=$1
    echo "Running semantic mapping from source robot: $ROBOT_NAME"
fi

roslaunch \
  kimera_interface \
  ucoslam_realsense_semantic.launch \
  robot_hostname:=${ROBOT_NAME} \
  metric_semantic_reconstruction:=false



