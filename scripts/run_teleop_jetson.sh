source ./init_workspace.sh

ROBOT_NAME=${HOSTNAME}
if [ "$#" -ne 1 ]; then
    ROBOT_NAME=${HOSTNAME}
    echo "Running teleoperation for $ROBOT_NAME"
else
    ROBOT_NAME=$1
    echo "Running teleoperation for remote robot $ROBOT_NAME"
fi

rosrun teleop_twist_keyboard \
	teleop_twist_keyboard.py \
	_key_timeout:=0.6 \
	_speed:=0.5 \
	_turn:=0.1 \
	cmd_vel:=/${ROBOT_NAME}/cmd_vel 



