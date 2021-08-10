source ./init_catkin_ws.sh


roslaunch \
  realsense2_camera \
  rs_camera.launch \
  enable_sync:=true \
  align_depth:=true \
  camera:=${HOSTNAME} \
  enable_pointcloud:=false \
  depth_fps:=6 \
  color_fps:=6

