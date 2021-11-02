source ./init_workspace.sh

roslaunch \
  kimera_interface \
  ucoslam_realsense_semantic.launch \
  should_use_sim_time:=true \
  play_bag:=true \
  rosbag_rate:=0.3 \
  bag_file:=$HOME/data/rsd_435.bag \
  use_compressed_bag:=false \
  rgb_encoding:=raw \
  depth_encoding:=raw \
  robot_hostname:=hp_laptop \
  left_cam_topic:=/hp_laptop/color/image_color \
  metric_semantic_reconstruction:=false \
  #use_compressed_bag:=true \
  #rgb_encoding:=compressed \
  #depth_encoding:=compressedDepth \
