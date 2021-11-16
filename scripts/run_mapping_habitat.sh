source ./init_workspace.sh

roslaunch \
  kimera_interface \
  gt_habitat_semantic.launch \
  should_use_sim_time:=true \
  play_bag:=false \
  rgb_encoding:=raw \
  depth_encoding:=raw
