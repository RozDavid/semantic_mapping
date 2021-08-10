source ./init_workspace.sh

# Previously we recorded a rosbag with /tf, /rgb, /depth images ad corresponding camera_info topics. 
# That rosbag's path is set in the reconstruction launch file in the kimera_interface package

# We set the parameters in the roslaunch file for reconstructin or start it directly from a script or command line
# e.g. roslaunch kimera_interface ucoslam_k4a_semantic.launch metric_semantic_reconstruction:=true rosbag_rate:=0.3
# Note: make sure to have simulated_time:=true when using rosbags


#If we want to do a semantic reconstruction we need to start an ESANet or MaskRCNN node to have synchronized semantic images with the 
# color/depth images for pointcloud generation
#e.g. source run_maskrcnn_segmentation_k4a.sh 

# Wait for the reconstruction to build over the playing rosbag - check the result in the rviz window that appears automatically

# Generate mesh with the following command:


rosservice call /kimera_semantics_node/generate_mesh

# Note: that generated mesh will appear under the path that we set in the reconstruction_pipeline launch file
# with  <param name="mesh_filename" value="$(find kimera_interface)/mesh_results/semantic_mesh.ply"/>


