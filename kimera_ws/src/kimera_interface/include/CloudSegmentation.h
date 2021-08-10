//
// Created by David Rozenberszki on 2021. 02. 26..
//

#ifndef KIMERA_INTERFACE_CLOUDSEGMENTATION_H
#define KIMERA_INTERFACE_CLOUDSEGMENTATION_H

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include "pcl_ros/point_cloud.h"
#include <jsk_recognition_msgs/BoundingBoxArray.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/common/common.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/don.h>

#include <pcl/io/obj_io.h>
#include <pcl/PolygonMesh.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/common/transforms.h>
#include <eigen3/Eigen/Dense>

#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>

#include "ClassInstance.h"
#include "kimera_semantics/color.h"
#include <chrono>

#include <kimera_interface/ClassInstanceArray.h>

typedef sensor_msgs::PointCloud2 PointCloud;
typedef std::vector<std::pair<int, kimera::HashableColor>> ColorPairs;

class CloudSegmentation {

public:
    CloudSegmentation();

    ~CloudSegmentation();

    void processCloud();

    // Segmentation functions
    void filterbyColor(kimera::HashableColor targetcolor, pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud,
                       pcl::PointCloud<pcl::PointXYZRGB>::Ptr filtered_cloud);

    void segmentCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr filteredCloud, std::vector<ClassInstance> &instances,
                      int &target_label_index, kimera::HashableColor &target_label_color);

    void alignPointCloudToFloor(pcl::PointCloud<pcl::PointXYZRGB>::Ptr in_cloud_rgb,
                                pcl::PointCloud<pcl::PointXYZRGB>::Ptr out_cloud_rgb);

    // ROS functions
    void loadParams();

    void pointcloudCallback(const sensor_msgs::PointCloud2ConstPtr &cloud_msg);

    void generateBoundingBoxes(int label_index, const std::vector<ClassInstance>& instances, jsk_recognition_msgs::BoundingBoxArray& bounding_boxes);

    void generateInstancePointCloud(const std::vector<ClassInstance>& instances, kimera_interface::ClassInstanceArray& instance_cloud);


private:
    //ROS
    ros::NodeHandle nh_private;

    // Topics
    ros::Publisher instance_pointcloud_publisher_;
    ros::Publisher aligned_pointcloud_publisher_;
    ros::Publisher bounding_box_publisher_;
    ros::Subscriber pointcloud_subscriber_;

    // Transformations
    tf2_ros::Buffer tfBuffer_;
    tf2_ros::TransformListener tfListener_;
    std::string source_frame_;
    std::string target_frame_;
    geometry_msgs::TransformStamped transformStamped_;


    //ROS params
    std::string label2color_mapping_path_ = "";
    std::string pointcloud_topic_ = "";
    std::string aligned_pointcloud_topic_ = "";
    std::string instance_pointcloud_topic_ = "";
    std::string bounding_box_topic_ = "";
    std::vector<int> target_label_indexes_;
    bool align_pointcloud_;
    double cluster_tolerance_;
    int min_cluster_size_;

    //Difference of normals algorith params
    //based on: https://pcl.readthedocs.io/projects/tutorials/en/latest/don_segmentation.html
    bool use_difference_of_normals_;
    double small_don_scale_;
    double large_don_scale_;
    double don_treshold_;

    double target_frequency_;
    int queue_size_;
    std::shared_ptr<kimera::SemanticLabel2Color> semantic_label_to_color_map_ = nullptr;

    // The cloud
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr rgb_cloud_;
    std::string cloud_frame_id_;
    ros::Time cloud_stamp_;

    ColorPairs target_colors_;

};


#endif //KIMERA_INTERFACE_CLOUDSEGMENTATION_H
