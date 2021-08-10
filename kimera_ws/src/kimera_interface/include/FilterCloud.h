//
// Created by David Rozenberszki on 2021. 02. 22..
//

#ifndef KIMERA_SEMANTICS_FILTERCLOUD_H
#define KIMERA_SEMANTICS_FILTERCLOUD_H

#include <boost/version.hpp>

#if ((BOOST_VERSION / 100) % 1000) >= 53

#include <boost/thread/lock_guard.hpp>

#endif

#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <sensor_msgs/PointCloud2.h>
#include <image_geometry/pinhole_camera_model.h>
#include <depth_image_proc/depth_traits.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>

#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <Eigen/Geometry>
#include <Eigen/Dense>

#include <glog/logging.h>
#include <iostream>


using namespace message_filters::sync_policies;
namespace enc = sensor_msgs::image_encodings;

typedef ApproximateTime<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo> SyncPolicy;
typedef message_filters::Synchronizer<SyncPolicy> Synchronizer;
typedef sensor_msgs::PointCloud2 PointCloud;


class FilterCloud {

public:
    FilterCloud();
    ~FilterCloud();

    /**
     * Synchronized subscriber callback. Invoked when all three input msgs are received within the SyncPolicy treshold
     * @param depth_msg The depth image in float
     * @param rgb_msg  RGB ROS msg type, the same dimensions as the depth image
     * @param info_msg Custom msg type that stores the camera parameters and frames
     */
    void imageCb(const sensor_msgs::ImageConstPtr &depth_msg,
                 const sensor_msgs::ImageConstPtr &rgb_msg,
                 const sensor_msgs::CameraInfoConstPtr &info_msg);

    /**
     * The conversion function. We can include the filtering methods here if we want to downsample or depth pixel values by
     * custom rules
     * @tparam T
     * @param depth_msg
     * @param rgb_msg_in
     * @param cloud_msg
     * @param model_
     * @param red_offset
     * @param green_offset
     * @param blue_offset
     * @param color_step
     */
    template<typename T>
    void convert(const sensor_msgs::ImageConstPtr &depth_msg,
                 const sensor_msgs::ImageConstPtr &rgb_msg_in,
                 const PointCloud::Ptr &cloud_msg,
                 const image_geometry::PinholeCameraModel& model_,
                 int red_offset, int green_offset, int blue_offset, int color_step);

    /**
     * Filters by finding velocity and angular velocity outliers in the last n received transform
     * The queue size, and velocity/angular velocity limits are given as ROS params
     * @param latestTransform stamped transform looked up by tf:: in the world/odometry fram
     * @param previousTransforms a vector of the last n transform
     * @return True if we should filter this message and false if we can go on integrating in the global map
     */
    bool filterByLocalizedPose(geometry_msgs::TransformStamped& latestTransform,
                               std::vector<geometry_msgs::TransformStamped>& previousTransforms);

    void loadParams();


private:

    //ROS
    ros::NodeHandle nh_private;

    // Transformations
    tf2_ros::Buffer tfBuffer_;
    tf2_ros::TransformListener tfListener_;
    std::string source_frame_ = "world";
    std::string target_frame_ = "camera_link";
    std::string pointcloud_frame_ = target_frame_;
    geometry_msgs::TransformStamped transformStamped_;

    // Transform outlier filter
    bool filterTransformOutliers_ = true;
    std::vector<geometry_msgs::TransformStamped> previousTransforms_;
    int transformOutlierQueueSize_ = 10;
    double translation_treshold_ = 1.0; // m/s
    double rotation_treshold_ = M_PI; // rad/s
    double timestamp_difference_limit_ = 0.5; // s

    // Publications
    ros::Publisher pub_point_cloud_;

    // Subscriptions
    boost::shared_ptr<image_transport::ImageTransport> rgb_it_, depth_it_;
    image_transport::SubscriberFilter sub_depth_, sub_rgb_;
    std::string rgb_encoding_ = "raw";
    std::string depth_encoding_ = "raw";
    message_filters::Subscriber<sensor_msgs::CameraInfo> sub_info_;
    boost::shared_ptr<Synchronizer> sync_;
    std::string depth_image_transport_param = "depth_image_transport";

    // ROS params
    int queue_size_ = 150;
    double max_interval_duration_;
    std::string color_topic_name = "/rgb/image_raw";
    std::string camera_info_topic_name = "/rgb/camera_info";
    std::string depth_topic_name = "/depth_to_rgb/image_raw";
    std::string pointcloud_topic_name = "/kimera_interface/filtered_pointcloud";
    std::vector<int> void_color = {0,0,0};


    //Pc message
    PointCloud::Ptr consistent_msg_;
};


#endif //KIMERA_SEMANTICS_FILTERCLOUD_H
