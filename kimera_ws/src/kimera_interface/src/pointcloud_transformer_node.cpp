//
// Created by David Rozenberszki on 2021. 03. 23..
//

#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <boost/foreach.hpp>

#include <sensor_msgs/PointCloud2.h>
#include <tf2_ros/transform_listener.h>
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

typedef sensor_msgs::PointCloud2 PointCloud;
typedef sensor_msgs::PointCloud2::ConstPtr PointCloudPtr;

tf2_ros::Buffer tf_buffer_;
ros::Publisher publisher;
std::vector<ros::Subscriber> subscribers;

std::string target_frame_;
double tf_timeout_;
std::vector<std::string> pointcloud_topic_names_;
std::string output_topic_name_;

void pointCloudCallback(const PointCloudPtr & msg) {
    geometry_msgs::TransformStamped transform;

    try {
        transform = tf_buffer_.lookupTransform(target_frame_, msg->header.frame_id,
                                               msg->header.stamp, ros::Duration(0.5));
        PointCloud cloud_out;
        pcl_ros::transformPointCloud(target_frame_, transform.transform, *msg, cloud_out);
        publisher.publish(cloud_out);
    }
    catch (tf2::TransformException &ex) {
        ROS_WARN("%s", ex.what());
        return;
    }
}

void loadParams(const ros::NodeHandle& nh){

    // The TF frame where we want to direct all the incoming topics
    std::string target_frame;
    nh.param("target_frame", target_frame_, target_frame);

    nh.param("tf_timeout", tf_timeout_, 0.5);

    // We subscribe a vector of topics and this param stores the names
    nh.param("input_cloud_topics", pointcloud_topic_names_, {});

    std::string output_topic_name;
    nh.param("output_topic_name", output_topic_name_, output_topic_name);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "pointcloud_transformer_node");
    ros::NodeHandle nh_private("~");

    tf2_ros::TransformListener tfListener(tf_buffer_);

    loadParams(nh_private);

    publisher = nh_private.advertise<PointCloud>(output_topic_name_, 3);

    for(const auto& topic_name : pointcloud_topic_names_) {
        subscribers.push_back(nh_private.subscribe<PointCloud>(topic_name, 1, pointCloudCallback));
    }

    ros::spin();
}