//
// Created by David Rozenberszki on 2021. 02. 22..
//

#include "../include/FilterCloud.h"

FilterCloud::FilterCloud() : tfListener_(tfBuffer_) {

    nh_private = ros::NodeHandle("~");

    rgb_it_.reset(new image_transport::ImageTransport(nh_private));
    depth_it_.reset(new image_transport::ImageTransport(nh_private));

    loadParams();
    ROS_DEBUG("Subscribed to color topic: [%s] \n depth topic: [%s]\n and info topic: [%s],\n publishing to: [%s]",
              color_topic_name.c_str(), depth_topic_name.c_str(), camera_info_topic_name.c_str(),
              pointcloud_topic_name.c_str());

    std::string msg = "Subscribed to color topic: " + color_topic_name + " depth topic: " + depth_topic_name + ""
                                                                                                               " info topic: " +
                      camera_info_topic_name + "\n publishing   to: " + pointcloud_topic_name;
    std::cout << msg << std::endl;

    // depth image can use different transport.(e.g. compressedDepth)
    image_transport::TransportHints depth_hints(depth_encoding_, ros::TransportHints(), nh_private, depth_image_transport_param);
    sub_depth_.subscribe(*depth_it_, depth_topic_name, 10, depth_hints);

    // rgb uses normal ros transport hints.
    image_transport::TransportHints hints(rgb_encoding_, ros::TransportHints(), nh_private);
    sub_rgb_.subscribe(*rgb_it_, color_topic_name, queue_size_, hints);
    sub_info_.subscribe(nh_private, camera_info_topic_name, queue_size_);

    auto *mySyncronizer = new Synchronizer(SyncPolicy(queue_size_), sub_depth_, sub_rgb_, sub_info_);
    mySyncronizer->setMaxIntervalDuration(ros::Duration(max_interval_duration_));

    sync_.reset(mySyncronizer);
    sync_->registerCallback(boost::bind(&FilterCloud::imageCb, this, _1, _2, _3));

    pub_point_cloud_ = nh_private.advertise<PointCloud>(pointcloud_topic_name, 1);

    ros::Rate loop_rate(30);
    while (ros::ok())
    {
        // Wait for transform to arrive before processing
        // Skip processing if no transform available
        try {
            geometry_msgs::TransformStamped tmp_trans;
            tmp_trans = tfBuffer_.lookupTransform(target_frame_, source_frame_, ros::Time(0),
                                                          ros::Duration(0.5));

            if(tmp_trans.header.stamp.toNSec() != transformStamped_.header.stamp.toNSec()){
               transformStamped_ = tmp_trans;
            }
        } catch (tf2::TransformException &ex) {
            std::cout << "Skipping pointcloud generation, could NOT transform " << source_frame_ << " to " <<
                      target_frame_ << std::endl << ex.what() << std::endl;
        }

        ros::spinOnce();
        loop_rate.sleep();
    }

    ros::spin();
}

template<typename T>
void FilterCloud::convert(const sensor_msgs::ImageConstPtr &depth_msg,
                          const sensor_msgs::ImageConstPtr &rgb_msg, const PointCloud::Ptr &cloud_msg,
                          const image_geometry::PinholeCameraModel &model_, int red_offset, int green_offset,
                          int blue_offset, int color_step) {
    // Use correct principal point from calibration
    float center_x = model_.cx();
    float center_y = model_.cy();

    // Combine unit conversion (if necessary) with scaling by focal length for computing (X,Y)
    double unit_scaling = depth_image_proc::DepthTraits<T>::toMeters(T(1));
    float constant_x = unit_scaling / model_.fx();
    float constant_y = unit_scaling / model_.fy();
    float bad_point = std::numeric_limits<float>::quiet_NaN();

    const T *depth_row = reinterpret_cast<const T *>(&depth_msg->data[0]);
    int row_step = depth_msg->step / sizeof(T);
    const uint8_t *rgb = &rgb_msg->data[0];
    uint rgb_skip = rgb_msg->step - rgb_msg->width * color_step;

    sensor_msgs::PointCloud2Iterator<float> iter_x(*cloud_msg, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(*cloud_msg, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(*cloud_msg, "z");
    sensor_msgs::PointCloud2Iterator<uint8_t> iter_r(*cloud_msg, "r");
    sensor_msgs::PointCloud2Iterator<uint8_t> iter_g(*cloud_msg, "g");
    sensor_msgs::PointCloud2Iterator<uint8_t> iter_b(*cloud_msg, "b");
    sensor_msgs::PointCloud2Iterator<uint8_t> iter_a(*cloud_msg, "a");

    float total_points = cloud_msg->height * cloud_msg->width;
    int filtered_points = 0;

    for (int v = 0; v < int(cloud_msg->height); ++v, depth_row += row_step, rgb += rgb_skip) {
        for (int u = 0; u <
                        int(cloud_msg->width); ++u, rgb += color_step, ++iter_x, ++iter_y, ++iter_z, ++iter_a, ++iter_r, ++iter_g, ++iter_b) {
            T depth = depth_row[u];

            // Check for invalid depth measurements
            if (!depth_image_proc::DepthTraits<T>::valid(depth)) {
                *iter_x = *iter_y = *iter_z = bad_point;
                filtered_points++;
            }
            // Check for invalid RGB measurements
            int r = rgb[red_offset];
            int g = rgb[green_offset];
            int b = rgb[blue_offset];
            if (r == void_color[0] && g == void_color[1] && b == void_color[2]) {
                *iter_x = *iter_y = *iter_z = bad_point;
            } else {
                // Fill in XYZ
                *iter_x = (u - center_x) * depth * constant_x;
                *iter_y = (v - center_y) * depth * constant_y;
                *iter_z = depth_image_proc::DepthTraits<T>::toMeters(depth);
            }

            // Fill in color
            *iter_a = 255;
            *iter_r = rgb[red_offset];
            *iter_g = rgb[green_offset];
            *iter_b = rgb[blue_offset];
        }
    }
}

void FilterCloud::imageCb(const sensor_msgs::ImageConstPtr &depth_msg, const sensor_msgs::ImageConstPtr &rgb_msg_in,
                          const sensor_msgs::CameraInfoConstPtr &info_msg) {

    // Check for bad inputs
    if (depth_msg->header.frame_id != rgb_msg_in->header.frame_id) {
        std::cout << "Depth image frame id " << depth_msg->header.frame_id << " doesn't match RGB image frame id "
                  << rgb_msg_in->header.frame_id << std::endl << "Can't create pointclouds...";
        return;
    }

    // Check for tracking and localization consistency
    bool check1 = previousTransforms_.empty();
    bool check2 = filterTransformOutliers_;
    bool check3 = false;
    if(!check1){
        uint64_t this_stamp = transformStamped_.header.stamp.toNSec();
        uint64_t last_stamp = previousTransforms_.back().header.stamp.toNSec();
        bool check3 = (this_stamp==last_stamp);
    }
    bool check4 = filterByLocalizedPose(transformStamped_, previousTransforms_);

    if (check2 && (check1 || check3 ||  check4) ) {

        previousTransforms_.push_back(transformStamped_);

        while (previousTransforms_.size() > transformOutlierQueueSize_) {
            previousTransforms_.erase(previousTransforms_.begin());
        }
        ROS_WARN("Skipping depth image integration");
        consistent_msg_ = nullptr;
        return;
    }

    // Update camera model
    image_geometry::PinholeCameraModel model_;
    model_.fromCameraInfo(info_msg);

    // Check if the input image has to be resized
    sensor_msgs::ImageConstPtr rgb_msg = rgb_msg_in;
    if (depth_msg->width != rgb_msg->width || depth_msg->height != rgb_msg->height) {

        sensor_msgs::CameraInfo info_msg_tmp = *info_msg;
        info_msg_tmp.width = depth_msg->width;
        info_msg_tmp.height = depth_msg->height;
        float ratio = float(depth_msg->width) / float(rgb_msg->width);
        info_msg_tmp.K[0] *= ratio;
        info_msg_tmp.K[2] *= ratio;
        info_msg_tmp.K[4] *= ratio;
        info_msg_tmp.K[5] *= ratio;
        info_msg_tmp.P[0] *= ratio;
        info_msg_tmp.P[2] *= ratio;
        info_msg_tmp.P[5] *= ratio;
        info_msg_tmp.P[6] *= ratio;
        model_.fromCameraInfo(info_msg_tmp);

        cv_bridge::CvImageConstPtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvShare(rgb_msg, rgb_msg->encoding);
        }
        catch (cv_bridge::Exception &e) {
            ROS_ERROR_THROTTLE(5, "cv_bridge exception: %s", e.what());
            return;
        }
        cv_bridge::CvImage cv_rsz;
        cv_rsz.header = cv_ptr->header;
        cv_rsz.encoding = cv_ptr->encoding;
        cv::resize(cv_ptr->image.rowRange(0, depth_msg->height / ratio), cv_rsz.image,
                   cv::Size(depth_msg->width, depth_msg->height));
        if ((rgb_msg->encoding == enc::RGB8) || (rgb_msg->encoding == enc::BGR8) || (rgb_msg->encoding == enc::MONO8))
            rgb_msg = cv_rsz.toImageMsg();
        else
            rgb_msg = cv_bridge::toCvCopy(cv_rsz.toImageMsg(), enc::RGB8)->toImageMsg();

        //NODELET_ERROR_THROTTLE(5, "Depth resolution (%ux%u) does not match RGB resolution (%ux%u)",
        //                       depth_msg->width, depth_msg->height, rgb_msg->width, rgb_msg->height);
        //return;
    } else
        rgb_msg = rgb_msg_in;

    // Supported color encodings: RGB8, BGR8, MONO8
    int red_offset, green_offset, blue_offset, color_step;
    if (rgb_msg->encoding == enc::RGB8) {
        red_offset = 0;
        green_offset = 1;
        blue_offset = 2;
        color_step = 3;
    } else if (rgb_msg->encoding == enc::BGR8) {
        red_offset = 2;
        green_offset = 1;
        blue_offset = 0;
        color_step = 3;
    } else if (rgb_msg->encoding == enc::MONO8) {
        red_offset = 0;
        green_offset = 0;
        blue_offset = 0;
        color_step = 1;
    } else {
        try {
            rgb_msg = cv_bridge::toCvCopy(rgb_msg, enc::RGB8)->toImageMsg();
        }
        catch (cv_bridge::Exception &e) {
            ROS_ERROR_THROTTLE(5, "Unsupported encoding [%s]: %s", rgb_msg->encoding.c_str(), e.what());
            std::cout<<"Unsupported encoding: "<< rgb_msg->encoding<<std::endl;
            return;
        }
        red_offset = 0;
        green_offset = 1;
        blue_offset = 2;
        color_step = 3;
    }

    // Allocate new point cloud message
    PointCloud::Ptr cloud_msg(new PointCloud);
    cloud_msg->header = depth_msg->header; // Use depth image time stamp
    cloud_msg->header.frame_id = pointcloud_frame_; // Changing the frame
    cloud_msg->height = depth_msg->height;
    cloud_msg->width = depth_msg->width;
    cloud_msg->is_dense = false;
    cloud_msg->is_bigendian = false;

    sensor_msgs::PointCloud2Modifier pcd_modifier(*cloud_msg);
    pcd_modifier.setPointCloud2FieldsByString(2, "xyz", "rgb");

    if (depth_msg->encoding == enc::TYPE_16UC1) {
        convert<uint16_t>(depth_msg, rgb_msg, cloud_msg, model_, red_offset, green_offset, blue_offset, color_step);
    } else if (depth_msg->encoding == enc::TYPE_32FC1) {
        convert<float>(depth_msg, rgb_msg, cloud_msg, model_, red_offset, green_offset, blue_offset, color_step);
    } else {
        ROS_ERROR_THROTTLE(5, "Depth image has unsupported encoding [%s]", depth_msg->encoding.c_str());
        return;
    }

    if(previousTransforms_.size()>0){
        uint64_t this_stamp = transformStamped_.header.stamp.toNSec();
        uint64_t last_stamp = previousTransforms_.back().header.stamp.toNSec();
        if(this_stamp != last_stamp){
            previousTransforms_.push_back(transformStamped_);

            if(consistent_msg_ != nullptr){
                pub_point_cloud_.publish(consistent_msg_);
            }
            consistent_msg_ =  cloud_msg;
        }

    }else{
        previousTransforms_.push_back(transformStamped_);
    }

    while (previousTransforms_.size() > transformOutlierQueueSize_) {
        previousTransforms_.erase(previousTransforms_.begin());
    }
}

void FilterCloud::loadParams() {

    //Reading the params

    // Queue size for images and depth images, important to drop if drop if processing takes longer than FPS
    nh_private.param("queue_size", queue_size_, 20);

    // Sets the limit in seconds when we accept synced messages as corresponding
    nh_private.param("max_interval_duration", max_interval_duration_, 0.2);

    // For filtering this color from the pointcloud - saves computation and increases prediction accuracy for the mesh
    nh_private.param("void_rgb_color", void_color, {0, 0, 0});

    // ROS topic names
    nh_private.param("color_topic_name",
                     color_topic_name,
                     color_topic_name);
    nh_private.param("rgb_encoding",
                     rgb_encoding_,
                     rgb_encoding_);
    nh_private.param("camera_info_topic_name",
                     camera_info_topic_name,
                     camera_info_topic_name);
    nh_private.param("depth_topic_name",
                     depth_topic_name,
                     depth_topic_name);
    nh_private.param("depth_encoding",
                     depth_encoding_,
                     depth_encoding_);
    nh_private.param("pointcloud_topic_name",
                     pointcloud_topic_name,
                     pointcloud_topic_name);

    // We are filtering false localizations
    // If we transform a metric cloud with thw wrong pose, reconstruction can be noisy
    nh_private.param("filter_transform_outliers",
                     filterTransformOutliers_,
                     filterTransformOutliers_);
    nh_private.param("transform_outlier_queueSize",
                     transformOutlierQueueSize_,
                     transformOutlierQueueSize_);

    //The frames for movement detection
    nh_private.param("source_frame",
                     source_frame_,
                     source_frame_);

    nh_private.param("target_frame",
                     target_frame_,
                     target_frame_);

    //The published cloud will be under this frame
    nh_private.param("pointcloud_frame",
                     pointcloud_frame_,
                     pointcloud_frame_);

    // The limits for the trajectory consistency
    nh_private.param("translation_treshold",
                     translation_treshold_,
                     translation_treshold_);


    nh_private.param("rotation_treshold",
                     rotation_treshold_,
                     rotation_treshold_);

    nh_private.param("timestamp_difference_limit",
                     timestamp_difference_limit_,
                     timestamp_difference_limit_);

}


bool FilterCloud::filterByLocalizedPose(geometry_msgs::TransformStamped &latestTransform,
                                        std::vector<geometry_msgs::TransformStamped> &previousTransforms) {
    bool should_filter = false;

    std::vector<geometry_msgs::TransformStamped> transform_vec = previousTransforms;
    transform_vec.push_back(latestTransform);

    for (int i = 1; i < transform_vec.size(); i++) {

        geometry_msgs::TransformStamped previous = transform_vec[i - 1];
        geometry_msgs::TransformStamped current = transform_vec[i];

        double time_diff = static_cast<double>(current.header.stamp.toNSec() - previous.header.stamp.toNSec()) / 1e9;
        // Stop if there is a huge time difference from the last measurement - relocalization can be still false
        if (time_diff > timestamp_difference_limit_) {
            std::cout<<"time_diff="<<time_diff<<std::endl;
            should_filter = true;
            return should_filter;
        }

        double disp = std::sqrt(std::pow(
                (current.transform.translation.x - previous.transform.translation.x) +
                        (current.transform.translation.y - previous.transform.translation.y) +
                        (current.transform.translation.z - previous.transform.translation.z),
                2));

        // Skip if the movement is inconsistent
        if (time_diff > 0 && std::abs(disp) > translation_treshold_) {
            std::cout<<"translation="<<disp<<std::endl;
            should_filter = true;
            return should_filter;
        }

        Eigen::Quaterniond prev_quat(previous.transform.rotation.w, previous.transform.rotation.x,
                                     previous.transform.rotation.y, previous.transform.rotation.z);
        Eigen::Quaterniond current_quat(current.transform.rotation.w, current.transform.rotation.x,
                                        current.transform.rotation.y, current.transform.rotation.z);

        Eigen::Quaterniond angular_diff = current_quat * prev_quat.inverse();
        angular_diff.normalize();

        double rot_angle = 2 * std::acos(angular_diff.w());

        // Skip if the rotation is inconsistent
        if (time_diff > 0 && std::abs(rot_angle) > rotation_treshold_) {
            should_filter = true;
            std::cout<<"rot_angle="<<rot_angle<<std::endl;
            return should_filter;
        }
    }

    return should_filter;
}

FilterCloud::~FilterCloud() = default;


/*

int main(int argc, char **argv) {

    ros::init(argc, argv, "cloud_filter_node");

    google::InitGoogleLogging(argv[0]);
    google::ParseCommandLineFlags(&argc, &argv, false);
    google::InstallFailureSignalHandler();

    FilterCloud filterCloud;

    return 0;
}
*/
