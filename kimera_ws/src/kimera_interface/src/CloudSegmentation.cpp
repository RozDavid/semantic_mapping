//
// Created by David Rozenberszki on 2021. 02. 26..
//

#include "CloudSegmentation.h"

CloudSegmentation::CloudSegmentation() : tfListener_(tfBuffer_) {

    // Initialize nodehandle
    nh_private = ros::NodeHandle("~");

    // Initialize the full cloud that will later store the subscribed messages
    rgb_cloud_ = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>);

    ROS_DEBUG("Started instance segmentation node...");

    // Load parameters from the ROS server
    loadParams();

    // We need the mapping to convert colors to labels to apply filtering
    semantic_label_to_color_map_ = kimera::make_unique<kimera::SemanticLabel2Color>(label2color_mapping_path_);

    for (int label : target_label_indexes_) {
        kimera::HashableColor label_color = semantic_label_to_color_map_->getColorFromSemanticLabel(label);
        target_colors_.push_back(std::make_pair(label, label_color));
    }

    //Subscriber for the full TSDF cloud and publisher of the instance bounding boxes
    pointcloud_subscriber_ = nh_private.subscribe(pointcloud_topic_, queue_size_,
                                                  &CloudSegmentation::pointcloudCallback, this);
    bounding_box_publisher_ = nh_private.advertise<jsk_recognition_msgs::BoundingBoxArray>(bounding_box_topic_, 10);

    if (align_pointcloud_ && aligned_pointcloud_topic_ != "") {
        aligned_pointcloud_publisher_ = nh_private.advertise<PointCloud>(aligned_pointcloud_topic_, 1);
    }

    if (instance_pointcloud_topic_ != "") {
        instance_pointcloud_publisher_ = nh_private.advertise<kimera_interface::ClassInstanceArray>(instance_pointcloud_topic_, 1);
    }

    //We need to define a target rate for pointcloud processing,
    // callbacks cannot be limited, but processing can be done by desired rate like this
    ros::Rate rate(target_frequency_);
    while (ros::ok()) {
        if (!rgb_cloud_->empty()){
            processCloud();
        }

        ros::spinOnce();
        rate.sleep();
    }

}

void CloudSegmentation::alignPointCloudToFloor(pcl::PointCloud<pcl::PointXYZRGB>::Ptr in_cloud_rgb, pcl::PointCloud<pcl::PointXYZRGB>::Ptr out_cloud_rgb) {
    // Extract floor points
    const int floor_id = 5;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_floor_rgb(new pcl::PointCloud<pcl::PointXYZRGB>);
    kimera::HashableColor label_color = semantic_label_to_color_map_->getColorFromSemanticLabel(floor_id);
    filterbyColor(label_color, in_cloud_rgb, cloud_floor_rgb);

    // Remove RGB information
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::copyPointCloud(*cloud_floor_rgb, *cloud);

    if (cloud_floor_rgb->size() == 0) {
        return;
    }

    std::cout << "[Instance segmentation] Aligning point cloud to " << cloud_floor_rgb->size() << " floor points..." << std::endl;

    // Segment plane from floor points
    pcl::SACSegmentation<pcl::PointXYZ> sac;
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane(new pcl::PointCloud<pcl::PointXYZ>);

    sac.setInputCloud(cloud);
    sac.setMethodType(pcl::SAC_RANSAC);
    sac.setModelType(pcl::SACMODEL_PLANE);
    sac.setDistanceThreshold(15); // Distance need to be adjusted according to the obj
    sac.setMaxIterations(100);
    sac.setProbability(0.95);
    sac.segment(*inliers, *coefficients);

    // Plane Model: ax+by+cz+d=0; saved in *coefficients

    float a = coefficients->values[0];
    float b = coefficients->values[1];
    float c = coefficients->values[2];
    // float d = coefficients->values[3];

    // std::cout << "Floor plane coefficients: [" << a << ", " << b << ", " << c << ", " << d << "]" << std::endl;

    // Generate transformation matrix
    Eigen::Vector3f A(a,b,c), B(0,0,1);
    Eigen::Matrix3f rot_mx = Eigen::Quaternionf().setFromTwoVectors(A,B).toRotationMatrix();
    Eigen::Matrix4f transform_mx = Eigen::Matrix4f::Identity();
    transform_mx.block(0,0,3,3) = rot_mx;

    // std::cout << "Floor transformation:\n" << transform_mx << std::endl;

    // Execute the transformation
    pcl::transformPointCloud(*in_cloud_rgb, *out_cloud_rgb, transform_mx);

    // Send the transformed pointcloud back
    if (aligned_pointcloud_topic_ != "") {
        aligned_pointcloud_publisher_.publish(out_cloud_rgb);
    }
}

void CloudSegmentation::processCloud() {

    std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();

    if (align_pointcloud_) {
        // Align point cloud to the floor points
        alignPointCloudToFloor(rgb_cloud_, rgb_cloud_);
    }

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::copyPointCloud(*rgb_cloud_, *temp_cloud);

    jsk_recognition_msgs::BoundingBoxArray bounding_boxes;

    kimera_interface::ClassInstanceArray class_instance_arr;
    class_instance_arr.header.frame_id = cloud_frame_id_;
    class_instance_arr.header.stamp = cloud_stamp_;

    for(auto label_color_pair : target_colors_){

        std::vector<ClassInstance> objectInstances;
        int label_index = label_color_pair.first;
        kimera::HashableColor label_color = label_color_pair.second;

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);
        filterbyColor(label_color, temp_cloud, cloud_filtered);
        segmentCloud(cloud_filtered, objectInstances, label_index, label_color);
        generateBoundingBoxes(label_index, objectInstances, bounding_boxes);
        generateInstancePointCloud(objectInstances, class_instance_arr);
    }

    bounding_boxes.header.frame_id = cloud_frame_id_;
    bounding_boxes.header.stamp = cloud_stamp_;
    bounding_box_publisher_.publish(bounding_boxes);

    if (instance_pointcloud_topic_ != "") {
        instance_pointcloud_publisher_.publish(class_instance_arr);
    }

    std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
    std::cout << "[Instance segmentation] Found "<< bounding_boxes.boxes.size() <<" instances in " << std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count() << "[ms]" << std::endl;

}

void CloudSegmentation::loadParams() {

    //Reading the params
    std::string label2color_path = "";
    nh_private.param("label2color_mapping_path", label2color_mapping_path_, label2color_path);

    std::string pointcloud_topic = "/cloud_pcd";
    nh_private.param("pointcloud_topic", pointcloud_topic_, pointcloud_topic);

    std::string aligned_pointcloud_topic = "/aligned_pcd";
    nh_private.param("aligned_pointcloud_topic", aligned_pointcloud_topic_, aligned_pointcloud_topic);
    nh_private.param("align_pointcloud", align_pointcloud_, false);

    std::string instance_pointcloud_topic = "/instance_pcd";
    nh_private.param("instance_pointcloud_topic", instance_pointcloud_topic_, instance_pointcloud_topic);

    std::string bounding_box_topic = "semantics/instance_bounding_boxes";
    nh_private.param("bounding_box_topic", bounding_box_topic_, bounding_box_topic);

    // We need to have the given labels values -1 for target label as count starts from 1 with void labels in the color mapping
    std::vector<int> target_label_indexes{ 12 };
    nh_private.param("target_label_indexes", target_label_indexes_, target_label_indexes);

    nh_private.param("cluster_tolerance", cluster_tolerance_, 0.1);
    nh_private.param("min_cluster_size", min_cluster_size_, 5);
    nh_private.param("use_difference_of_normals", use_difference_of_normals_, true);
    nh_private.param("small_don_scale", small_don_scale_, 0.1);
    nh_private.param("large_don_scale", large_don_scale_, 1.0);
    nh_private.param("don_treshold", don_treshold_, 0.1);

    nh_private.param("target_frequency", target_frequency_, 0.5);
    nh_private.param("queue_size", queue_size_, 1);

    std::string source_frame = "hp_laptop_odometry_frame";
    nh_private.param("source_frame", source_frame_, source_frame);

    std::string target_frame = "hp_laptop_color_optical_frame";
    nh_private.param("target_frame", target_frame_, target_frame);

    // We need to have this -1 as count starts from 1 with void labels in the color mapping
    //target_label_index_ -= 1;

}

void CloudSegmentation::pointcloudCallback(const sensor_msgs::PointCloud2ConstPtr &cloud_msg) {

    // No transformation segmentation at this point
    //How could we increase performance with this

    /*try{
        transformStamped_ = tfBuffer_.lookupTransform(target_frame_, source_frame_, cloud_msg->header.stamp,
                                                    ros::Duration(1.0));
    } catch (tf2::TransformException &ex) {
        ROS_WARN_THROTTLE(5, "Could NOT transform %s to %s: %s", source_frame_.c_str(), target_frame_.c_str(), ex.what());
    }*/

    pcl::fromROSMsg(*cloud_msg, *rgb_cloud_);
    cloud_frame_id_ = cloud_msg->header.frame_id;
}

void
CloudSegmentation::filterbyColor(kimera::HashableColor targetcolor, pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud,
                                 pcl::PointCloud<pcl::PointXYZRGB>::Ptr filtered_cloud) {

    pcl::PackedRGBComparison<pcl::PointXYZRGB>::Ptr
            red_condition_g(new pcl::PackedRGBComparison<pcl::PointXYZRGB>("r", pcl::ComparisonOps::EQ, targetcolor.r));
    pcl::PackedRGBComparison<pcl::PointXYZRGB>::Ptr
            green_condition_g(
            new pcl::PackedRGBComparison<pcl::PointXYZRGB>("g", pcl::ComparisonOps::EQ, targetcolor.g));
    pcl::PackedRGBComparison<pcl::PointXYZRGB>::Ptr
            blue_condition_g(
            new pcl::PackedRGBComparison<pcl::PointXYZRGB>("b", pcl::ComparisonOps::EQ, targetcolor.b));

    pcl::ConditionAnd<pcl::PointXYZRGB>::Ptr color_cond(new pcl::ConditionAnd<pcl::PointXYZRGB>());
    color_cond->addComparison(red_condition_g);

    color_cond->addComparison(green_condition_g);

    color_cond->addComparison(blue_condition_g);


    // Build the filter
    pcl::ConditionalRemoval<pcl::PointXYZRGB> color_filter;
    color_filter.setInputCloud(input_cloud);
    color_filter.setCondition(color_cond);
    color_filter.filter(*filtered_cloud);

}

void CloudSegmentation::segmentCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr filteredCloud,
                                     std::vector<ClassInstance> &instances, int &target_label_index,
                                     kimera::HashableColor &target_label_color) {

    if (filteredCloud->size() == 0) {
        return;
    }

    // Create the KdTree object for the search method of the extraction
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>);
    tree->setInputCloud(filteredCloud);

    // create the extraction object for the clusters
    std::vector<pcl::PointIndices> cluster_indices;

    if(use_difference_of_normals_){
        if (small_don_scale_ >= large_don_scale_)
        {
            ROS_ERROR("Error: Large scale must be > small scale!");
        }

        // Compute normals using both small and large scales at each point
        pcl::NormalEstimationOMP<pcl::PointXYZRGB, pcl::PointNormal> ne;
        ne.setInputCloud (filteredCloud);
        ne.setSearchMethod (tree);

        /**
       * NOTE: setting viewpoint is very important, so that we can ensure
       * normals are all pointed in the same direction!
       */
        ne.setViewPoint (std::numeric_limits<float>::max (), std::numeric_limits<float>::max (), std::numeric_limits<float>::max ());

        // calculate normals with the small scale
        pcl::PointCloud<pcl::PointNormal>::Ptr normals_small_scale (new pcl::PointCloud<pcl::PointNormal>);

        ne.setRadiusSearch (small_don_scale_);
        ne.compute (*normals_small_scale);

        // calculate normals with the large scale
        pcl::PointCloud<pcl::PointNormal>::Ptr normals_large_scale (new pcl::PointCloud<pcl::PointNormal>);

        ne.setRadiusSearch (large_don_scale_);
        ne.compute (*normals_large_scale);

        // Create output cloud for DoN results
        pcl::PointCloud<pcl::PointNormal>::Ptr don_cloud (new pcl::PointCloud<pcl::PointNormal>);
        copyPointCloud (*filteredCloud, *don_cloud);

        pcl::DifferenceOfNormalsEstimation<pcl::PointXYZRGB, pcl::PointNormal, pcl::PointNormal> don;
        don.setInputCloud (filteredCloud);
        don.setNormalScaleLarge (normals_large_scale);
        don.setNormalScaleSmall (normals_small_scale);


        if (!don.initCompute ())
        {
            ROS_ERROR("Error: Could not initialize DoN feature operator");
        }

        // Compute DoN
        don.computeFeature (*don_cloud);

        // Build the condition for filtering
        pcl::ConditionOr<pcl::PointNormal>::Ptr range_cond (
                new pcl::ConditionOr<pcl::PointNormal> ()
        );
        range_cond->addComparison (pcl::FieldComparison<pcl::PointNormal>::ConstPtr (
                new pcl::FieldComparison<pcl::PointNormal> ("curvature", pcl::ComparisonOps::GT, don_treshold_))
        );
        // Build the filter
        pcl::ConditionalRemoval<pcl::PointNormal> condrem;
        condrem.setCondition (range_cond);
        condrem.setInputCloud (don_cloud);

        pcl::PointCloud<pcl::PointNormal>::Ptr doncloud_filtered (new pcl::PointCloud<pcl::PointNormal>);

        // Apply filter
        condrem.filter (*doncloud_filtered);

        pcl::search::KdTree<pcl::PointNormal>::Ptr segtree (new pcl::search::KdTree<pcl::PointNormal>);
        segtree->setInputCloud (doncloud_filtered);

        pcl::EuclideanClusterExtraction<pcl::PointNormal> ec;

        ec.setClusterTolerance (cluster_tolerance_);
        ec.setMinClusterSize (min_cluster_size_);
        ec.setMaxClusterSize (100000);
        ec.setSearchMethod (segtree);
        ec.setInputCloud (doncloud_filtered);
        ec.extract (cluster_indices);

    }else{

        // specify euclidean cluster parameters
        pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
        ec.setClusterTolerance(cluster_tolerance_); // 2cm
        ec.setMinClusterSize(min_cluster_size_);
        ec.setMaxClusterSize(25000);
        ec.setSearchMethod(tree);
        ec.setInputCloud(filteredCloud);
        // exctract the indices pertaining to each cluster and store in a vector of pcl::PointIndices
        ec.extract(cluster_indices);

    }


    // Here, cluster_indices is a vector of indices for each cluster. iterate through each indices
    for (const auto & cluster_ind : cluster_indices) {

        auto cluster = new pcl::PointCloud<pcl::PointXYZRGB>;
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr clusterPtr(cluster);

        // Assign each point corresponding to this cluster in filteredCloud
        for (auto pit = cluster_ind.indices.begin(); pit != cluster_ind.indices.end(); ++pit) {
            clusterPtr->points.push_back(filteredCloud->points[*pit]);
        }

        pcl::PointXYZRGB minPt, maxPt;
        pcl::getMinMax3D(*clusterPtr, minPt, maxPt);

        ClassInstance instance;
        instance.num_of_points = clusterPtr->size();
        instance.class_label = target_label_index;
        instance.color = target_label_color;
        instance.centroid = Eigen::Vector3f((maxPt.x + minPt.x) / 2,
                                            (maxPt.y + minPt.y) / 2,
                                            (maxPt.z + minPt.z) / 2);
        instance.dimensions = Eigen::Vector3f((maxPt.x - minPt.x),
                                              (maxPt.y - minPt.y),
                                              (maxPt.z - minPt.z));

        instance.points = clusterPtr;
        instances.push_back(instance);
    }
}

void CloudSegmentation::generateBoundingBoxes(int label_index, const std::vector<ClassInstance>& instances,
                                              jsk_recognition_msgs::BoundingBoxArray& bounding_boxes) {

    for (int i = 0; i < instances.size(); i++) {
        jsk_recognition_msgs::BoundingBox box;
        box.label = label_index;
        box.value = i;
        box.pose.position.x = instances[i].centroid.x();
        box.pose.position.y = instances[i].centroid.y();
        box.pose.position.z = instances[i].centroid.z();
        box.dimensions.x = instances[i].dimensions.x();
        box.dimensions.y = instances[i].dimensions.y();
        box.dimensions.z = instances[i].dimensions.z();

        box.header.frame_id = cloud_frame_id_;
        box.header.stamp = cloud_stamp_;
        bounding_boxes.boxes.push_back(box);

    }
}

void CloudSegmentation::generateInstancePointCloud(const std::vector<ClassInstance>& instances,
                                                   kimera_interface::ClassInstanceArray& instance_cloud) {

    for (int i = 0; i < instances.size(); i++) {
        kimera_interface::ClassInstance ci;
        ci.centroid.x = instances[i].centroid.x();
        ci.centroid.y = instances[i].centroid.y();
        ci.centroid.z = instances[i].centroid.z();
        ci.dimensions.x = instances[i].dimensions.x();
        ci.dimensions.y = instances[i].dimensions.y();
        ci.dimensions.z = instances[i].dimensions.z();
        ci.instance_id = i + 1;
        ci.label_id = instances[i].class_label;
        pcl::toROSMsg(*instances[i].points.get(), ci.pc);
        instance_cloud.instances.push_back(ci);
    }
}

CloudSegmentation::~CloudSegmentation() = default;