//
// Created by David Rozenberszki on 2021. 02. 28..
//

#ifndef KIMERA_INTERFACE_CLASSINSTANCE_H
#define KIMERA_INTERFACE_CLASSINSTANCE_H

#include <pcl/point_types.h>
#include <Eigen/Dense>
#include "kimera_semantics/color.h"

struct ClassInstance{
    int num_of_points;
    Eigen::Vector3f centroid;
    Eigen::Vector3f dimensions;
    int class_label;
    kimera::HashableColor color;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr points;
};

#endif //KIMERA_INTERFACE_CLASSINSTANCE_H
