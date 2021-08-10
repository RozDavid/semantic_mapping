//
// Created by David Rozenberszki on 2021. 02. 26..
//

#include "CloudSegmentation.h"
#include <glog/logging.h>

int main(int argc, char **argv) {

    ros::init(argc, argv, "instance_segmentation_node");

    google::InitGoogleLogging(argv[0]);
    google::ParseCommandLineFlags(&argc, &argv, false);
    google::InstallFailureSignalHandler();

    CloudSegmentation cloudSegmentation;

    return 0;
}