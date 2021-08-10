//
// Created by David Rozenberszki on 2021. 02. 22..
//

#include "../include/FilterCloud.h"


int main(int argc, char **argv) {

    ros::init(argc, argv, "cloud_filter_node");

    google::InitGoogleLogging(argv[0]);
    google::ParseCommandLineFlags(&argc, &argv, false);
    google::InstallFailureSignalHandler();

    FilterCloud filterCloud;

    return 0;
}