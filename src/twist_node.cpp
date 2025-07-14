#pragma once

#include "twist_manager.h"

int main(int argc, char** argv) {
    // 初始化日志
    // google::InitGoogleLogging(argv[0]);
    // FLAGS_log_dir = "/home/hao/Desktop/radar-event-new/src/TwistEstimator/glog_log";

    ros::init(argc, argv, "twist_node");
    ros::NodeHandle nh;

    // const std::string config_file = "/home/hao/Desktop/radar-event-new/src/TwistEstimator/config/params.yaml";

    std::string config_file = "/home/hao/Desktop/twist_ws/src/TwistEstimator/config/params.yaml";
    nh.getParam("config", config_file)

    TwistManager processor(nh, config_file);
    processor.Spin();

    return 0;
}
