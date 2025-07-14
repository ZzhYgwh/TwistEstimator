#pragma once

#include "radar_ego_velocity_estimator/radar_ego_velocity_estimator.h"
#include "event_flow_detector/event_flow_detector.h"

#include <ros/ros.h>
#include <Eigen/Dense>
#include <yaml-cpp/yaml.h>
#include <geometry_msgs/TwistWithCovarianceStamped.h>

#include <map>

// #include <glog/logging.h>

class TwistManager {
public:
    TwistManager(ros::NodeHandle& nh, const std::string& config_file);
    ~TwistManager();

    void RadarCallback(const sensor_msgs::PointCloud2::ConstPtr& radar_msgs_ptr);
    void EventCallback(const dvs_msgs::EventArray::ConstPtr& event_msgs_ptr);
    bool ParseYaml(const std::string& filename);
    void Spin();

private:
    EventFlowDetector* event_detector_;

    Estimator* estimator_;

    ros::Publisher pub_twist_;
    ros::Publisher pub_inlier_;
    ros::Publisher pub_outlier_;
    ros::Subscriber radar_sub_;
    ros::Subscriber event_sub_;
    
    rio::RadarEgoVelocityEstimator estimator_;
    std::vector<double> egovel_time_;
    std::fstream test_file_;

    EventParams event_params_;
    RadarParams radar_params_;
    RadarEventParams radar_event_params_;
};