#pragma once

#include "twist_manager.h"

// 构造函数
TwistManager::TwistManager(ros::NodeHandle& nh, const std::string& config_file) {

    // 解析 YAML 文件
    if (!ParseYaml(config_file)) {
        ROS_ERROR("Failed to parse configuration file.");
        ros::shutdown();
    }

    // 初始化订阅和发布者
    radar_sub_ = nh.subscribe<sensor_msgs::PointCloud2>(radar_params_.topic, 1000, &TwistManager::RadarCallback, this);
    event_sub_ = nh.subscribe<dvs_msgs::EventArray>(event_params_.topic, 1000, &TwistManager::EventCallback, this);
    pub_twist_ = nh.advertise<geometry_msgs::TwistWithCovarianceStamped>("radar/twist", 1000);
    pub_inlier_ = nh.advertise<sensor_msgs::PointCloud2>("radar/inliers", 1000);
    pub_outlier_ = nh.advertise<sensor_msgs::PointCloud2>("radar/outliers", 1000);

    // 初始化事件检测器
    event_detector_ = new EventFlowDetector(event_params_, radar_event_params_);
}

// 析构函数
TwistManager::~TwistManager() {
    delete event_detector_;
    google::ShutdownGoogleLogging();
}

// 解析 YAML 配置文件
bool TwistManager::ParseYaml(const std::string& filename) {
    try {
        YAML::Node config = YAML::LoadFile(filename);

        // 解析事件参数
        event_params_.topic = config["event"]["topic"].as<std::string>();
        event_params_.freq = config["event"]["freq"].as<int>();
        event_params_.fx = config["event"]["fx"].as<double>();
        event_params_.fy = config["event"]["fy"].as<double>();
        event_params_.cx = config["event"]["cx"].as<double>();
        event_params_.cy = config["event"]["cy"].as<double>();
        auto resolution = config["event"]["resolution"].as<std::array<int, 2>>();
        event_params_.resolution[0] = resolution[0];
        event_params_.resolution[1] = resolution[1];
        event_params_.deltaT = config["event"]["deltaT"].as<double>();
        event_params_.t1 = config["event"]["t1"].as<double>();
        event_params_.mesh_size = config["event"]["mesh_size"].as<int>();
        event_params_.timeshift_cam_imu = config["event"]["timeshift_cam_imu"].as<double>();

        // 解析雷达参数
        radar_params_.topic = config["radar"]["topic"].as<std::string>();

        // 解析雷达事件参数
        auto T_re_list = config["radar_event"]["T_re"];
        for (int i = 0; i < 4; ++i) {
            for (int j = 0; j < 4; ++j) {
                radar_event_params_.T_re(i, j) = T_re_list[i][j].as<double>();
            }
        }
    } catch (const std::exception& e) {
        std::cerr << e.what() << '\n';
        return false;
    }
    return true;
}

// 雷达回调函数
void TwistManager::RadarCallback(const sensor_msgs::PointCloud2::ConstPtr& radar_msgs_ptr) {
    if (radar_msgs_ptr == nullptr) {
        return;
    }

    geometry_msgs::TwistWithCovarianceStamped twist;
    Eigen::Vector3d v_r, sigma_v_r;
    sensor_msgs::PointCloud2 inlier_radar_msg, outlier_radar_msg;

    clock_t start_ms = clock();
    if (estimator_.estimate(*radar_msgs_ptr, v_r, sigma_v_r, inlier_radar_msg, outlier_radar_msg)) {
        clock_t end_ms = clock();
        double time_used = double(end_ms - start_ms) / CLOCKS_PER_SEC;
        egovel_time_.push_back(time_used);

        twist.header.stamp = radar_msgs_ptr->header.stamp;
        twist.header.frame_id = "radar";
        twist.twist.twist.linear.x = v_r.x();
        twist.twist.twist.linear.y = v_r.y();
        twist.twist.twist.linear.z = v_r.z();
        twist.twist.covariance[0] = std::pow(sigma_v_r.x(), 2);
        twist.twist.covariance[7] = std::pow(sigma_v_r.y(), 2);
        twist.twist.covariance[14] = std::pow(sigma_v_r.z(), 2);

        // pub_twist_.publish(twist);
        // pub_outlier_.publish(outlier_radar_msg);

        event_detector_->radar_doppler_velocity.push_back(twist);
        event_detector_->radar_inliers.push_back(inlier_radar_msg);    
    } else {
        ROS_WARN("Radar Estimation Failed!");
    }
}

// 事件回调函数
void TwistManager::EventCallback(const dvs_msgs::EventArray::ConstPtr& event_msgs_ptr) {
    if (event_msgs_ptr == nullptr) {
        return;
    }

    event_detector_->event_stream.push_back(*event_msgs_ptr);
}

// 主循环
void TwistManager::Spin() {
    geometry_msgs::TwistWithCovarianceStamped twist;
    std::vector<cv::Point2d> points;
    while (ros::ok()) {
        // 前端检测
        if(event_detector_->Detector())
        {
            twist = event_detector_->GetTwist();

            pub_twist_.publish(twist);
            pub_inlier_.publish(event_detector_->GetInliers());

            // 后端优化
            // estimator_->Estimate(twist, inlier_radar_msg);
        }
        ros::spinOnce();
    }
}

