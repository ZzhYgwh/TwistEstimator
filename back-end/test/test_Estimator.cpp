#pragma once

// #include "../../include/radar-event/EventDetector.hpp"

#include "radar_ego_velocity_estimator/radar_ego_velocity_estimator.h"
#include "event_flow_detector/event_flow_detector.h"

#include "estimator/TwistEstimator.h"



void RadarCallback(const sensor_msgs::PointCloud2::ConstPtr radar_msgs_ptr);

void EventCallback(const dvs_msgs::EventArray::ConstPtr event_msgs_ptr);

bool ParseYaml(const std::string& filename, EventParams& event, RadarParams& radar, 
                    RadarEventParams& radar_event, bool& show_events, double& smooth);

EventFlowDetector* event_detector_;
// TwistEstimator* event_detector_;
TwistEstimator* estimator_;

ros::Publisher pub_twist_;
ros::Publisher pub_inlier_;
ros::Publisher pub_outlier_;
ros::Publisher pub_flow_img_;
int main(int argc,char ** argv)
{
    google::InitGoogleLogging(argv[0]);
    // 指定日志文件的保存路径
    FLAGS_log_dir = "/home/hao/Desktop/radar-event-new/src/TwistEstimator/../glog_log";

    ros::init(argc,argv,"test_detector");
    ros::NodeHandle nh;

    EventParams event;
    RadarParams radar;
    RadarEventParams radar_event;
    radar_event.T_re = Eigen::Matrix4d::Identity();  // Initialize to identity

    // const std::string filename = "/home/hao/Desktop/radar-event-new/src/TwistEstimator/config/params.yaml";
    const std::string filename = "/home/hao/Desktop/radar-event-new/src/TwistEstimator/config/demo.yaml";
    bool show_events = false;
    double smooth = 0.1;
    ParseYaml(filename, event, radar, radar_event, show_events, smooth);
    // std::cout << "smooth = " << smooth << std::endl;

    ros::Subscriber radar_sub_ = nh.subscribe<sensor_msgs::PointCloud2>(radar.topic, 1000, RadarCallback);
    ros::Subscriber event_sub_ = nh.subscribe<dvs_msgs::EventArray>(event.topic, 1000, EventCallback);

    pub_twist_ = nh.advertise<geometry_msgs::TwistWithCovarianceStamped>("radar/twist", 1000);
    pub_inlier_ = nh.advertise<sensor_msgs::PointCloud2>("radar/inliers", 1000);
    pub_outlier_ = nh.advertise<sensor_msgs::PointCloud2>("radar/outliers", 1000);
    // pub_flow_img_ = nh.advertise<sensor_msgs::Image>("event/flow", 1000);

    event_detector_ = new EventFlowDetector(nh, event, radar_event, show_events, smooth);

    estimator_ = new TwistEstimator(nh);

    // geometry_msgs::TwistWithCovarianceStamped twist;

    geometry_msgs::TwistWithCovarianceStamped twist_msgs_;

    while(ros::ok())
    {
        if(event_detector_->Detector())
        {
            twist_msgs_ = event_detector_->GetTwist();
            pub_twist_.publish(twist_msgs_);
            TwistData twist = event_detector_->GetTwistData();
            estimator_->Estimate(twist);
        }
        // else
        // {
        //     ROS_WARN("Detector Failed!");
        // }
        // rate.sleep();
        ros::spinOnce();
    }

    google::ShutdownGoogleLogging();  // 在程序结束时关闭 Google Logging

    return 0;
}

std::vector<double> egovel_time;
rio::RadarEgoVelocityEstimator estimator;
// This part of the code is adapted from: [4DRadarSLAM: ]   
void RadarCallback(const sensor_msgs::PointCloud2::ConstPtr radar_msgs_ptr)
{
    if(radar_msgs_ptr == nullptr)
        return ;
    
    // sensor_msgs::PointCloud2 radar_msgs_copy = *radar_msgs_ptr;
    // if(radar.type == "ti_mmwave")
    // {
    //     // 遍历 PointCloud2 的 fields 修改字段名
    //     for (auto& field : radar_msgs_copy.fields) {
    //         if (field.name == "velocity") {
    //             field.name = "doppler";  // 将 velocity 改为 doppler
    //         }
    //     }
    // }

    // [step1]: radar ego estimation
    geometry_msgs::TwistWithCovarianceStamped twist;
    Eigen::Vector3d v_r, sigma_v_r;
    sensor_msgs::PointCloud2 inlier_radar_msg, outlier_radar_msg;
    clock_t start_ms = clock();
    if (estimator.estimate(*radar_msgs_ptr, v_r, sigma_v_r, inlier_radar_msg, outlier_radar_msg)) {
        clock_t end_ms = clock();
        double time_used = double(end_ms - start_ms) / CLOCKS_PER_SEC;
        egovel_time.push_back(time_used);
        // ROS_WARN("Radar Estimation Successed!");
        
        twist.header.stamp = radar_msgs_ptr->header.stamp;
        twist.header.frame_id = "radar";
        twist.twist.twist.linear.x = v_r.x();
        twist.twist.twist.linear.y = v_r.y();
        twist.twist.twist.linear.z = v_r.z();

        twist.twist.covariance.at(0)  = std::pow(sigma_v_r.x(), 2);
        twist.twist.covariance.at(7)  = std::pow(sigma_v_r.y(), 2);
        twist.twist.covariance.at(14) = std::pow(sigma_v_r.z(), 2);

        // pub_twist_.publish(twist);
        // pub_inlier_.publish(inlier_radar_msg);
        // pub_outlier_.publish(outlier_radar_msg);

        // ROS_INFO("inliers = %d", )

        event_detector_->radar_doppler_velocity.push_back(twist);
        event_detector_->radar_inliers.push_back(inlier_radar_msg);

        // LOG(ERROR) << "inlier_radar_msg.size = " << inlier_radar_msg.width << std::endl;
    }
    // else
    // {
    //     ROS_WARN("Radar Estimation Failed!");
    // }
}

std::fstream test_file;
void EventCallback(const dvs_msgs::EventArray::ConstPtr event_msgs_ptr)
{
    if(event_msgs_ptr == nullptr)
        return ;
    event_detector_->event_stream.push_back(*event_msgs_ptr);

    // test_file.open("/home/hao/Desktop/event.csv", std::ios::out | std::ios::app);
    // test_file << std::setprecision(18) << event_msgs_ptr->header.stamp.toSec() << ", "
    //           << event_msgs_ptr->events.size() << std::endl;
    // test_file.close();

    // test_file.open("/home/hao/Desktop/event.txt", std::ios::out | std::ios::app);
    // test_file << "scan_timestamp = " << event_msgs_ptr->header.stamp.toSec() << std::endl;

    // for(auto e : event_msgs_ptr->events)
    // {
    //     test_file << "e.t = " << e.ts << std::endl; 
    // }
    // test_file << " ------------ " << event_msgs_ptr->header.stamp.toSec() << std::endl;
    // test_file.close();
}

// Function to parse the YAML file and retrieve parameters
bool ParseYaml(const std::string& filename, EventParams& event, RadarParams& radar, 
                    RadarEventParams& radar_event, bool& show_events, double& smooth){
    try
    {
        YAML::Node config = YAML::LoadFile(filename);

        // Parse event parameters
        event.topic = config["event"]["topic"].as<std::string>();
        event.freq = config["event"]["freq"].as<int>();
        event.fx = config["event"]["fx"].as<double>();
        event.fy = config["event"]["fy"].as<double>();
        event.cx = config["event"]["cx"].as<double>();
        event.cy = config["event"]["cy"].as<double>();
        
        event.k1 = config["event"]["k1"].as<double>();
        event.k2 = config["event"]["k2"].as<double>();
        event.p1 = config["event"]["p1"].as<double>();
        event.p2 = config["event"]["p2"].as<double>();
        event.k3 = config["event"]["k3"].as<double>();

        // event.resolution = config["event"]["resolution"].as<int[2]>();
        std::array<int, 2> resolution = config["event"]["resolution"].as<std::array<int, 2>>();
        event.resolution[0] = resolution[0];
        event.resolution[1] = resolution[1];

        event.deltaT = config["event"]["deltaT"].as<double>();
        event.t1 = config["event"]["t1"].as<double>();
        event.mesh_size = config["event"]["mesh_size"].as<int>();
        event.timeshift_cam_imu = config["event"]["timeshift_cam_imu"].as<double>();

        LOG(INFO) << "deltaT = " << event.deltaT << ", t1 = " << event.t1 << std::endl;

        // Parse radar parameters
        radar.topic = config["radar"]["topic"].as<std::string>();

        // Parse radar-event parameters (4x4 transformation matrix)
        auto T_re_list = config["radar_event"]["T_re"];
        for (int i = 0; i < 4; ++i) {
            for (int j = 0; j < 4; ++j) {
                radar_event.T_re(i, j) = T_re_list[i][j].as<double>();
            }
        }

        show_events = config["event"]["show_events"].as<bool>();
        smooth = config["smooth"].as<double>();
    }
    catch(const std::exception& e)
    {
        std::cerr << e.what() << '\n';
        return false;
    }

    return true;
}




// int main(int argc,char ** argv)
// {
//     google::InitGoogleLogging(argv[0]);
//     // 指定日志文件的保存路径
//     FLAGS_log_dir = "/home/hao/Desktop/radar-event-new/src/TwistEstimator/glog_log";

//     ros::init(argc,argv,"test_detector");
//     ros::NodeHandle nh;

//     EventParams event;
//     RadarParams radar;
//     RadarEventParams radar_event;
//     radar_event.T_re = Eigen::Matrix4d::Identity();  // Initialize to identity

//     const std::string filename = "/home/hao/Desktop/radar-event-new/src/TwistEstimator/config/params.yaml";
//     bool show_events = false;
//     double smooth = 0.1;
//     ParseYaml(filename, event, radar, radar_event, show_events, smooth);
//     LOG(INFO) << "smooth = " << smooth << std::endl;

//     ros::Subscriber radar_sub_ = nh.subscribe<sensor_msgs::PointCloud2>(radar.topic, 1000, RadarCallback);
//     ros::Subscriber event_sub_ = nh.subscribe<dvs_msgs::EventArray>(event.topic, 1000, EventCallback);

//     pub_twist_ = nh.advertise<geometry_msgs::TwistWithCovarianceStamped>("radar/twist", 1000);
//     pub_inlier_ = nh.advertise<sensor_msgs::PointCloud2>("radar/inliers", 1000);
//     pub_outlier_ = nh.advertise<sensor_msgs::PointCloud2>("radar/outliers", 1000);

//     event_detector_ = new EventFlowDetector(event, radar_event, show_events, smooth);
    
//     estimator_ = new TwistEstimator();
    
//     geometry_msgs::TwistWithCovarianceStamped twist_msgs_;

//     ros::Rate rate(20);
//     while(ros::ok())
//     {
//         if(event_detector_->Detector())
//         {
//             twist_msgs_ = event_detector_->GetTwist();
//             pub_twist_.publish(twist_msgs_);
//             TwistData twist = event_detector_->GetTwistData();
//             estimator_->Estimate(twist);
//         }
//         rate.sleep();
//         ros::spinOnce();
//     }

//     google::ShutdownGoogleLogging();  // 在程序结束时关闭 Google Logging

//     return 0;
// }