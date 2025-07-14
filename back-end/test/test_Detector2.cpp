#pragma once

// #include "../../include/radar-event/EventDetector.hpp"

#include "radar_ego_velocity_estimator/radar_ego_velocity_estimator.h"

// #include "event_flow_detector/event_flow_detector.h"

// #include "event_flow_detector/event_ct_flow.h"

#include "event_flow_detector/event_ct_flow_LP.h"

#include "estimator/TwistEstimator.h"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/conversions.h>  // For toPCL


void RadarCallback(const sensor_msgs::PointCloud2::ConstPtr radar_msgs_ptr);

void EventCallback(const dvs_msgs::EventArray::ConstPtr event_msgs_ptr);

bool ParseYaml(const std::string& filename, EventParams& event, RadarParams& radar, 
                    RadarEventParams& radar_event, bool& show_events, double& smooth, int& filter_num, 
                    int & median_radius, bool & ignore_polarity, bool& use_gauss, double & ratio_inliers);

void DopplerEstimation(const sensor_msgs::PointCloud2 radar_msgs_copy);

// Global
std::mutex detector_data_mutex;

// EventFlowDetector* event_detector_;

EventFlowDetector3* event_detector_;

geometry_msgs::TwistWithCovarianceStamped twist;
geometry_msgs::TwistWithCovarianceStamped last_twist;

ros::Publisher pub_twist_2;
ros::Publisher pub_twist_;
ros::Publisher pub_inlier_;
ros::Publisher pub_outlier_;
EventParams event;
RadarParams radar;

std::deque<sensor_msgs::PointCloud2> radar_buffer;
std::deque<dvs_msgs::EventArray> event_buffer;
std::mutex callback_mutex;

TwistEstimator* estimator_;

void RadarCallback(const sensor_msgs::PointCloud2::ConstPtr radar_msgs_ptr)
{
    // std::lock_guard<std::mutex> lock(callback_mutex);

    if(radar_msgs_ptr == nullptr)
        return ;

    sensor_msgs::PointCloud2 radar_msgs_raw = *radar_msgs_ptr;

    if(radar.type == "ti_mmwave")
    {
        // 遍历 PointCloud2 的 fields 修改字段名
        for (auto& field : radar_msgs_raw.fields) {
            if (field.name == "velocity") {
                field.name = "doppler";  // 将 velocity 改为 doppler
                // LOG(ERROR) << "change field name" << std::endl;
            }
        }
        // for (auto& field : radar_msgs_copy.fields) {
        //     LOG(ERROR) << "check field = " << field << std::endl;
        // }
    }

    LOG(ERROR) << "Catch Radar Data" << std::endl;

    // 数据交换
    {
        std::lock_guard<std::mutex> lock(callback_mutex);
        // LOG(ERROR) << "Catch Radar Data" << std::endl;
        radar_buffer.push_back(radar_msgs_raw);
    }
}

void savePointCloud2ToTxt(const sensor_msgs::PointCloud2& radar_msgs_copy, const std::string& filename) {
    // 打开文件
    std::ofstream outfile(filename, std::ios::out | std::ios::app);
    if (!outfile.is_open()) {
        ROS_ERROR("Unable to open file for writing");
        return;
    }

    // 获取字段信息
    for (const auto& field : radar_msgs_copy.fields) {
        outfile << "Field: " << field.name << " | " 
                << "Offset: " << field.offset << " | "
                << "Datatype: " << field.datatype << " | "
                << "Count: " << field.count << "\n";
    }

    outfile << "Field: " << radar_msgs_copy.width << " , " << radar_msgs_copy.height << std::endl;

    // 获取点云的每个数据（data字段是二进制数据）
    size_t point_step = radar_msgs_copy.point_step;
    size_t row_step = radar_msgs_copy.row_step;
    const uint8_t* data = &radar_msgs_copy.data[0];

    // 假设每个点包含多个字段（例如x, y, z, intensity等）
    for (size_t i = 0; i < radar_msgs_copy.height; ++i) {
        for (size_t j = 0; j < radar_msgs_copy.width; ++j) {
            const uint8_t* point = data + i * row_step + j * point_step;
            for (const auto& field : radar_msgs_copy.fields) {
                // 获取字段的值并保存
                if (field.name == "x" || field.name == "y" || field.name == "z") {
                    // 获取float类型的字段（例如x, y, z）
                    float value = *reinterpret_cast<const float*>(point + field.offset);
                    outfile << value << " ";
                } else if (field.name == "intensity") {
                    // 获取强度字段（假设为float类型）
                    float intensity = *reinterpret_cast<const float*>(point + field.offset);
                    outfile << intensity << " ";
                } else if (field.name == "doppler") {
                    // 处理其他字段，如doppler（假设为float类型）
                    float doppler = *reinterpret_cast<const float*>(point + field.offset);
                    outfile << doppler << " ";
                }
                // 你可以添加其他字段的处理代码
            }
            outfile << "\n";
        }
    }

    // 关闭文件
    outfile.close();
    ROS_INFO("PointCloud2 saved to %s", filename.c_str());
}

// This part of the code is adapted from: [4DRadarSLAM: ]   
// std::vector<double> egovel_time;
rio::RadarEgoVelocityEstimator estimator;
void DopplerEstimation(const sensor_msgs::PointCloud2 radar_msgs_copy)
{
    assert(radar_msgs_copy.fields[4].name == "doppler" && "missing field name 'doppler'");

    // [step1]: radar ego estimation
    geometry_msgs::TwistWithCovarianceStamped radar_twist;
    Eigen::Vector3d v_r, sigma_v_r;
    sensor_msgs::PointCloud2 inlier_radar_msg, outlier_radar_msg;
    clock_t start_ms = clock();

    // savePointCloud2ToTxt(radar_msgs_copy, "/media/hao/hao2/228/test/radar_doppler_data.txt");

    if (estimator.estimate(radar_msgs_copy, v_r, sigma_v_r, inlier_radar_msg, outlier_radar_msg)) {
        clock_t end_ms = clock();
        double time_used = double(end_ms - start_ms) / CLOCKS_PER_SEC;
        // egovel_time.push_back(time_used);
        // ROS_WARN("Radar Estimation Successed!");
        // radar_twist.header.stamp = radar_msgs_copy.header.stamp;

        // radar_twist.header.stamp = radar_msgs_ptr->header.stamp;
        radar_twist.header.stamp = radar_msgs_copy.header.stamp;
        radar_twist.header.frame_id = "radar";
        radar_twist.twist.twist.linear.x = v_r.x();
        radar_twist.twist.twist.linear.y = v_r.y();
        radar_twist.twist.twist.linear.z = v_r.z();

        radar_twist.twist.covariance.at(0)  = std::pow(sigma_v_r.x(), 2);
        radar_twist.twist.covariance.at(7)  = std::pow(sigma_v_r.y(), 2);
        radar_twist.twist.covariance.at(14) = std::pow(sigma_v_r.z(), 2);

        pub_twist_2.publish(radar_twist);
        // pub_twist_.publish(radar_twist);
        pub_inlier_.publish(inlier_radar_msg);
        // pub_outlier_.publish(outlier_radar_msg);

        LOG(ERROR) << "Radar Estimation: " << std::setprecision(8) << radar_twist.twist.twist.linear.x << ", " 
                    << radar_twist.twist.twist.linear.y << ", " << radar_twist.twist.twist.linear.z << std::endl;

        // HAO: 数据交换
        {
            // std::lock_guard<std::mutex> lock(detector_data_mutex);
            event_detector_->radar_doppler_velocity.push_back(radar_twist);
            event_detector_->radar_inliers.push_back(inlier_radar_msg);
        }

        // Debug: 只调试雷达多普勒
        {
            radar_twist.twist.twist.angular.x = 0;
            radar_twist.twist.twist.angular.y = 0;
            radar_twist.twist.twist.angular.z = 0;
            std::fstream detector2_file("/home/hao/Desktop/radar-event-new/src/TwistEstimator/output/detector2.csv",std::ios::out | std::ios::app);
            detector2_file << std::setprecision(18) << radar_twist.header.stamp.toSec() << "," 
                        << radar_twist.twist.twist.linear.x << ","
                        << radar_twist.twist.twist.linear.y << ","
                        << radar_twist.twist.twist.linear.z << ","
                        << radar_twist.twist.twist.angular.x << ","
                        << radar_twist.twist.twist.angular.y << ","
                        << radar_twist.twist.twist.angular.z << std::endl;
            detector2_file.close();
        }

        // Debug: 调试雷达数据输入是否跳帧
        {
            static double last_radar_timestamp = -1.0;
            static double delta_radar_time = -1.0;
            static double last_delta_radar_time = -1.0;
            if(last_radar_timestamp < 0)
            {
                last_radar_timestamp = radar_msgs_copy.header.stamp.toSec();
            }
            else
            {
                delta_radar_time = radar_msgs_copy.header.stamp.toSec() - last_radar_timestamp;
                if(last_delta_radar_time < 0)
                {
                    last_delta_radar_time = delta_radar_time;
                }
                else
                {
                    double radar_rate = delta_radar_time / last_delta_radar_time;
                    if(radar_rate < 0.75 && radar_rate > 1.25)
                    {
                        LOG(ERROR) << "Error: Radar Data Is Missing!" << std::endl;
                    }
                }
            }

        }


        // ROS_INFO("Radar Estimation: [%f, %f, %f]", v_r.x(), v_r.y(), v_r.z());

        // std::fstream file("/home/hao/Desktop/radar_esti_velocity",std::ios::out | std::ios::app);
        // file << "twist = [" << std::setprecision(18) << radar_msgs_copy.header.stamp.toSec() << ", "
        // << radar_twist.twist.twist.linear.x <<", "
        // << radar_twist.twist.twist.linear.y <<", "
        // << radar_twist.twist.twist.linear.z
        // << "]" << std::endl;
        // file.close();
    }
    else
    {
        radar_twist.twist.twist.linear.x = 0;
        radar_twist.twist.twist.linear.y = 0;
        radar_twist.twist.twist.linear.z = 0;
        LOG(ERROR) << "Radar Estimation Failed!";
    }
}

std::fstream event_file;
void EventCallback(const dvs_msgs::EventArray::ConstPtr event_msgs_ptr)
{
    // std::lock_guard<std::mutex> lock(event_callback_mutex);

    if(event_msgs_ptr == nullptr)
        return ;

    static double last_event_time = -1;
    if(last_event_time == -1)
        last_event_time = event_msgs_ptr->header.stamp.toSec();
    else
    {
        double cur_event_time = event_msgs_ptr->header.stamp.toSec();
        double time_rate = cur_event_time / last_event_time;
        last_event_time = cur_event_time;
        if(!(time_rate > 0.8 && time_rate < 1.2 ))
        {
            LOG(ERROR) << "missing event time!" << std::endl;
        }
    }


    LOG(ERROR) << "event_msgs_ptr.size = " << event_msgs_ptr->events.size() << std::endl;
    LOG(ERROR) << "event_msgs output = " << event_msgs_ptr->events.size() / (event_msgs_ptr->events.back().ts.toSec() - event_msgs_ptr->events.front().ts.toSec()) << " p/s" << std::endl;

    // event_file.open("/home/hao/Desktop/events.csv", std::ios::out | std::ios::app);
    // event_file << event_msgs_ptr->events.size() / (event_msgs_ptr->events.back().ts.toSec() - event_msgs_ptr->events.front().ts.toSec()) << std::endl;
    // event_file.close();

    // HAO: 数据交换
    {
        // std::lock_guard<std::mutex> lock(detector_data_mutex);
        // event_detector_->event_stream.push_back(*event_msgs_ptr);

        std::lock_guard<std::mutex> lock(callback_mutex);
        // LOG(ERROR) << "Catch Event Data" << std::endl;
        event_buffer.push_back(*event_msgs_ptr);
    }

}

// Function to parse the YAML file and retrieve parameters
bool ParseYaml(const std::string& filename, EventParams& event, RadarParams& radar, 
                    RadarEventParams& radar_event, bool& show_events, double& smooth, int& filter_num, 
                    int & median_radius, bool & ignore_polarity, bool& use_gauss, double & ratio_inliers)
{
    try
    {
        YAML::Node config = YAML::LoadFile(filename);
        std::cout << "A" << std::endl;
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

        event.t1_count = config["event"]["t1_count"].as<double>();
        event.t2_count = config["event"]["t2_count"].as<double>();

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
        LOG(ERROR) << "radar.topic = " << radar.topic << std::endl;
        radar.type = config["radar"]["type"].as<std::string>();

        // Parse radar-event parameters (4x4 transformation matrix)
        auto T_re_list = config["radar_event"]["T_re"];
        for (int i = 0; i < 4; ++i) {
            for (int j = 0; j < 4; ++j) {
                radar_event.T_re(i, j) = T_re_list[i][j].as<double>();
            }
        }

        show_events = config["event"]["show_events"].as<bool>();
        smooth = config["smooth"].as<double>();

        filter_num = config["filter_num"].as<int>();

        median_radius = config["median_radius"].as<int>();

        ignore_polarity = config["ignore_polarity"].as<bool>();

        use_gauss = config["use_gauss"].as<bool>();

        ratio_inliers = config["ratio_inliers"].as<double>();
        LOG(ERROR) << "ratio_inliers = " << ratio_inliers << std::endl;

        LOG(ERROR) << "radar modify radar_config: " << config["radar_config"].as<std::string>() << std::endl;
        estimator.LOAD(config["radar_config"].as<std::string>());
    }
    catch(const std::exception& e)
    {
        std::cerr << e.what() << '\n';
        return false;
    }

    return true;
}


// main loop
int use_threads = 1;
// 独立线程对象
// detector_thread;
void RUN()
{
    while (ros::ok())
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(2));  // 控制循环频率，避免过高的 CPU 占用
        // 与回调交换
        // if(detector_thread != busy)
        {
            sensor_msgs::PointCloud2 radar_msgs;
            radar_msgs.width = -1;
            dvs_msgs::EventArray event_msgs_copy;
            event_msgs_copy.width = 0;

            // 回调交换接口
            // 事件一般触发更快
            {
                std::lock_guard<std::mutex> lock(callback_mutex);
                if (!event_buffer.empty())
                {
                    // 数据交换
                    // {     
                        // 提取事件数据
                        event_msgs_copy = event_buffer.front();
                        event_buffer.pop_front();     
                        LOG(ERROR) << "Event Buffer = " << event_buffer.size() << std::endl;  
                    // }
                   
                }
                if (!radar_buffer.empty()){
                        radar_msgs = radar_buffer.front();
                        radar_buffer.pop_front();
                        LOG(ERROR) << "Doppler Buffer = " << radar_buffer.size() << std::endl;
                }  
                // else
                // {
                //     LOG(ERROR) << "No Event Input" << std::endl;
                // }
            }
            // LOG(ERROR) << "Data Stream Input" << std::endl;
            
            // 事件数据
            if(event_msgs_copy.width > 0)
            {
                // LOG(ERROR) << "event_msgs_copy.width = " << event_msgs_copy.width << std::endl;
                // LOG(ERROR) << "event_msgs_copy.sec = " << event_msgs_copy.header.stamp.toSec() << std::endl;
                event_detector_->event_stream.push_back(event_msgs_copy);
            }
            // else
            // {
            //     LOG(ERROR) << "No Event Input" << std::endl;
            //     continue;
            // }  

            // 多普勒估计
            if (radar_msgs.width != -1)
            {
                LOG(ERROR) << "DopplerEstimation" << std::endl;
                LOG(ERROR) << "radar_msgs_copy.width = " << radar_msgs.width << std::endl;
                DopplerEstimation(radar_msgs);    
            }
            // else
            // {
            //     LOG(ERROR) << "No Doppler Input" << std::endl;
            //     continue;
            // }       
           
            // std::lock_guard<std::mutex> lock(detector_mutex);
            LOG(ERROR) << "Detector" << std::endl;

            if (event_detector_->Detector())
            {
                twist = event_detector_->GetTwist();
                // LOG(ERROR) << std::setprecision(18) << "twist time diff = " << (twist.header.stamp - last_twist.header.stamp).toSec() << std::endl;
                last_twist = twist;
                pub_twist_.publish(twist);
                TwistData2 twist2 = event_detector_->GetTwistData2();
                // 临时添加
                /*{
                    std::fstream detector_file("/home/hao/Desktop/radar-event-new/src/TwistEstimator/output/detector.csv",std::ios::out | std::ios::app);
                    detector_file << std::setprecision(18) << twist.header.stamp.toSec() << "," 
                                << twist.twist.twist.linear.x << ","
                                << twist.twist.twist.linear.y << ","
                                << twist.twist.twist.linear.z << ","
                                << twist.twist.twist.angular.x << ","
                                << twist.twist.twist.angular.y << ","
                                << twist.twist.twist.angular.z << std::endl;
                    detector_file.close();

                }*/
                // estimator_->Estimate2(twist2);   // 只调试前端
            }
            else
            {
                LOG(ERROR) << "Detector Failed" << std::endl;
            }
        }      
    }
}


int main(int argc,char ** argv)
{
    google::InitGoogleLogging(argv[0]);
    // 指定日志文件的保存路径
    FLAGS_log_dir = "/home/hao/Desktop/radar-event-new/src/TwistEstimator/../glog_log";
    // FLAGS_minloglevel = google::;
    // FLAGS_logtostderr = true;

    ros::init(argc,argv,"test_detector");
    ros::NodeHandle nh;

    RadarEventParams radar_event;
    radar_event.T_re = Eigen::Matrix4d::Identity();  // Initialize to identity

    // "/home/hao/Desktop/radar-event-new/src/TwistEstimator/config/params.yaml";
    // const std::string filename = "/home/hao/Desktop/radar-event-new/src/TwistEstimator/config/dji.yaml";
    const std::string filename = "/home/hao/Desktop/radar-event-new/src/TwistEstimator/config/dji2.yaml";
    // const std::string filename = "/home/hao/Desktop/radar-event-new/src/TwistEstimator/config/dvs.yaml";
    // const std::string filename = "/home/hao/Desktop/radar-event-new/src/TwistEstimator/config/demo_228.yaml";
    bool show_events = false;
    double smooth = 0.1;

    int filter_num = 3;
    int median_radius = 5;
    bool ignore_polarity = false;
    bool use_gauss = false;
    double ratio_inliers = 0.1;
    ParseYaml(filename, event, radar, radar_event, show_events, smooth, filter_num, median_radius, ignore_polarity, use_gauss, ratio_inliers);
    std::cout << "smooth = " << smooth << std::endl;

    LOG(ERROR) << "radar.topic = " << radar.topic << std::endl;
    ros::Subscriber radar_sub_ = nh.subscribe<sensor_msgs::PointCloud2>(radar.topic, 100000, RadarCallback);
    LOG(ERROR) << "event.topic = " << event.topic << std::endl;
    ros::Subscriber event_sub_ = nh.subscribe<dvs_msgs::EventArray>(event.topic, 100000, EventCallback);

    // radar_buffer.reserve(5000);

    pub_twist_ = nh.advertise<geometry_msgs::TwistWithCovarianceStamped>("radar/twist", 1000);
    pub_twist_2 = nh.advertise<geometry_msgs::TwistWithCovarianceStamped>("radar/twist_raw", 1000);
    pub_inlier_ = nh.advertise<sensor_msgs::PointCloud2>("radar/inliers", 1000);
    pub_outlier_ = nh.advertise<sensor_msgs::PointCloud2>("radar/outliers", 1000);

    // event_detector_ = new EventFlowDetector(nh, event, radar_event, show_events, smooth, filter_num, median_radius, ignore_polarity, use_gauss, ratio_inliers);
    event_detector_ = new EventFlowDetector3(nh, event, radar_event, show_events, smooth, filter_num, ignore_polarity, ratio_inliers);
    estimator_ = new TwistEstimator(nh);
    
    // ros::Rate rate(20);

    // for callback
    // ros::AsyncSpinner spinner(2); // 使用1个线程
    // spinner.start();

    // for detect main
    // add this to another thread
    // while(ros::ok())
    // {
    //     // ros::spinOnce();
    //     if(event_detector_->Detector())
    //     {
    //         twist = event_detector_->GetTwist();
    //         // LOG(ERROR) << std::setprecision(18) << "twist time diff = " << (twist.header.stamp - last_twist.header.stamp).toSec() << std::endl;
    //         last_twist = twist;
    //         pub_twist_.publish(twist);
    //     }
    //     // rate.sleep();
    // }

    std::thread detect_thread(RUN);
    detect_thread.detach();

    // 使用AsyncSpinner来处理回调
    ros::AsyncSpinner spinner(2); // 启动2个线程
    spinner.start();
    // 主线程继续做其他事情
    ros::waitForShutdown();

    // ros::spin();

    // detect_thread.join();  // 等待线程安全退出
    google::ShutdownGoogleLogging();  // 在程序结束时关闭 Google Logging

    return 0;
}