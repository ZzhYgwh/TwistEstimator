#ifndef __EVENT_CT_FLOW_H__
#define __EVENT_CT_FLOW_H__

#include <random>
#include <vector>

#include <limits>
#include <cmath>
#include <algorithm>

#include <fstream>
#include <yaml-cpp/yaml.h>
#include <chrono>

#include <dvs_msgs/Event.h>
#include <dvs_msgs/EventArray.h>
#include <sensor_msgs/PointCloud2.h>

#include <sensor_msgs/Image.h>

#include <geometry_msgs/TwistWithCovarianceStamped.h>

#include <opencv2/opencv.hpp>


#include <opencv2/optflow.hpp>         // 包含 RLOF 和光流相关的功能
#include <opencv2/highgui.hpp>         // 用于图像显示和窗口管理

#include <Eigen/Core>
#include <Eigen/Dense>

#include <glog/logging.h>

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>

#include <thread>

#include <mutex>

// just for debug
#include <filesystem>

#include "event_flow_detector.h"


struct pair_hash {
    template <class T1, class T2>
    std::size_t operator()(const std::pair<T1, T2>& p) const {
        return std::hash<T1>{}(p.first) ^ (std::hash<T2>{}(p.second) << 1);
    }
};


class EventFlowDetector2 {
public:
    EventFlowDetector2(ros::NodeHandle& nh_this, EventParams& e_params, RadarEventParams r_params, bool show_events, double smooth, int filter_num = 3, bool ignore_polarity = false, double ratio_inliers = 0.1)
        : nh_this_(nh_this), event_dt(e_params.deltaT), event_t1(e_params.t1), sensor_width(e_params.resolution[0]), lpf(smooth), lpf_v(smooth), filter_num(filter_num), ignore_polarity(ignore_polarity), ratio_inliers(ratio_inliers),
        sensor_height(e_params.resolution[1]), t1_image_count(e_params.t1_count), t2_image_count(e_params.t2_count), T_re(r_params.T_re), radius(e_params.mesh_size), show_events_(show_events) 
        {
            K << e_params.fx, 0, e_params.cx,
                0, e_params.fy, e_params.cy,
                0,0,1;

            K_cv = cv::Mat(3, 3, CV_64F, K.transpose().data()).clone(); // 使用 .clone() 以确保数据独立
            K_cv = K_cv.t();

            distort_cv = (cv::Mat_<double>(1, 5) << e_params.k1, e_params.k2, e_params.p1, e_params.p2, e_params.k3);

            image_pub_ = nh_this.advertise<sensor_msgs::Image>("/event/flow_img", 10);

            pub_event_image_ = nh_this.advertise<sensor_msgs::Image>("/event/img", 10);

            cv_image_.encoding = "bgr8"; // 假设处理的是彩色图像
            cv_image_.header.frame_id = "camera_frame";

            LOG(ERROR) << "radius = " << radius << std::endl;
        }


    // 删除旧事件数据
    void RemoveOldEvents(double threshold_time) {
        // std::lock_guard<std::mutex> lock(detector_data_mutex);
        while (!event_stream.empty()) {
            dvs_msgs::EventArray::Ptr event_array = event_stream.front();

            if(event_array->events.front().ts.toSec() >= threshold_time)
                break;

            double end_event_time = event_array->events.back().ts.toSec();

            if (end_event_time < threshold_time) {
                event_stream.pop_front();
                continue;
            }

            auto& events = event_array->events;
            events.erase(std::remove_if(events.begin(), events.end(), 
                       [threshold_time](const dvs_msgs::Event& e) { return e.ts.toSec() < threshold_time; }), 
                       events.end());

            if (events.empty()) event_stream.pop_front();
        }
    }

    void drawSquareBorder(cv::Mat& image, const cv::Point2d& center, double radius) {
        // 计算正方形的顶点
        int x1 = static_cast<int>(center.x - radius);  // 左上角x
        int y1 = static_cast<int>(center.y - radius);  // 左上角y
        int x2 = static_cast<int>(center.x + radius);  // 右下角x
        int y2 = static_cast<int>(center.y + radius);  // 右下角y

        // 绘制正方形边框
        cv::rectangle(image, cv::Point(x1, y1), cv::Point(x2, y2), cv::Scalar(0, 255, 0), 1);  // 绿色，线宽为1
    }

    void PublishTimeImages(cv::Mat pub_img) {
        // 创建一个与 pub_img 相同尺寸的彩色图像，初始化为黑色
        cv::Mat color_img = cv::Mat::zeros(pub_img.size(), CV_8UC3);
        // cv::Mat color_img = pub_img.clone();
        // 创建掩膜，分别表示大于0、小于0和等于0的区域
        cv::Mat mask_pos = pub_img > 0;  // 大于0的区域
        cv::Mat mask_neg = pub_img < 0;  // 小于0的区域
        cv::Mat mask_zero = pub_img == 0; // 等于0的区域

        // 将大于0的区域设置为红色 (BGR: 0, 0, 255)
        color_img.setTo(cv::Vec3b(0, 0, 255), mask_pos);

        // 将小于0的区域设置为蓝色 (BGR: 255, 0, 0)
        color_img.setTo(cv::Vec3b(255, 0, 0), mask_neg);

        // 将等于0的区域设置为黑色 (BGR: 0, 0, 0) -> 已经是默认值
        color_img.setTo(cv::Vec3b(255, 255, 255), mask_zero);

        
        // 设置字体参数
        int font_face = cv::FONT_HERSHEY_SIMPLEX;
        double font_scale = 0.3;  // 字体大小
        int thickness = 1;        // 字体厚度
        cv::Scalar color(255, 255, 255);  // 文本颜色：白色


        static long int img_index = 0;
        cv::Size target_size(25, 25);
        for (size_t i = 0; i < flow_pre_points.size(); ++i) {
            cv::Mat color_img_temp = color_img.clone();

            cv::Point2d point = best_inliers[i];
            event_flow_velocity velocity = flow_pre_points[i];

            drawSquareBorder(color_img, point, radius);

            // 从 color_img 中分割 [point - radius, point + radius] 范围大小的局部邻域为flow_patch
            double radius_test = 1.0 * radius;
            // 计算局部邻域的矩形区域
            int x_start = std::max(static_cast<double>(0), point.x - radius_test);
            int y_start = std::max(static_cast<double>(0), point.y - radius_test);
            int x_end = std::min(static_cast<double>(color_img_temp.cols), point.x + radius_test + 1);
            int y_end = std::min(static_cast<double>(color_img_temp.rows), point.y + radius_test + 1);
            cv::Rect roi(x_start, y_start, x_end - x_start, y_end - y_start);
            cv::Mat flow_patch = color_img(roi).clone();

            cv::Point2d end_point = point + cv::Point2d(velocity.x, velocity.y);
            end_point.x = std::min(std::max(0.0, end_point.x), static_cast<double>(color_img.cols - 1));
            end_point.y = std::min(std::max(0.0, end_point.y), static_cast<double>(color_img.rows - 1));

            cv::arrowedLine(color_img, point, end_point, cv::Scalar(0, 255, 0), 1, 8, 0, 0.1);

            for (size_t i = 0; i < flow_pre_points.size(); ++i) {
                cv::imwrite("/media/hao/hao2/228/test/lab/flow_img/flowimage_" + std::to_string(img_index) 
                + "_" + std::to_string(i) + "_" + std::to_string(velocity.x) + "_" + std::to_string(velocity.y) + ".png", flow_patch);
            }
        }



        img_index ++;

        // // 在图像上绘制光流点和光流分量
        // LOG(ERROR) << "visualize points = " << flow_pre_points.size() << std::endl;
        // for (size_t i = 0; i < flow_pre_points.size(); ++i) {
        //     cv::Point2d point = best_inliers[i];
        //     event_flow_velocity velocity = flow_pre_points[i];

        //     // 绘制光流点（用绿色的圆圈标出）
        //     cv::circle(color_img, point, 1, cv::Scalar(0, 255, 0), -1);
        //     drawSquareBorder(color_img, point, radius);

        //     // 绘制光流向量（用绿色的箭头标出）
        //     // LOG(ERROR) << "flow = [" << velocity.x << ", " << velocity.y << "]" << std::endl;
        //     // if(abs(velocity.x) + abs(velocity.y) < 5)
        //     //     continue;

        //     cv::Point2d end_point = point + cv::Point2d(velocity.x, velocity.y);
        //     end_point.x = std::min(std::max(0.0, end_point.x), static_cast<double>(color_img.cols - 1));
        //     end_point.y = std::min(std::max(0.0, end_point.y), static_cast<double>(color_img.rows - 1));
            
        //     // LOG(ERROR) << "start_point = [" << point.x << ", " << point.y << "]" << std::endl;
        //     // LOG(ERROR) << "end_point = [" << end_point.x << ", " << end_point.y << "]" << std::endl;
        //     cv::arrowedLine(color_img, point, end_point, cv::Scalar(0, 255, 0), 1, 8, 0, 0.1);


        // }

        std::string angular_data = "Angular: x=" + std::to_string(twist_.twist.twist.angular.x) +
                            " y=" + std::to_string(twist_.twist.twist.angular.y) +
                            " z=" + std::to_string(twist_.twist.twist.angular.z);
        cv::Point text_position(20, 10); // 选择一个适当的位置
        cv::putText(color_img, angular_data, text_position, cv::FONT_HERSHEY_SIMPLEX, 0.35, cv::Scalar(255, 255, 255), 0.75);
        

        cv::imwrite("/media/hao/hao2/228/test/lab/img/timeimage_" + std::to_string(img_index) + ".png", color_img);
        

        // 转为 sensor_msgs::Image 中
        std_msgs::Header header;
        header.stamp = ros::Time::now();
        header.frame_id = "event";

        // 将 color_img 转换为 ROS 消息
        sensor_msgs::ImagePtr msg = cv_bridge::CvImage(header, "bgr8", color_img).toImageMsg();

        // 你可以在这里发布这个 msg 或者进一步处理它
        pub_event_image_.publish(msg);

        ROS_INFO("Publish Time Image");
        LOG(ERROR) << "Publish Time Image" << std::endl;
    }

    double AccumuConstractEventsWindows(double timestamp, cv::Mat& time_img, std::unordered_map<std::pair<int, int>, double, 
                                                pair_hash>& event_link, long int need_events = 1000)
    {
        LOG(ERROR) << "need events: " << need_events << std::endl;

        // 首先检查是否有足够数据
        long int accumulated_events = 0;
        for (auto it = event_stream.rbegin(); it != event_stream.rend(); ++it)
        {
            if(it->header.stamp.toSec() > timestamp)
                continue;

            if(it->events.back().ts.toSec() > timestamp && it->events.front().ts.toSec() < timestamp)
            {
                for(auto& e: it->events)
                {
                    if(e.ts.toSec() <= timestamp)
                        if (!ignore_polarity && e.polarity)
                            accumulated_events ++;
                        else
                            accumulated_events ++;

                }
                LOG(ERROR) << "N1: " << accumulated_events << std::endl;
                continue;
            }


            LOG(ERROR) << "N2: " << it->events.size() << std::endl;
            accumulated_events += it->events.size();
            LOG(ERROR) << "SUM1: " << accumulated_events << std::endl;

            if(accumulated_events > need_events)
               break;
            // else
            // {
            //     LOG(ERROR) << "No enough events" << std::endl;
            //     return -1;
            // }
        }

        if(accumulated_events < need_events)
        {
            LOG(ERROR) << "No enough events" << std::endl;
            return -1;
        }

        // 从 std::deque<dvs_msgs::EventArray> event_stream 中获取const long int constract_events 个事件作为窗口
        // 以 timestamp 为起点，向后分割，直到满足事件数，最后返回初始时间 start_accumu_time;
        // 相当于形成窗口 [start_accumu_time, timestamp]
        // 要求高效完成 
        time_img = cv::Mat::zeros(sensor_height, sensor_width, CV_64FC1);
        
        // 累计事件数
        double start_accumu_time = timestamp;
        accumulated_events = 0;

        // std::lock_guard<std::mutex> lock(detector_data_mutex);

        // 从后向前遍历，累积事件数直到满足条件
        for (auto it = event_stream.rbegin(); it != event_stream.rend(); ++it) {

            // if(it->header.stamp.toSec() > timestamp)
            //     continue;

            // std::cout << "event scan = " << event_stream.size() << std::endl;
            // 检查事件时间范围
            if (it->events.empty() || it->events.front().ts.toSec() < timestamp) {
                // std::cout << "events = " << it->events.size() << std::endl;
                for (auto event_it = it->events.rbegin(); event_it != it->events.rend(); ++event_it) {
                    if (event_it->ts.toSec() <= timestamp) {
                        accumulated_events++;
                        start_accumu_time = event_it->ts.toSec();
                        // TODO: 讨论是否需要正负极性
                        if (!ignore_polarity && event_it->polarity)
                            time_img.at<double>(event_it->y, event_it->x)++;
                        else
                            time_img.at<double>(event_it->y, event_it->x)++;

                            // 存储事件数据
                            event_link[{event_it->x, event_it->y}] = event_it->ts.toSec() - timestamp;
                        // else
                        //     time_img.at<double>(event_it->y, event_it->x)--;
                        // std::cout << "accumulated_events = " << accumulated_events << std::endl;
                        if (accumulated_events >= need_events) {
                            LOG(ERROR) << "accumulated_events = " << accumulated_events << std::endl;
                            LOG(ERROR) << "start_accumu_time = " << std::setprecision(18) << start_accumu_time << std::endl;
                            LOG(ERROR) << "duration = " << timestamp - start_accumu_time << std::endl;
                            return timestamp - start_accumu_time;
                        }
                    }
                }
            }
        }

        // std::cout << "accumulated_events = " << accumulated_events << std::endl;
        // return timestamp; // 如果没有事件，直接返回当前时间
        // LOG(ERROR) << "Event Stream is not enough!" << std::endl; 
        return -1;  // 如果没有事件或事件不足，返回报错
        
    }

    // 时间图像压缩函数
    void AccumulateEvents(double start_time, double end_time, cv::Mat& TimeImage, long int& count) {
        
        long int events_count = 0;
        int end_index = -1;

        if(event_stream.back().events.back().ts.toSec() < end_time) return;
        // for(std::iterator<dvs_msgs::EventArray>::)

        // int count = 0;
        for (const auto& event_array : event_stream) {
            double first_event_time = event_array.events.front().ts.toSec();
            double last_event_time = event_array.events.back().ts.toSec();

            if (first_event_time > end_time) break;
            if (last_event_time < start_time) continue;

            for (const auto& event : event_array.events) {
                double event_time = event.ts.toSec();
                if (event_time >= start_time && event_time <= end_time &&
                    event.x >= 0 && event.x < sensor_width && event.y >= 0 && event.y < sensor_height) {
                    // TimeImage.at<double>(event.y, event.x)++;
                    // TimeImage.at<double>(event.y, event.x)++;
                    // count++;

                    // 2-5 修改 区分正负极性
                    if(!ignore_polarity && event.polarity)
                        TimeImage.at<double>(event.y, event.x)++;
                    else
                        TimeImage.at<double>(event.y, event.x)++;
                    // 2-14 修改 不使用负极性
                    // else
                    //     TimeImage.at<double>(event.y, event.x)--;
                    count++;
                }
            }
        }

        // cv::GaussianBlur(TimeImage, TimeImage, cv::Size(5, 5), 0.5);
        LOG(ERROR) << "TImage compress count = " << count << std::endl;

        // 假设 TimeImage1 已经被定义为 cv::Mat
        cv::Mat mask;
        cv::threshold(TimeImage, mask, 0, 1, cv::THRESH_BINARY); // 将大于 0 的像素设为 1，其他设为 0
        // 统计非零像素的数量
        int pix_count = cv::countNonZero(mask);
        LOG(ERROR) << "Debug: before ditort timeimage " << pix_count << std::endl;
        
        cv::threshold(TimeImage, mask, 0, 1, cv::THRESH_BINARY); // 将大于 0 的像素设为 1，其他设为 0
        pix_count = cv::countNonZero(mask);
        LOG(ERROR) << "Debug: after ditort timeimage " << pix_count << std::endl;
    }

        // 压缩时间图像
    bool AccumulateTimeImage(double timestamp) {
        TimeImage1 = cv::Mat::zeros(sensor_height, sensor_width, CV_64FC2);
        TimeImage1_events.clear();

        event_dt = AccumuConstractEventsWindows(timestamp, TimeImage1,  TimeImage1_events, t1_image_count);


        double window1_start = std::max(event_stream.front().header.stamp.toSec(), timestamp - 2 * event_dt);
        if(!(event_dt > 0 && event_t1 > 0))
        {
            LOG(ERROR) << "Failed: AccumulateTimeImage" << std::endl;
            return false;
        }
        RemoveOldEvents(window1_start);
        LOG(ERROR) << "event_dt = " << event_dt << ", event_t1 = " << event_t1 << std::endl;
        return true;
    }

    void gridBasedSampling(const std::vector<cv::Point2d>& nonZeroPoints,
                            std::vector<cv::Point2d>& sampledPoints,
                            int grid_size, int points_num) {
            sampledPoints.clear();
            
            std::unordered_map<int, std::unordered_map<int, cv::Point2f>> grid_map;
            std::random_device rd;
            std::mt19937 g(rd());
            std::vector<cv::Point2d> shuffledPoints = nonZeroPoints;
            
            std::shuffle(shuffledPoints.begin(), shuffledPoints.end(), g); // 随机打乱

            for (const auto& p : shuffledPoints) {
                int grid_x = static_cast<int>(p.x) / grid_size;
                int grid_y = static_cast<int>(p.y) / grid_size;
                
                if (grid_map[grid_x].count(grid_y) == 0) {
                    grid_map[grid_x][grid_y] = p; // 只选择每个网格中的第一个点
                    sampledPoints.push_back(p);
                }
                
                if (sampledPoints.size() >= points_num) break;
            }
        }

    bool CalculateOpFlowPrepointRANSAC() {
        int points_num = 50;
        const int max_iterations = 50;
        double min_cov_points = std::numeric_limits<double>::max();


        // 按位与操作
        cv::Mat result;
        // cv::bitwise_and(TimeImage1, TimeImage2, result);
        // TimeImage1 和 TimeImage2 是两个图像，我想取其中的与，统计非零的像素数量
        std::vector<cv::Point2d> nonZeroPoints;  
        cv::findNonZero(TimeImage1, nonZeroPoints);
        int nonZeroResult = nonZeroPoints.size();
        LOG(ERROR) << nonZeroResult << " pixels is same for motion " << std::endl; 

        LOG(ERROR) << "filter_num = " << filter_num << std::endl;
 
        // 筛选出事件数 > 1 的点
        std::unordered_map<int, std::unordered_map<int, cv::Point2d>> grid;
        std::vector<cv::Point2d> filteredPoints;
        for (const auto& pt : nonZeroPoints) {
            // LOG(ERROR) << "pt = (" << pt.x << "," << pt.y << ")" << std::endl;
            // LOG(ERROR) << "TimeImage1.at<double>(pt) = " << TimeImage1.at<double>(pt) << std::endl;
            // if (TimeImage1.at<float>(pt) > filter_num) {
            //     // LOG(ERROR) << "TimeImage1.at<double>(pt) = (" << pt.x << "," << pt.y << ")" << std::endl;
            //     filteredPoints.push_back(pt);
            // }

            int min_x = std::max(pt.x - radius, 0.0);
            int max_x = std::min(pt.x + radius, static_cast<double>(TimeImage1.cols-1));
            int min_y = std::max(pt.y - radius, 0.0);
            int max_y = std::min(pt.y + radius, static_cast<double>(TimeImage1.rows-1)); 
            
            int neriber_num = 0;
            for(int p_x = min_x; p_x <= max_x; p_x ++)
                for(int p_y = min_y; p_y <= max_y; p_y ++)
                {
                    if(TimeImage1.at<float>(p_y,p_x) > 0)
                        neriber_num ++;
                }
            int diameter = 2 * radius + 1;
            int actual_dis_x = max_x - min_x + 1;
            int actual_dis_y = max_y - min_y + 1;
            if(neriber_num > int(actual_dis_y * actual_dis_x * ratio_inliers))
            {
                filteredPoints.push_back(pt);
                LOG(ERROR) << "actual_dis_x = " << actual_dis_x << ", actual_dis_y = " << actual_dis_y << std::endl;
                LOG(ERROR) << "neriber_num = " << neriber_num << " need = " << int(actual_dis_y * actual_dis_x * ratio_inliers) << std::endl;
            }
        }
 
    
        LOG(ERROR) << "Choose " << filteredPoints.size() << " pixels for Optical Flow " << std::endl; 

        nonZeroPoints = filteredPoints;

        if(nonZeroResult < points_num)
        {
            LOG(ERROR) << "Event points are not enough, size = " << nonZeroPoints.size() 
            << "need for = " << points_num << std::endl;
            return false;
        }    

        LOG(ERROR) << "TimeImage1_events.SIZE = " << TimeImage1_events.size() << std::endl;

        for (int iter = 0; iter < max_iterations; ++iter) {
            std::vector<cv::Point2d> selected_points_temp;

            std::vector<cv::Point2d> selected_points;
            std::shuffle(nonZeroPoints.begin(), nonZeroPoints.end(), std::mt19937{std::random_device{}()});
        
            gridBasedSampling(nonZeroPoints, selected_points, 2 * radius + 1, points_num);
            

            std::vector<event_flow_velocity> flow_temp_vec;

            int rows = 0;
            int zeros_count = 0;
            double cov_points = 0;
            for (const auto& pt : selected_points) {
                int x = static_cast<int>(pt.x);
                int y = static_cast<int>(pt.y);

                // 平面拟合参数
                Eigen::Vector4d plane_p_ = Eigen::Vector4d::Zero();

                // 查找 
                std::vector<Eigen::Vector4d> a_collect;

                for (int i = -radius; i <= radius; ++i) {
                    for (int j = -radius; j <= radius; ++j) {
                        int pixel_x = x + j;
                        int pixel_y = y + i;
                        // LOG(ERROR) << "i = " << i << " j = " << j << std::endl;

                        // 确保不越界
                        if (pixel_x < 0 || pixel_x > TimeImage1.cols - 1 || pixel_y < 0 || pixel_y > TimeImage1.rows - 1) {
                            continue;
                        }
                        std::pair<int, int> search_coord = {pixel_x, pixel_y};

                        // LOG(ERROR) << "Searching for (" << pixel_x << ", " << pixel_y << ")";
                        // for (const auto& kv : TimeImage1_events) {
                        //     LOG(ERROR) << "Stored key: (" << kv.first.first << ", " << kv.first.second << ") -> " << kv.second;
                        // }

                        // double time = TimeImage1_events.find(search_coord);
                        double time;
                        auto it = TimeImage1_events.find(search_coord);
                        if (it != TimeImage1_events.end()) {
                            time = it->second;  // 访问 value
                            LOG(ERROR) << "find value" << std::endl;
                        } else {
                            // 处理 key 不存在的情况
                            // time = -1.0;  // 或者设置默认值
                            // LOG(ERROR) << "missing value" << std::endl;
                            continue;
                        }

                        Eigen::Vector4d event_p;
                        // event_p << pixel_x, pixel_y, time, 1.0;
                        // 去中心
                        event_p << j, i, time, 1.0;
                        a_collect.push_back(event_p);
                    }
                }
                
                if(a_collect.size() < 4)
                    continue;

                // 构建最小二乘法
                /*{
                    Eigen::MatrixXd A(a_collect.size(), 4);
                    for(int i = 0; i < a_collect.size(); i++)
                    {
                        A.row(i) = a_collect[i].transpose();
                    }
                    Eigen::VectorXd b(a_collect.size());

                    LOG(ERROR) << " A: " << A << std::endl;
                    LOG(ERROR) << " b: " << b << std::endl;
                }*/

                // 去中心的平面最小二乘
                Eigen::MatrixXd A(a_collect.size(), 3);
                for(int i = 0; i < a_collect.size(); i++)
                {
                    A.row(i) = a_collect[i].segment(0, 3).transpose();
                }
                Eigen::VectorXd b(a_collect.size());

                Eigen::Vector3d col_means = A.colwise().mean();
                col_means(0) = 0;
                col_means(1) = 0;
 
                LOG(ERROR) << " A: " << A << std::endl;
                LOG(ERROR) << " b: " << b << std::endl;   
                LOG(ERROR) << " A_means: " << col_means.transpose() << std::endl; 
                // A 的每列减去对应的平均值
                A = A.rowwise() - col_means.transpose();

                //
                double Grad_xx = A.col(0).transpose() * A.col(0);
                double Grad_xy = A.col(0).transpose() * A.col(1);
                double Grad_yy = A.col(1).transpose() * A.col(1);
                double Grad_xt = A.col(0).transpose() * A.col(2);
                double Grad_yt = A.col(1).transpose() * A.col(2);
                Eigen::Matrix2d A_M;
                A_M << Grad_xx, Grad_xy, Grad_xy, Grad_yy;
                Eigen::Vector2d b_M;
                b_M << Grad_xt, Grad_yt;

                // Eigen::Vector2d res = A_M.inverse() * b_M;
                Eigen::Vector2d res = A_M.ldlt().solve(b_M);

                res = -1.0 * res / res.squaredNorm();

                LOG(ERROR) << " A_M: " << A_M << std::endl;
                LOG(ERROR) << " b_M: " << b_M << std::endl;   
                LOG(ERROR) << " res: " << res << std::endl;
                
                /*
                {
                LOG(ERROR) << " A: " << A << std::endl;
                LOG(ERROR) << " b: " << b << std::endl;                

                // Eigen::Matrix4d AHA = A.transpose() * A;
                // Eigen::Vector4d AHb = A.transpose() * b;
                // plane_p_ = (AHA).inverse() * (AHb);

                Eigen::JacobiSVD<Eigen::MatrixXd> svd(A, Eigen::ComputeFullV);
                plane_p_ = svd.matrixV().col(3);

                // 2-25 修改
                // 计算法向光流
                plane_p_ = plane_p_ / plane_p_(2);      // 按第三列归一化,第三个参数是 1 

                double grad_norm = plane_p_(0) * plane_p_(0) + plane_p_(1) * plane_p_(1);
                event_flow_velocity flow(- plane_p_(0) / grad_norm, - plane_p_(1) / grad_norm);

                LOG(ERROR) << " local plane parameter: " << plane_p_.transpose() << std::endl;
                }*/

                event_flow_velocity flow(res(0), res(1));

                //获取光流 1 / A 1 / B
                // event_flow_velocity flow(- 1.0 * plane_p_(2)/ plane_p_(0), - 1.0 * plane_p_(2) / plane_p_(1));
                flow_temp_vec.push_back(flow);
                selected_points_temp.push_back(cv::Point2d(x, y));

                LOG(ERROR) << "flow = " << flow.x << " " << flow.y << std::endl;

            }

            // TODO: 是否可以进一步筛选
            {
                LOG(ERROR) << "cov_points = " << cov_points << ", min_cov_points = " << min_cov_points << std::endl;
                LOG(ERROR) << "selected_points.size() = " << selected_points.size() << std::endl;
                if (cov_points < min_cov_points) {
                    min_cov_points = cov_points;
                    best_inliers.clear();

                    flow_pre_points = flow_temp_vec; 
                    best_inliers = selected_points_temp;

                    assert(flow_pre_points.size() == best_inliers.size());
                    LOG(ERROR) << "First Debug for flow: best_inliers.size = " << best_inliers.size() << std::endl;
                }
            }
        }

        LOG(ERROR) << "RANSAC OpFlow: \n"
                  << "\t min_cov: " << min_cov_points 
                  << "\t inlier: " <<  best_inliers.size()
                  << std::endl;

        if(best_inliers.size() < 5)
        {
            LOG(ERROR) << "inliers is not enough!" << std::endl;
            return false; 
        }

        // debug
        LOG(ERROR) << "INFO all flow = " << std::endl;
        for(auto& f: flow_pre_points)
            LOG(ERROR) << "flow = " << f.x << ", " << f.y << std::endl;

        return true;       
    }


    bool CalculateOpFlowLocalPlane()
    {

    }

    // 计算角速度
    bool LSQAugularVelocityEsti(geometry_msgs::TwistWithCovarianceStamped& radar_vel) {
        // assert(best_inliers.size() > 2);

        Eigen::MatrixXd A(best_inliers.size(), 3);
        Eigen::VectorXd b(best_inliers.size());
        Eigen::Vector3d linear_vel(radar_vel.twist.twist.linear.x, radar_vel.twist.twist.linear.y, radar_vel.twist.twist.linear.z);

        if(linear_vel.norm() < 1e-6)
        {
            LOG(WARNING) << "radar ego velocity is valid!" << std::endl;
            return false;
        }

        LOG(ERROR) << "Mid Debug for flow: " << best_inliers.size() << std::endl;

        LOG(ERROR) << "K.inverse() = " << K.inverse() << std::endl;

        for (int i = 0; i < best_inliers.size(); ++i) {
            Eigen::Vector3d pixel(best_inliers[i].x , best_inliers[i].y, 1.0);
            Eigen::Vector3d pixel_cord = K.inverse() * pixel;
            Eigen::Matrix3d pixel_skew = skew(pixel_cord);
            Eigen::Vector3d prefix = pixel_skew * T_re.block(0, 0, 3, 3) * linear_vel;
            // Eigen::Vector3d flow(best_velocities[i].x, best_velocities[i].y, 0);

            // LOG(INFO) << "pixel = " << pixel.transpose() << std::endl;
            // LOG(INFO) << "K_inv = " << K.inverse() << std::endl;
            // LOG(INFO) << "pixel_skew = " << pixel_skew << std::endl;            
            // LOG(INFO) << "R_re = " << T_re.block(0, 0, 3, 3) << std::endl;

            // LOG(INFO) << "pixel_skew = " << pixel_skew.transpose() << std::endl;
            // LOG(INFO) << "R_re = " << T_re.block(0, 0, 3, 3) << std::endl;
            // LOG(INFO) << "linear_vel = " << linear_vel.transpose() << std::endl;

            LOG(ERROR) << "pixel_cord = " << pixel_cord.transpose() << std::endl;
            LOG(ERROR) << "linear_vel = " << linear_vel.transpose() << std::endl;
            LOG(ERROR) << "prefix = " << prefix.transpose() << std::endl;
        
            
            Eigen::Vector3d flow(flow_pre_points[i].x, flow_pre_points[i].y, 0);              // TODO: flow is 0??
            LOG(ERROR) << "flow = " << flow.transpose() << std::endl;

            // LOG(INFO) << "flow = " << flow.transpose() << std::endl;

            // TODO: 投影到归一化场景                 
            // flow(0) = flow(0) / K(0,0);
            // flow(1) = flow(1) / K(1,1);            
            flow = K.inverse() * flow;

            // LOG(INFO) << "flow cordinate = " << flow.transpose() << std::endl;

            A.row(i) = prefix.transpose() * pixel_skew;
            b(i) = -1.0 * prefix.transpose() * flow;    //TODO: modify in paper

            // LOG(INFO) << "A(i) = " << A.row(i).transpose() << std::endl;
            // LOG(INFO) << "b(i) = " << b(i) << std::endl;
        }

        Eigen::Vector3d angular_vec = A.colPivHouseholderQr().solve(b);

        // angular_vec = lpf.filter(angular_vec);

        LOG(ERROR) << "First Angular Velocity = " << std::endl;
        LOG(ERROR) << angular_vec.transpose() << std::endl;

        // 计算残差
        Eigen::VectorXd residual = A * angular_vec - b;

        // LOG(INFO) << "Angular Velocity Residual = " << residual << std::endl;

        // 加权迭代一次
        Eigen::VectorXd weights = residual.array().square().inverse();
        Eigen::MatrixXd W = weights.asDiagonal();
        Eigen::MatrixXd A_weighted = W * A;
        Eigen::VectorXd b_weighted = W * b;
        angular_vec = A_weighted.colPivHouseholderQr().solve(b_weighted);

        LOG(ERROR) << "Second Angular Velocity = " << std::endl;
        LOG(ERROR) << angular_vec.transpose() << std::endl;

        // 将事件系的角速度投影回雷达系
        angular_vec = T_re.block(0, 0, 3, 3).transpose() * angular_vec;

        radar_vel.twist.twist.angular.x = angular_vec(0);
        radar_vel.twist.twist.angular.y = angular_vec(1);
        radar_vel.twist.twist.angular.z = angular_vec(2);

        // 计算协方差矩阵
        residual = A_weighted * angular_vec - b_weighted;
        double sigma_r_squared = (residual.squaredNorm()) / (A.rows() - A.cols());
        Eigen::Matrix3d covariance_matrix = sigma_r_squared * (A.transpose() * A).inverse();
        // LOG(INFO) << "Angular Velocity covariance_matrix = " << covariance_matrix << std::endl;

        // 将协方差矩阵填充到 Covariance 中
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                radar_vel.twist.covariance[21 + i * 6 + j] = covariance_matrix(i, j);  // 填充角速度的协方差
            }
        }

        /*std::fstream optical_flow_file("/media/hao/hao2/228/test/lab/doc/optical_flow.txt", std::ios::out | std::ios::app);
        optical_flow_file << "calculate w: A * w = b " << std::endl;
        optical_flow_file << "inliners: " << best_inliers.size() << std::endl;
        optical_flow_file << "A = " << A << std::endl;
        optical_flow_file << "b = " << b << std::endl;
        optical_flow_file << "res = " << residual.transpose() << std::endl;
        optical_flow_file.close();*/

        // dji seq outut
        // LOG_Velocity(radar_vel, "/media/hao/hao2/228/test/lab/detector.tum");

        // dvs seq output
        LOG_Velocity(radar_vel, "/home/hao/Desktop/twist_ws/src/TwistEstimator/output/detector.tum");

        return true;
    }

    bool init_output = false;
    void LOG_Velocity(geometry_msgs::TwistWithCovarianceStamped& radar_vel, std::string filename)
    {
        if(!init_output)
        {
            /*std::fstream ss(filename, std::ios::out | std::ios::trunc);
            ss.clear();
            ss.close();*/
            // ss << "timestamp" << "," << "linear_x" << ", " << "linear_y" << ", " << "linear_z" << ", "
            //     << "angular_x" << ", " << "angular_y" << ", " << "angular_z" << std::endl;
            init_output = true;
            LOG(ERROR) << "create tum file: " << filename << std::endl;
        }

        // 创建一个字符串流
        std::fstream ss(filename, std::ios::out | std::ios::app);

        ss << std::setprecision(18) << radar_vel.header.stamp.toSec() << " ";
        ss << radar_vel.twist.twist.linear.x  << " ";
        ss << radar_vel.twist.twist.linear.y  << " ";
        ss << radar_vel.twist.twist.linear.z << " ";
        ss << radar_vel.twist.twist.angular.x << " ";
        ss << radar_vel.twist.twist.angular.y << " ";
        ss << radar_vel.twist.twist.angular.z << " ";

        // // 记录协方差矩阵
        for (int i = 0; i < 6; ++i) {
            for (int j = 0; j < 6; ++j) {
                ss << radar_vel.twist.covariance[i * 6 + j];
                if(!(i == 5 && j == 5)) 
                    ss << " ";
            }
            // ss << "\n";
        }
        ss << std::endl;
        // 使用 glog 记录信息
        ss.close();
    }

    void INFO_Velocity(geometry_msgs::TwistWithCovarianceStamped& radar_vel)
    {
        // 打印 header 信息
        LOG(ERROR) << "Detector: Angular Estimation:" << std::endl;
        LOG(ERROR) << "  Seq: " << radar_vel.header.seq << std::endl;
        LOG(ERROR) << "  Stamp: " << radar_vel.header.stamp << std::endl;
        LOG(ERROR) << "  Frame ID: " << radar_vel.header.frame_id << std::endl;

        // 打印 linear 线速度信息
        LOG(ERROR) << "Linear Velocity:" << std::endl;
        LOG(ERROR) << "  x: " << radar_vel.twist.twist.linear.x << std::endl;
        LOG(ERROR) << "  y: " << radar_vel.twist.twist.linear.y << std::endl;
        LOG(ERROR) << "  z: " << radar_vel.twist.twist.linear.z << std::endl;

        // 打印 angular 角速度信息
        LOG(ERROR) << "Angular Velocity:" << std::endl;
        LOG(ERROR) << "  x: " << radar_vel.twist.twist.angular.x << std::endl;
        LOG(ERROR) << "  y: " << radar_vel.twist.twist.angular.y << std::endl;
        LOG(ERROR) << "  z: " << radar_vel.twist.twist.angular.z << std::endl;

        // 打印协方差矩阵
        LOG(ERROR) << "Covariance: [";
        for (size_t i = 0; i < 36; ++i) {
            LOG(ERROR) << radar_vel.twist.covariance[i];
            if (i != 35)
                LOG(ERROR) << ", ";
        }
        LOG(ERROR) << "]" << std::endl;
    }


    // 检测主函数
    std::fstream time_file;
    int skip_radar_scan = 0;
    bool Detector() {
        // HAO: 数据交换
        {
            // std::lock_guard<std::mutex> lock(detector_data_mutex);
            // 如果传感器数据不足，返回
            if (radar_doppler_velocity.empty() || event_stream.empty() || radar_inliers.empty())
            { 
                // LOG(ERROR) << "data is not enough" << std::endl;
                return false;
            }

            // 数据对齐
            assert(radar_doppler_velocity.size() == radar_inliers.size() && "Doppler is not same as Inliers");

            process_time = radar_doppler_velocity.front().header.stamp;
            // LOG(ERROR) << "process event time = " << process_time << std::endl;

            // 如果事件相机不足，返回
            if (event_stream.back().header.stamp < process_time) {
                // std::cout << "event data is not new than radar" << std::endl;
                // LOG(ERROR) << "event data is not new than radar" << std::endl;
                // event_stream.clear();
                return false;
            }

            // 事件数据比较新，删除雷达数据
            if (event_stream.front().header.stamp > process_time) {
                // std::cout << "event data is not new than radar" << std::endl;
                LOG(ERROR) << "radar data is not new than event" << std::endl;
                radar_doppler_velocity.pop_front();
                radar_inliers.pop_front();
                return false;
            }
        
            assert(radar_doppler_velocity.front().header.stamp == radar_inliers.front().header.stamp);
            cur_inliers = radar_inliers.front();
            // LOG(ERROR) << "Check Time: " << std::setprecision(8) << process_time.toSec() << ", " << cur_inliers.header.stamp.toSec() << std::endl;
            // 点云数据比较旧，删除点云数据
            if (process_time > cur_inliers.header.stamp) {
                // std::cout << "event data is not new than radar" << std::endl;
                // LOG(ERROR) << "radar data is not new than event" << std::endl;
                radar_inliers.pop_front();
                return false;
            }  

            twist_ = radar_doppler_velocity.front();
            radar_doppler_velocity.pop_front(); 
            radar_inliers.pop_front();           
        }

        // return false;

        // std::cout << "start detector" << std::endl;
        
        // while (!radar_doppler_velocity.empty() && !event_stream.empty() &&
        //     event_stream.back().header.stamp > radar_doppler_velocity.front().header.stamp 
        //     && ros::ok()) 

        // debug
        // std::this_thread::sleep_for(std::chrono::milliseconds(2));
        // std::this_thread::sleep_for(std::chrono::seconds(2));

        {
            auto start = std::chrono::high_resolution_clock::now();
            // std::cout << "A" << std::endl;

            double process_time_sec = process_time.toSec();
            // CompressTimeImage(process_time_sec);

            bool accumu_done = AccumulateTimeImage(process_time_sec);

            if(!accumu_done)
            {
                LOG(ERROR) << "not enough events" << std::endl;;
                return false;
            }
            auto process_time = std::chrono::high_resolution_clock::now();
            // LOG(ERROR) << "process_time_sec = " << process_time_sec << std::endl;
            // std::cout << "process_time_sec = " << process_time_sec << std::endl;
            // double accumu_start_time = AccumuConstractEventsWindows(process_time_sec);
            // LOG(ERROR) << "accumu_start_time = " << accumu_start_time << std::endl;
            // std::cout << "accumu_start_time = " << accumu_start_time << std::endl;
            
            // if(accumu_start_time == -1)
            // {
            //     LOG(ERROR) << "Not enough Events" << std::endl;
            //     skip_radar_scan ++;
            //     return false;
            // }
            // LOG(ERROR) << "Successfuly, while skip " << skip_radar_scan << std::endl;
            // skip_radar_scan = 0;

            // else
            // {
            //     LOG(ERROR) << "accumu_start_time = " << accumu_start_time << std::endl;
            // }
            // VisualizeTimeImages(TimeImage1);
            // PublishTimeImages(TimeImage1);
            // std::cout << "VisualizeTimeImages " << std::endl;
            // RemoveOldEvents(process_time_sec);
            // std::cout << "RemoveOldEvents " << std::endl;
            // std::cout << "B" << std::endl;
            // bool have_flow = CalculateOpFlowRANSAC();

            bool have_flow = CalculateOpFlowPrepointRANSAC();
            auto optical_flow_time = std::chrono::high_resolution_clock::now();
            // bool have_flow = CalculateOpFlowLocalPlane();

            // bool have_flow = false;

            // bool have_flow = CalculateOpFlowPyramidRANSAC();
            // return false;
            // LK 光流
            // bool have_flow = CalculateSparseOpFlowOpenCV();
            // double time_diff = process_time_sec - accumu_start_time;
            // LOG(ERROR) << "time_diff = " << time_diff << std::endl;

            // bool have_flow = CalculateOpFlowForWindows(time_diff);

            
            // RLOF 光流
            // bool have_flow = CalculateDenseOpFlowOpenCV();

            // std::cout << "C" << std::endl;          
            // if(!(have_flow && AugularVelocityEsti(twist_)))
            if(!(have_flow && LSQAugularVelocityEsti(twist_)))
            {
                twist_.twist.twist.angular.x = 0.0f;
                twist_.twist.twist.angular.y = 0.0f;
                twist_.twist.twist.angular.z = 0.0f;
                LOG(ERROR) << "Angular Estimation Failed!" << std::endl;
                // 角速度不可用,尽保留线速度
                // return false;

                // 后端数据不输入
                best_inliers.clear();
                flow_pre_points.clear();
            } 
            auto angular_vel_time = std::chrono::high_resolution_clock::now();
            PublishTimeImages(TimeImage1);

            // else
            // {
            //     LOG(WARNING) << "Optical Flow Failed!" << std::endl;
            // }
            // AugularVelocityEstiRANSAC(radar_doppler_velocity.front());
            // std::cout << "D" << std::endl;
            // twist_ = radar_doppler_velocity.front();
            // radar_doppler_velocity.pop_front();

            auto end = std::chrono::high_resolution_clock::now();
            std::chrono::duration<double> elapsed = end - start;
            // time_file.open("/home/hao/Desktop/time.txt",std::ios::out | std::ios::app);
            // time_file << "Elapsed time: " << elapsed.count() << " seconds" << std::endl;
            // time_file.close();

            {
                LOG(ERROR) << "total detector: " << elapsed.count() << "seconds";
                elapsed = process_time - start;
                LOG(ERROR) << "process time: " << elapsed.count() << "seconds";
                elapsed = optical_flow_time - process_time;
                LOG(ERROR) << "optical flow: " << elapsed.count() << "seconds";
                elapsed = angular_vel_time - optical_flow_time;
                LOG(ERROR) << "angular vel:  " << elapsed.count() << "seconds";
                elapsed = end - angular_vel_time;
                LOG(ERROR) << "publish:      " << elapsed.count() << "seconds";                
            }

            // Prepare for back-end
            // std::cout << std::setprecision(20) << "twist_.header.stamp = " << twist_.header.stamp.toSec()
            //     << " radar_inliers.front().header.stamp = " << radar_inliers.front().header.stamp.toSec()
            //     << " radar_doppler_velocity.front().header.stamp = " << radar_doppler_velocity.front().header.stamp.toSec()
            //     << std::endl;
            assert(twist_.header.stamp == cur_inliers.header.stamp 
                && "radar_inliers not valid Or Time is not correct!");
            // twist_result_.push_back(TwistData(twist_, cur_inliers, best_inliers, flow_velocity));

            INFO_Velocity(twist_);


            LOG(ERROR) << "Final Debug for flow: " << best_inliers.size() << std::endl;
            std::vector<Eigen::Vector3d> normal_flows;
            std::vector<double> normal_norms;
            twist_result2_.push_back(TwistData2(twist_, cur_inliers, best_inliers, flow_pre_points, normal_flows, normal_norms));

        }

        return true;
    }

    // 获取光流速度
    event_flow_velocity GetFlowVelocity() const {
        if(twist_result_.empty())
            LOG(ERROR) << "Flow is not exist!" << std::endl;
            return {};
        return twist_result_.front().best_flow;
    }

    // TODO: 数据拷贝可以使用指针来进行
    // 获取光流速度
    geometry_msgs::TwistWithCovarianceStamped GetTwist() const {
        return twist_;
    }

    sensor_msgs::PointCloud2 GetInliers() {
        if(twist_result_.empty())
            LOG(ERROR) << "Inliers is not exist!" << std::endl;
            return {};
        return twist_result_.front().point_cloud;
    }

    // 获取检测过程数据(for tight optimization)
    // TwistData GetTwistData() {
    //     std::cout << std::setprecision(20) << "twist_.header.stamp = " << twist_.header.stamp.toSec()
    //               << " radar_inliers.front().header.stamp = " << radar_inliers.front().header.stamp.toSec()
    //               << " radar_doppler_velocity.front().header.stamp = " << radar_doppler_velocity.front().header.stamp.toSec()
    //               << std::endl;
    //     assert(twist_.header.stamp == radar_inliers.front().header.stamp 
    //     && "radar_inliers not valid Or Time is not correct!");
    //     sensor_msgs::PointCloud2 inliers = radar_inliers.front();
    //     radar_doppler_velocity.pop_front();
    //     radar_inliers.pop_front();
    //     return TwistData(twist_, inliers, best_inliers, flow_velocity);          
    // }

    TwistData GetTwistData() {
        TwistData twist_result_temp_ = twist_result_.front();
        twist_result_.pop_front();
        return twist_result_temp_;          
    }

    TwistData2 GetTwistData2() {
        TwistData2 twist_result_temp_ = twist_result2_.front();
        twist_result2_.pop_front();
        return twist_result_temp_;          
    }

// 成员变量
public:
    std::deque<dvs_msgs::EventArray::Ptr> event_stream;
    std::deque<geometry_msgs::TwistWithCovarianceStamped> radar_doppler_velocity;
    std::deque<sensor_msgs::PointCloud2::Ptr> radar_inliers;
    std::deque<TwistData> twist_result_;

    std::deque<TwistData2> twist_result2_;

    const long int constract_events = 500; //500;
    
    ros::NodeHandle& GetNodeHandle()
    {return this->nh_this_;}

private:
    // 成员变量
    double event_dt;
    double event_t1;
    cv::Mat K_cv;          // intrincs matrix in opencv
    cv::Mat distort_cv;
    Eigen::Matrix3d K;      // intrincs matrix
    Eigen::Matrix3d distort;      // distort parameters
    int sensor_width;
    int sensor_height;
    Eigen::Matrix4d T_re;

    cv::Mat TimeImage1;
    cv::Mat TimeImage2;

    std::unordered_map<std::pair<int, int>, double, pair_hash> TimeImage1_events;

    cv::Mat TimeImage1_color;

    cv::Mat TimeImage1_on_event;
    cv::Mat TimeImage1_off_event;
    
    event_flow_velocity flow_velocity;
    // std::vector<double*> flow_velocity;
    std::vector<event_flow_velocity> flow_pre_points;
    int radius;
    double ratio_inliers;
    bool ignore_polarity;

    std::vector<cv::Point2d> best_inliers;

    long int t1_image_count;
    long int t2_image_count;

    bool show_events_ = false;

    geometry_msgs::TwistWithCovarianceStamped twist_;

    LowPassFilter lpf;

    LowPassFilter lpf_v;

    int filter_num;

    ros::NodeHandle nh_this_;
    cv_bridge::CvImage cv_image_;           // transport TimeImage
    ros::Publisher image_pub_;

    ros::Time process_time; 

    ros::Publisher pub_event_image_;

    // 存储所有有效光流
    std::vector<Eigen::Vector2d> valid_flows;  

    sensor_msgs::PointCloud2 cur_inliers;

    // 生成随机像素点
    void RandomlySelectPoints(const cv::Mat& image, std::vector<cv::Point2d>& points, int& points_num) {
        std::vector<cv::Point2d> all_points;
        for (int y = radius; y < image.rows - radius; ++y) {
            for (int x = radius; x < image.cols - radius; ++x) {
                if (image.at<double>(y, x) > 1e-4) {
                    all_points.emplace_back(x, y);
                }
            }
        }

        // std::cout << "pixel valid = " << all_points.size() << std::endl;
        if (all_points.size() < points_num)
        {
            LOG(ERROR) << "point num is not enough for random select, need = " 
                    << points_num << ", actual = " << all_points.size() << std::endl;
            return;
        }
        // points_num = std::min(points_num, static_cast<int>(all_points.size()));

        // std::cout << "all_point = " << all_points.size() << std::endl;

        std::random_shuffle(all_points.begin(), all_points.end());
        points.assign(all_points.begin(), all_points.begin() + points_num);

        // points_num = points.size();
    }

    // 生成按事件累积数排序的像素点
    void SelectTopEventPoints(const cv::Mat& image, std::vector<cv::Point2d>& points, int& points_num) {
        std::vector<std::pair<cv::Point2d, double>> point_event_counts;

        // 计算每个点的事件累积数
        for (int y = radius; y < image.rows - radius; ++y) {
            for (int x = radius; x < image.cols - radius; ++x) {
                double event_count = image.at<double>(y, x);
                if (abs(event_count) > 1e-4) {
                    point_event_counts.emplace_back(cv::Point2d(x, y), event_count);
                }
            }
        }

        // 检查是否有足够的点
        if (point_event_counts.size() < points_num) {
            LOG(ERROR) << "point num is not enough for selection, need = " 
                    << points_num << ", actual = " << point_event_counts.size() << std::endl;
            return;
        }

        // 按事件累积数排序
        std::sort(point_event_counts.begin(), point_event_counts.end(),
                [](const std::pair<cv::Point2d, double>& a, const std::pair<cv::Point2d, double>& b) {
                    return a.second > b.second; // 降序排序
                });

        // 选取前 points_num 个点
        points.clear();
        for (int i = 0; i < points_num; ++i) {
            points.push_back(point_event_counts[i].first);
        }
    } 

    // 构建反对称矩阵
    Eigen::Matrix3d skew(const Eigen::Vector3d& vec) const {
        Eigen::Matrix3d result;
        result << 0, -vec(2), vec(1), vec(2), 0, -vec(0), -vec(1), vec(0), 0;
        return result;
    }
};


#endif