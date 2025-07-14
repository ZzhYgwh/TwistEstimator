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

#include <image_geometry/pinhole_camera_model.h>

#include <thread>

#include <mutex>
#include <omp.h>
#include <unordered_set>


// just for debug
#include <filesystem>

#include "event_flow_detector.h"


struct pair_hash {
    template <class T1, class T2>
    std::size_t operator()(const std::pair<T1, T2>& p) const {
        return std::hash<T1>{}(p.first) ^ (std::hash<T2>{}(p.second) << 1);
    }
};


class EventFlowDetector3 {
public:
    EventFlowDetector3(ros::NodeHandle& nh_this, EventParams& e_params, RadarEventParams r_params, bool show_events, double smooth, int filter_num = 3, bool ignore_polarity = false, double ratio_inliers = 0.1)
        : nh_this_(nh_this), event_dt(e_params.deltaT), event_t1(e_params.t1), sensor_width(e_params.resolution[0]), lpf(smooth), lpf_v(smooth), filter_num(filter_num), ignore_polarity(ignore_polarity), ratio_inliers(ratio_inliers),
        sensor_height(e_params.resolution[1]), t1_image_count(e_params.t1_count), t2_image_count(e_params.t2_count), T_re(r_params.T_re), radius(e_params.mesh_size), show_events_(show_events) 
        {
            K << e_params.fx, 0, e_params.cx,
                0, e_params.fy, e_params.cy,
                0,0,1;

            K_cv = cv::Mat(3, 3, CV_64F, K.transpose().data()).clone(); // 使用 .clone() 以确保数据独立
            K_cv = K_cv.t();

            // distort_cv = (cv::Mat_<double>(1, 5) << e_params.k1, e_params.k2, e_params.p1, e_params.p2, e_params.k3);
            // LOG(ERROR) << "distort_cv = " << e_params.k1 << " " 
            //                                 << e_params.k2 << " "
            //                                 << e_params.p1 << " " 
            //                                 << e_params.p2 << " " 
            //                                 << e_params.k3 << std::endl;

            image_pub_ = nh_this.advertise<sensor_msgs::Image>("/event/flow_img", 10);  
            pub_event_image_ = nh_this.advertise<sensor_msgs::Image>("/event/img", 10);
            pub_raw_image_ = nh_this.advertise<sensor_msgs::Image>("/dvs/raw_img", 10);

            event_buffer.reserve(100000);
            raw_img_buffer.reserve(100000);
            
            cam_info.height = sensor_height;
            cam_info.width = sensor_width;
            // cam_info.K = K_cv;
            std::copy(K_cv.ptr<double>(), K_cv.ptr<double>() + 9, cam_info.K.begin());

            for (size_t i = 0; i < 9; ++i) {
                LOG(ERROR) << cam_info.K[i] << " ";
            }

            // 直接赋值
            cam_info.distortion_model = "plumb_bob";
            cam_info.D = {e_params.k1, e_params.k2, e_params.p1, e_params.p2, e_params.k3};

            LOG(ERROR) << "Distortion Coefficients: ";
            for (size_t i = 0; i < cam_info.D.size(); i++) {
                LOG(ERROR) << cam_info.D[i] << " ";
            }
            LOG(ERROR) << std::endl;

            // Set P matrix (assuming no stereo calibration)
            cam_info.P[0] = cam_info.K[0];
            cam_info.P[1] = cam_info.K[1];
            cam_info.P[2] = cam_info.K[2];
            cam_info.P[3] = 0.0;
            cam_info.P[4] = cam_info.K[3];
            cam_info.P[5] = cam_info.K[4];
            cam_info.P[6] = cam_info.K[5];
            cam_info.P[7] = 0.0;
            cam_info.P[8] = cam_info.K[6];
            cam_info.P[9] = cam_info.K[7];
            cam_info.P[10] = cam_info.K[8];
            cam_info.P[11] = 0.0;

            // Set R matrix to identity
            cam_info.R[0] = cam_info.R[4] = cam_info.R[8] = 1.0;
            cam_info.R[1] = cam_info.R[2] = cam_info.R[3] = 0.0;
            cam_info.R[5] = cam_info.R[6] = cam_info.R[7] = 0.0;

            // memcpy(cam_info.K.data(), K_cv.ptr<double>(), 9 * sizeof(double));
            camera_model.fromCameraInfo(cam_info);


            cv_image_.encoding = "bgr8"; // 假设处理的是彩色图像
            cv_image_.header.frame_id = "camera_frame";

            // Init
            TimeImage1 = cv::Mat::zeros(sensor_height, sensor_width, CV_64FC1);
            TimeImage1_time = cv::Mat::zeros(sensor_height, sensor_width, CV_64FC1);
            TimeImage1_events.reserve(1.2 * t1_image_count);  // 预分配 20 万个 hash bucket

            // LOG(ERROR) << "radius = " << radius << std::endl;
        }


    bool compareTimeStamp(const sensor_msgs::Image::Ptr& lhs, const sensor_msgs::Image::Ptr& rhs)
    {
        return lhs->header.stamp.toSec() < rhs->header.stamp.toSec();
    }
    void GetImage(sensor_msgs::Image::Ptr& raw_img)
    {
        if (process_time.isZero() || raw_img_buffer.empty())
        {
            LOG(ERROR) << "No Need Image" << std::endl;
            return; // 不处理
        }
        LOG(ERROR) << "Search" << std::endl;
        // 使用 std::lower_bound 查找最接近的时间戳
        auto it = std::lower_bound(raw_img_buffer.begin(), raw_img_buffer.end(), process_time,
                                [](const sensor_msgs::Image::Ptr& img, const ros::Time& time) {
                                    return img->header.stamp.toSec() < time.toSec();
                                });
        LOG(ERROR) << "Search Done" << std::endl;
        if (it != raw_img_buffer.end())
        {
            // 比较当前位置前后的帧，选择时间差最小的帧
            auto best_it = it;
            if (it != raw_img_buffer.begin())
            {
                auto prev_it = std::prev(it);
                double time_diff_prev = std::abs((*prev_it)->header.stamp.toSec() - process_time.toSec());
                double time_diff_curr = std::abs((*it)->header.stamp.toSec() - process_time.toSec());

                if (time_diff_prev < time_diff_curr)
                {
                    best_it = prev_it;
                }
            }

            raw_img = *best_it;

            // 删除所有较旧的帧
            raw_img_buffer.erase(raw_img_buffer.begin(), best_it + 1);
            if(*(best_it) == nullptr) 
                  LOG(ERROR) << "invalid image" << std::endl;

            LOG(ERROR) << "Selected image with min_time_diff = "
                    << std::abs((*best_it)->header.stamp.toSec() - process_time.toSec()) << std::endl;
        }
    }



    void EventArray2EventVec()
    {
        // std::lock_guard<std::mutex> lock(callback_mutex);
        {
            // LOG(ERROR) << "event_stream.size = " << event_stream.size() << std::endl;
            // 取出事件流的所有事件，避免持锁时间过长
            for (const auto& event_array : event_stream) {
                event_buffer.insert(event_buffer.end(), event_array->events.begin(), event_array->events.end());
            }
            event_stream.clear();
        }
        // LOG(ERROR) << "restore " << event_buffer.size() << std::endl;
    }


    void RemoveOldEvents(double threshold_time)
    {
        // auto it = std::find_if(event_buffer.begin(), event_buffer.end(),
        //                    [threshold_time](const dvs_msgs::Event& e) { return e.ts.toSec() > threshold_time; });

        auto it = std::lower_bound(event_buffer.begin(), event_buffer.end(), threshold_time,
            [](const dvs_msgs::Event& e, double t) {
                return e.ts.toSec() <= t;
            });
        if(it != event_buffer.begin())
            event_buffer.erase(event_buffer.begin(), it);
        // else
        //     LOG(ERROR) << "do not erase any event"<< std::endl;
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

/*
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

            velocity.x *= cam_info.K[0];
            velocity.y *= cam_info.K[4];

            // View neighborhood information
            // drawSquareBorder(color_img, point, radius);

            // 从 color_img 中分割 [point - radius, point + radius] 范围大小的局部邻域为flow_patch
            /*
            double radius_test = 1.0 * radius;
            // 计算局部邻域的矩形区域
            int x_start = std::max(static_cast<double>(0), point.x - radius_test);
            int y_start = std::max(static_cast<double>(0), point.y - radius_test);
            int x_end = std::min(static_cast<double>(color_img_temp.cols), point.x + radius_test + 1);
            int y_end = std::min(static_cast<double>(color_img_temp.rows), point.y + radius_test + 1);
            cv::Rect roi(x_start, y_start, x_end - x_start, y_end - y_start);
            cv::Mat flow_patch = color_img(roi).clone();
            

            cv::Point2d end_point(point.x + velocity.x, point.y + velocity.y);
            end_point.x = std::min(std::max(0.0, end_point.x), static_cast<double>(color_img.cols - 1));
            end_point.y = std::min(std::max(0.0, end_point.y), static_cast<double>(color_img.rows - 1));

            cv::Point2d edge_start_point(point.x - velocity.y, point.y + velocity.x);
            cv::Point2d edge_end_point(point.x - velocity.y, point.y - velocity.x);
            cv::arrowedLine(color_img, edge_start_point, edge_end_point, cv::Scalar(0, 0, 255), 1, 8, 0, 0.1);

            cv::arrowedLine(color_img, point, end_point, cv::Scalar(0, 255, 0), 1, 8, 0, 0.1);

            // for (size_t i = 0; i < flow_pre_points.size(); ++i) {
            //     cv::imwrite("/media/hao/hao2/228/test/lab/flow_img/flowimage_" + std::to_string(img_index) 
            //     + "_" + std::to_string(i) + "_" + std::to_string(velocity.x) + "_" + std::to_string(velocity.y) + ".png", flow_patch);
            // }
        }

        img_index ++;

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
*/

/*
    void PublishTimeImages(cv::Mat pub_img, cv::Mat time_img) {
        // 创建一个与 pub_img 相同尺寸的彩色图像，初始化为黑色
        cv::Mat color_img = cv::Mat::zeros(pub_img.size(), CV_8UC3);

        sensor_msgs::Image::Ptr raw_img_msg_ptr;
        GetImage(raw_img_msg_ptr);
        cv::Mat back_img;
        // cv_bridge sensor_msgs::Image raw_img_msg; 2 cv::Mat back_img
        back_img = cv_bridge::toCvCopy(*raw_img_msg_ptr, "bgr8")->image;
    
        // cv::Mat color_img = pub_img.clone();
        // 创建掩膜，分别表示大于0、小于0和等于0的区域
        cv::Mat mask_pos = pub_img > 0;  // 大于0的区域
        cv::Mat mask_neg = pub_img < 0;  // 小于0的区域
        cv::Mat mask_zero = pub_img == 0; // 等于0的区域

        // 将大于0的区域设置为红色 (BGR: 0, 0, 255)
        color_img.setTo(cv::Vec3b(0, 0, 255), mask_pos);

        // 将小于0的区域设置为蓝色 (BGR: 255, 0, 0)
        color_img.setTo(cv::Vec3b(255, 0, 0), mask_neg);

        // 对 time_img 进行归一化，假设 time_img 的值范围为 [min_value, max_value]
        double min_val, max_val;
        cv::minMaxLoc(time_img, &min_val, &max_val);  // 获取 time_img 的最小值和最大值
        cv::Mat normalized_time_img;
        normalized_time_img = (time_img - min_val) / (max_val - min_val) * 80 + 175;
        // time_img.convertTo(normalized_time_img, CV_64FC1, 1.0 / (max_val - min_val));  // 归一化到 [0, 1]
        
        // 使用 normalized_time_img 替代 color_img 的红色通道
        for (int y = 0; y < normalized_time_img.rows; ++y) {
            for (int x = 0; x < normalized_time_img.cols; ++x) {
                int red_value = cv::saturate_cast<int>(normalized_time_img.at<double>(y, x));  // 获取红色通道的值

                // 设置 color_img 的红色通道
                color_img.at<cv::Vec3b>(y, x)[2] = red_value;  // OpenCV 使用 BGR 顺序，所以红色通道是索引 2
            }
        }

        // 将等于0的区域设置为黑色 (BGR: 0, 0, 0) -> 已经是默认值
        // color_img.setTo(cv::Vec3b(255, 255, 255), mask_zero);
        // LOG(ERROR) << "Size mismatch: back_img.size() = " << back_img.cols << "x" << back_img.rows  
        //        << ", pub_img.size() = " << pub_img.cols << "x" << pub_img.rows;
        if (!back_img.empty() && back_img.size() == pub_img.size() && back_img.type() == CV_8UC3)
        {
            back_img.copyTo(color_img, mask_zero);  // 仅在 mask_zero 处覆盖
        }
                
        // 设置字体参数
        int font_face = cv::FONT_HERSHEY_SIMPLEX;
        double font_scale = 0.3;  // 字体大小
        int thickness = 1;        // 字体厚度
        cv::Scalar color(255, 255, 255);  // 文本颜色：白色


        static long int img_index = 0;
        cv::Size target_size(25, 25);
        for (size_t i = 0; i < flow_pre_points.size(); ++i) {
            // cv::Mat color_img_temp = color_img.clone();

            cv::Point2d point = best_inliers[i];
            event_flow_velocity velocity = flow_pre_points[i];

            velocity.x *= cam_info.K[0];
            velocity.y *= cam_info.K[4];

            // View neighborhood information
            // drawSquareBorder(color_img, point, radius);

            // 从 color_img 中分割 [point - radius, point + radius] 范围大小的局部邻域为flow_patch
            /*double radius_test = 1.0 * radius;
            // 计算局部邻域的矩形区域
            int x_start = std::max(static_cast<double>(0), point.x - radius_test);
            int y_start = std::max(static_cast<double>(0), point.y - radius_test);
            int x_end = std::min(static_cast<double>(color_img_temp.cols), point.x + radius_test + 1);
            int y_end = std::min(static_cast<double>(color_img_temp.rows), point.y + radius_test + 1);
            cv::Rect roi(x_start, y_start, x_end - x_start, y_end - y_start);
            cv::Mat flow_patch = color_img(roi).clone();

            Eigen::Vector2d velocity_normalize;
            velocity_normalize << velocity.x, velocity.y;
            velocity_normalize.normalize();

            cv::Point2d end_point(point.x + velocity.x, point.y + velocity.y);
            end_point.x = std::min(std::max(0.0, end_point.x), static_cast<double>(color_img.cols - 1));
            end_point.y = std::min(std::max(0.0, end_point.y), static_cast<double>(color_img.rows - 1));

            cv::Point2d edge_start_point(point.x + 10 * velocity_normalize(1), point.y - 10 * velocity_normalize(0));
            cv::Point2d edge_end_point(point.x - 10 * velocity_normalize(1), point.y + 10 * velocity_normalize(0));
            // cv::arrowedLine(color_img, edge_start_point, edge_end_point, cv::Scalar(0, 0, 255, 255), 1, 8, 0, 0.1);
            // cv::line(color_img, edge_start_point, edge_end_point, cv::Scalar(0, 0, 255, 255), 1, 8, 0, 0.1);
            // cv::line(color_img, edge_start_point, edge_end_point, cv::Scalar(0, 0, 255, 255), 1, 8, 0);

            cv::line(color_img, edge_start_point, edge_end_point, cv::Scalar(255, 0, 0), 1, 8, 0);

            // cv::arrowedLine(color_img, point, end_point, cv::Scalar(0, 255, 0, 255), 1, 8, 0, 0.1);
            cv::arrowedLine(color_img, point, end_point, cv::Scalar(0, 255, 0), 1, 8, 0, 0.0);

            /*for (size_t i = 0; i < flow_pre_points.size(); ++i) {
                cv::imwrite("/media/hao/hao2/228/test/lab/flow_img/flowimage_" + std::to_string(img_index) 
                + "_" + std::to_string(i) + "_" + std::to_string(velocity.x) + "_" + std::to_string(velocity.y) + ".png", flow_patch);
            }
        }

        img_index ++;

        // std::string angular_data = "Angular: x=" + std::to_string(twist_.twist.twist.angular.x) +
        //                     " y=" + std::to_string(twist_.twist.twist.angular.y) +
        //                     " z=" + std::to_string(twist_.twist.twist.angular.z);
        // cv::Point text_position(20, 10); // 选择一个适当的位置
        // cv::putText(color_img, angular_data, text_position, cv::FONT_HERSHEY_SIMPLEX, 0.35, cv::Scalar(255, 255, 255), 0.75);
        

        // cv::imwrite("/media/hao/hao2/228/test/lab/img/timeimage_" + std::to_string(img_index) + ".png", color_img);
        

        // 转为 sensor_msgs::Image 中
        std_msgs::Header header;
        header.stamp = ros::Time::now();
        header.frame_id = "event";

        // 将 color_img 转换为 ROS 消息
        sensor_msgs::ImagePtr msg = cv_bridge::CvImage(header, "bgr8", color_img).toImageMsg();

        // 你可以在这里发布这个 msg 或者进一步处理它
        pub_event_image_.publish(msg);

        /*
            sensor_msgs::Image raw_img_msg;
            GetImage(raw_img_msg);
            raw_img_msg.header.stamp = ros::Time::now();
            pub_raw_image_.publish(raw_img_msg);
        

        // ROS_INFO("Publish Time Image");
        LOG(ERROR) << "Publish Time Image" << std::endl;
    }*/


    void PublishTimeImages(cv::Mat pub_img, cv::Mat time_img) {
        // 创建一个与 pub_img 相同尺寸的彩色图像，初始化为黑色
        cv::Mat color_img = cv::Mat::zeros(pub_img.size(), CV_8UC3);

        // 获取背部图像
        LOG(ERROR) << "try to GetImage " << std::endl;
        sensor_msgs::Image::Ptr raw_img_msg_ptr;
        GetImage(raw_img_msg_ptr);
        LOG(ERROR) << "GetImage " << std::endl;
        cv::Mat back_img;
        if(raw_img_msg_ptr != nullptr)
            back_img = cv_bridge::toCvCopy(*raw_img_msg_ptr, "bgr8")->image;
        else
        {
            LOG(ERROR) << "raw_img_msg_ptr have no image " << std::endl;
            back_img = cv::Mat(sensor_height, sensor_width, CV_8UC3, cv::Scalar(0, 0, 0));
        }
        

        // 计算大于0、小于0和等于0的区域掩膜
        cv::Mat mask_pos = pub_img > 0;  // 大于0的区域
        cv::Mat mask_neg = pub_img < 0;  // 小于0的区域
        cv::Mat mask_zero = pub_img == 0; // 等于0的区域

        // 合并区域颜色设置
        color_img.setTo(cv::Vec3b(0, 0, 255), mask_pos);  // 红色
        color_img.setTo(cv::Vec3b(255, 0, 0), mask_neg);  // 蓝色

        // 归一化 time_img
        cv::Mat normalized_time_img;
        cv::normalize(time_img, normalized_time_img, 175, 255, cv::NORM_MINMAX);

        // 使用归一化的时间图像替代红色通道
        normalized_time_img.convertTo(normalized_time_img, CV_8UC1); // 转换为 8 位单通道
        for (int y = 0; y < normalized_time_img.rows; ++y) {
            for (int x = 0; x < normalized_time_img.cols; ++x) {
                int red_value = normalized_time_img.at<uchar>(y, x);
                color_img.at<cv::Vec3b>(y, x)[2] = red_value;
            }
        }

        // 覆盖黑色区域（如果需要）
        if (!back_img.empty() && back_img.size() == pub_img.size() && back_img.type() == CV_8UC3) {
            back_img.copyTo(color_img, mask_zero);  // 仅覆盖 zero 区域
        }

        // 绘制流动箭头
        for (size_t i = 0; i < flow_pre_points.size(); ++i) {
            cv::Point2d point = best_inliers[i];
            event_flow_velocity velocity = flow_pre_points[i];

            velocity.x *= cam_info.K[0];
            velocity.y *= cam_info.K[4];

            // 归一化 velocity
            Eigen::Vector2d velocity_normalize;
            velocity_normalize << velocity.x, velocity.y;
            velocity_normalize.normalize();

            // LOG(ERROR) << "velocity.x = " << velocity.x << ", velocity.y = " << velocity.y << std::endl;

            // 计算箭头末端位置
            cv::Point2d end_point(point.x + velocity.x, point.y + velocity.y);
            end_point.x = std::min(std::max(0.0, end_point.x), static_cast<double>(color_img.cols - 1));
            end_point.y = std::min(std::max(0.0, end_point.y), static_cast<double>(color_img.rows - 1));

            // 绘制箭头
            cv::Point2d edge_start_point(point.x + 10 * velocity_normalize(1), point.y - 10 * velocity_normalize(0));
            cv::Point2d edge_end_point(point.x - 10 * velocity_normalize(1), point.y + 10 * velocity_normalize(0));
            cv::line(color_img, edge_start_point, edge_end_point, cv::Scalar(255, 0, 0), 1, 8, 0);
            cv::arrowedLine(color_img, point, end_point, cv::Scalar(0, 255, 0), 1, 8, 0, 0.0);
        }

        // ROS 消息转换并发布
        std_msgs::Header header;
        header.stamp = ros::Time::now();
        header.frame_id = "event";

        sensor_msgs::ImagePtr msg = cv_bridge::CvImage(header, "bgr8", color_img).toImageMsg();
        pub_event_image_.publish(msg);
    }


    double AccumuConstractEventsWindows(double timestamp,
                                        cv::Mat& time_img, 
                                        std::unordered_map<std::pair<int, int>, std::vector<double>, pair_hash>& event_link, 
                                        cv::Mat& image_time,
                                        long int need_events = 1000)
    {
            // 使用 std::sort 按 ts 进行升序排序
            // std::sort(event_buffer.begin(), event_buffer.end(), 
            //     [](const dvs_msgs::Event &a, const dvs_msgs::Event &b) { return a.ts < b.ts; });

            // LOG(ERROR) << "timestamp = " << std::setprecision(20) << timestamp << " it = " << it - event_buffer.begin() << std::endl;

            // LOG(ERROR) << "timestamp = " << timestamp << std::endl;
            // auto it = std::find_if(event_buffer.begin(), event_buffer.end(),
            //                         [timestamp](const dvs_msgs::Event& e) { return e.ts.toSec() > timestamp; });

            auto it = std::lower_bound(event_buffer.begin(), event_buffer.end(), timestamp,
                [](const dvs_msgs::Event& e, double t) {
                    return e.ts.toSec() <= t;
                });

            // LOG(ERROR) << "A = " << it - event_buffer.begin() + 1 << " 1.2 * need_events = " << 1.2 * need_events << std::endl;
            int start_p = it - event_buffer.begin() - 1;
            if(start_p < 1.2 * need_events)
            {
                // LOG(ERROR) << "No valid events" << std::endl;
                return -1;
            }
            // if(event_buffer.size() < 1.2 * need_events)
            //     return -1;

            // 使用下面的值初始化
            double event_buffer_front_t = event_buffer.front().ts.toSec();
            // LOG(ERROR) << "event_buffer_front_t = " << event_buffer_front_t << std::endl; 
            // image_time = cv::Mat::ones(sensor_height, sensor_width, CV_64FC1) * event_buffer_front_t;
;
            image_time.setTo(event_buffer_front_t);

            // time_img = cv::Mat::zeros(sensor_height, sensor_width, CV_64FC1);
    
            // 事件累计计算
            long int accumu_count = 0;
            long int count_not_in_img = 0;
            long int rectify_fault = 0;
            cv::Rect roi(0, 0, sensor_width, sensor_height);
            for (int i = start_p; i > 0; i--) {
                // LOG(ERROR) << "end at: = " << std::max(static_cast<long int>(start_p - need_events), static_cast<long int>(0));
                // LOG(ERROR) << " i = " << i << std::endl; 
                // if(i < 0)
                //     return -1;

                const auto e = event_buffer[i];

                // 去畸变
                cv::Point2d e_p;
                e_p.x = static_cast<double>(e.x);
                e_p.y = static_cast<double>(e.y);
                
                // LOG(ERROR) << "e_p = " << e_p.x << ", " << e_p.y << std::endl;
                // (static_cast<double>(e.x), static_cast<double>(e.y));   
                // cv::Point2d undistorted_e_p = camera_model.rectifyPoint(e_p);
                cv::Point2d undistorted_e_p = e_p;

                // LOG(ERROR) << "undistorted_e_p.x = " << undistorted_e_p.x << std::endl;   
                // LOG(ERROR) << "undistorted_e_p.y = " << undistorted_e_p.y << std::endl;
                /*if(undistorted_e_p.x < 0 || undistorted_e_p.x > sensor_width - 1 
                    || undistorted_e_p.y < 0 || undistorted_e_p.y > sensor_height - 1 
                    || std::isnan(undistorted_e_p.x) || std::isnan(undistorted_e_p.y))
                    {
                        // i--;
                        // need_events++;
                        // rectify_fault++;
                        continue;
                    }*/
                
                if (!roi.contains(undistorted_e_p)) {
                    continue;
                }


                // if(undistorted_e_p.x < 2 * radius || sensor_width - undistorted_e_p.x < 2 * radius 
                //         || undistorted_e_p.y < 2 * radius || sensor_height - undistorted_e_p.y < 2 * radius)
                // {
                //     // event_buffer.erase(event_buffer.begin() + i);
                //     // i --;
                //     // need_events ++;
                //     // count_not_in_img++;
                //     continue;
                // }

                // std::pair<int, int> key = {undistorted_e_p.x, undistorted_e_p.y};

                if (ignore_polarity || e.polarity) {
                    int u = static_cast<int>(undistorted_e_p.x);
                    int v = static_cast<int>(undistorted_e_p.y);
                    std::pair<int, int> key = {u, v};

                    double* time_img_ptr = time_img.ptr<double>(v);
                    double* image_time_ptr = image_time.ptr<double>(v);

                    time_img_ptr[u]++;

                    double current_time = e.ts.toSec();
                    // image_time_ptr[u] = std::max(current_time, image_time_ptr[u]);
                    if (current_time > image_time_ptr[u]) {
                        image_time_ptr[u] = current_time;
                    }

                    event_link[key].push_back(current_time);
                }


                // 根据极性进行事件累计
                /*if(!ignore_polarity && e.polarity)
                {
                    int u = static_cast<int>(undistorted_e_p.x);
                    int v = static_cast<int>(undistorted_e_p.y);

                    if (v >= 0 && v < sensor_height && u >= 0 && u < sensor_width) {
                        double* time_img_ptr = time_img.ptr<double>(v);
                        double* image_time_ptr = image_time.ptr<double>(v);

                        time_img_ptr[u]++;
                        image_time_ptr[u] = std::max(e.ts.toSec(), image_time_ptr[u]);
                    }

                    // time_img.at<double>(undistorted_e_p.y, undistorted_e_p.x)++;

                    // // LOG(ERROR) << "e.ts.toSec() = " << e.ts.toSec() << std::endl;
                    // double pixel_time = image_time.at<double>(undistorted_e_p.y, undistorted_e_p.x);
                    // image_time.at<double>(undistorted_e_p.y, undistorted_e_p.x) = std::max(e.ts.toSec(), pixel_time);

                    // 更新 event_link
                    event_link[key].push_back(e.ts.toSec());  // 假设 ts 是 ros::Time 类型
                }
                else
                {
                    time_img.at<double>(undistorted_e_p.y, undistorted_e_p.x)++;
                    // 更新 event_link
                    event_link[key].push_back(e.ts.toSec());  // 假设 ts 是 ros::Time 类型
                }*/
        
                //  --i;
                accumu_count++;
                // LOG(ERROR) << "accumu_count = " << accumu_count << std::endl;
                // LOG(ERROR) << "count_not_in_img = " << count_not_in_img << std::endl;
                // LOG(ERROR) << "rectify_fault = " << rectify_fault << std::endl;

                if(accumu_count >= need_events)
                    break;
            }
            // LOG(ERROR) << "accumu_count = " << accumu_count << std::endl;
            // LOG(ERROR) << "need_events = " << need_events << std::endl;

            // if(accumu_count < need_events)
            //     return -1;

            // double final_event_time = event_buffer[need_events-1].ts.toSec() - event_buffer[0].ts.toSec();
            double final_event_time = event_buffer[start_p].ts.toSec() - event_buffer[start_p - accumu_count + 1].ts.toSec();

            return final_event_time;
    }

    // 压缩时间图像
    bool AccumulateTimeImage(double timestamp) {
        std::chrono::time_point<std::chrono::high_resolution_clock> acc_start_time = std::chrono::high_resolution_clock::now();
        // TimeImage1 = cv::Mat::zeros(sensor_height, sensor_width, CV_64FC1);
        TimeImage1.setTo(0);
        TimeImage1_events.clear();

        // TimeImage1_time = cv::Mat::zeros(sensor_height, sensor_width, CV_64FC1);
        std::chrono::time_point<std::chrono::high_resolution_clock> acc_init_time = std::chrono::high_resolution_clock::now();
        // LOG(ERROR) << "t1_image_count = " << t1_image_count << std::endl;
        double event_dt = AccumuConstractEventsWindows(timestamp, TimeImage1, TimeImage1_events, TimeImage1_time, t1_image_count);
        if(event_dt < 0)
        {
            LOG(ERROR) << "Failed: AccumulateTimeImage" << std::endl;
            return false;
        }

        std::chrono::time_point<std::chrono::high_resolution_clock> acc_accum_time = std::chrono::high_resolution_clock::now();

        // static double delte_duration = 2.5;
        double delte_duration = std::min(2 * event_dt, 2.5);
        // assert(event_buffer.size() >= t1_image_count);
        // LOG(ERROR) << "erase time = " << event_buffer[t1_image_count-1].ts.toSec() << std::endl;
        RemoveOldEvents(std::max(0.0, timestamp - delte_duration));
        LOG(ERROR) << "event_dt = " << event_dt << std::endl;

        std::chrono::time_point<std::chrono::high_resolution_clock> remove_accum_time = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double, std::milli> elapsed;
        elapsed = remove_accum_time - acc_start_time;
        LOG(ERROR) << "Total Time: " << elapsed.count() << std::endl;
        elapsed = acc_init_time - acc_start_time;
        LOG(ERROR) << "Init Time: " << elapsed.count() << std::endl;
        elapsed = acc_accum_time - acc_init_time;
        LOG(ERROR) << "Accumu Time: " << elapsed.count() << std::endl;
        elapsed = remove_accum_time - acc_accum_time;
        LOG(ERROR) << "Remove Time: " << elapsed.count() << std::endl;
        return true;
    }   

/*
    bool CalculateOpFlowPrepointSingleFit()
    {
        std::chrono::time_point<std::chrono::high_resolution_clock> flow_start_time = std::chrono::high_resolution_clock::now();
        // TimeImage1 > 0 要求其中的点必须大于0.5 * radius * radius
        cv::Mat mask = TimeImage1 > 0; // (param.inlier_ratio * param.radius * param.radius);
        std::vector<cv::Point2d> valid_points;
        cv::findNonZero(mask, valid_points);
        // LOG(ERROR) << "valid_points.size = " << valid_points.size() << std::endl;
        if(valid_points.empty())
            return false;
        // std::vector<cv::Point2d> select_points;

        // LOG(ERROR) << "flow_pre_points.size = " << flow_pre_points.size() << std::endl;
        // LOG(ERROR) << "best_inliers.size = " << best_inliers.size() << std::endl;
        int grid_size = 2 * radius + 1;
        std::unordered_map<int, std::unordered_map<int, std::vector<cv::Point2d>>> grid_map = 
                                                        InitgridBased(valid_points, grid_size);

        // int iterator = 0;
        int select_event_points = 25;   // 65
        double ransac_err = 1e5;
        std::vector<cv::Point2d> selected_points;
        selected_points.reserve(select_event_points);
        double pre_error;
        std::vector<cv::Point2d> pre_inliers;
        pre_inliers.reserve(select_event_points);
        std::vector<event_flow_velocity> pre_points;
        pre_points.reserve(select_event_points);
        std::vector<Eigen::Vector3d> points;
        points.reserve(2 * grid_size * grid_size);
        // int neighbor_count;
        Eigen::JacobiSVD<Eigen::MatrixXd> svd;

        double neighbor_need = ratio_inliers * (2 * radius + 1) * (2 * radius + 1);

        std::chrono::time_point<std::chrono::high_resolution_clock> flow_init_time = std::chrono::high_resolution_clock::now();
        // 外循环,迭代计算
        for(int iterator = 0; iterator < 15; iterator ++) // 25 // 15
        {
            selected_points.clear();
            pre_inliers.clear();
            pre_points.clear();
            /*{
            std::random_device rd;
            std::mt19937 gen(rd()); // 随机数生成器
            std::shuffle(valid_points.begin(), valid_points.end(), gen); // 打乱顺序
            selected_points.assign(valid_points.begin(), valid_points.begin() + 65);
            } // * /

            std::chrono::time_point<std::chrono::high_resolution_clock> ransac_init_time = std::chrono::high_resolution_clock::now();            
            
            // gridBasedSampling(grid_map, valid_points, selected_points, 65);
            gridBasedSampling(grid_map, selected_points, select_event_points);

            std::chrono::time_point<std::chrono::high_resolution_clock> ransac_sample_time = std::chrono::high_resolution_clock::now();

            // LOG(ERROR) << "selected_points.size = " << selected_points.size() << std::endl;
            // last pre error
            pre_error = 0;
            
            // 内循环,遍历局部点
            for(auto& p: selected_points)
            {
                // LOG(ERROR) << "iterator = " << iterator << std::endl;
                // if(iterator++ > 25)
                //     break;
                // std::chrono::time_point<std::chrono::high_resolution_clock> ransac_iter_start_time = std::chrono::high_resolution_clock::now();
                // if(p.x < 2 * radius || p.x > sensor_width - 2 * radius - 1 
                //     || p.y < 2 * radius || p.y > sensor_height - 2 * radius - 1)
                //     continue;

                std::vector<Eigen::Vector3d> points;
                points.reserve(2 * grid_size * grid_size);
                // points.clear();
                int neighbor_count = 0;
                double p_x, p_y;
                std::pair<int, int> key;
                std::chrono::time_point<std::chrono::high_resolution_clock> ransac_iter_init_time = std::chrono::high_resolution_clock::now();
                // 遍历 TimeImage1_events，查找 (p.x ± radius, p.y ± radius) 范围内的点
                for (int dx = -radius; dx <= radius; ++dx) {
                    for (int dy = -radius; dy <= radius; ++dy) {
                        p_x = p.x + dx;
                        p_y = p.y + dy;
                        key = {p_x, p_y};
                        if (TimeImage1_events.find(key) != TimeImage1_events.end()) {
                            // LOG(ERROR) << "key = " << key.first << " " << key.second << std::endl;

                            neighbor_count ++; // TimeImage1_events[key].size();
                            for(auto& t: TimeImage1_events[key])
                                points.emplace_back(Eigen::Vector3d(p_x, p_y, t));
                            // continue;
                        }
                    }
                }

                // LOG(ERROR) << "param.inlier_ratio = " << param.inlier_ratio << std::endl;
                // LOG(ERROR) << "param.radius = " << param.radius << std::endl;

                if(neighbor_count < neighbor_need)
                    continue;

                Eigen::Vector4d normal;

                // LOG(ERROR) << "neighbor_count = " << neighbor_count << " need = " << ratio_inliers * (2 * radius + 1) * (2 * radius + 1) << std::endl;
                // std::chrono::time_point<std::chrono::high_resolution_clock> ransac_iter_count_time = std::chrono::high_resolution_clock::now();
                // LOG(ERROR) << "points.size = " << points.size() << std::endl;
                // 直接构建矩阵,去中心话
                // Eigen::MatrixXd A(points.size(), 4);
                Eigen::MatrixXd A;
                // (15, 4);
                {
                    std::vector<Eigen::Vector3d> random_points;
                    // 随机选择 15 个点拟合平面
                    std::random_device rd;
                    std::mt19937 gen(rd());
                    std::uniform_int_distribution<> dis(0, points.size() - 1);
                    for (size_t i = 0; i < std::min(15, static_cast<int>(points.size())); ++i) {
                        random_points.push_back(points[dis(gen)]);
                    }
                    
                    // LOG(ERROR) << "random_points.size = " << random_points.size() << std::endl;
                    A.resize(random_points.size(), 4);
                    for (size_t i = 0; i < random_points.size(); ++i) {
                        A(i, 0) = random_points[i].x();
                        A(i, 1) = random_points[i].y();
                        A(i, 2) = random_points[i].z();
                        A(i, 3) = 1.0;
                    }

                    // for (size_t i = 0; i < points.size(); ++i) {
                    //     A(i, 0) = points[i].x();
                    //     A(i, 1) = points[i].y();
                    //     A(i, 2) = points[i].z();
                    //     A(i, 3) = 1.0;
                    // }

                    // LOG(ERROR) << "A = " << A << std::endl;
                    Eigen::Vector4d A_mean = A.colwise().mean();  // 计算每列的均值
                    A_mean(3) = 0;
                    A.rowwise() -= A_mean.transpose();
                    // LOG(ERROR) << "A_mean = " << A_mean << std::endl;
                    // LOG(ERROR) << "A decenter = " << A << std::endl;
                    // LOG(ERROR) << "fx = " << cam_info.K[0] << ", fy = " << cam_info.K[4] << std::endl;
                    A.col(0).array() /= cam_info.K[0];
                    A.col(1).array() /= cam_info.K[4];
                    // LOG(ERROR) << "A coordinate = " << A << std::endl;
                }
                std::chrono::time_point<std::chrono::high_resolution_clock> ransac_iter_decenterize_time = std::chrono::high_resolution_clock::now();
                std::chrono::time_point<std::chrono::high_resolution_clock> ransac_A, ransac_B;
                {   
                    // 计算 SVD 拟合 Ax + By + Ct + D = 0
                    // Eigen::JacobiSVD<Eigen::MatrixXd> svd(A, Eigen::ComputeThinV | Eigen::ComputeThinU);
                    // normal = svd.matrixV().col(3); // 选择最小奇异值对应的列
                    svd.compute(A, Eigen::ComputeThinV);
                    normal = svd.matrixV().col(3);

                    // Eigen::Vector4d normal = svd.matrixV().rightCols<1>(); // 最后一列

                    ransac_A = std::chrono::high_resolution_clock::now();

                    // LOG(ERROR) << "normal = " << normal.transpose() << std::endl;

                    // 计算光流 u, v
                    double A_ = normal(0);
                    double B_ = normal(1);
                    double C_ = normal(2);

                    // Eigen::Vector2d flow;
                    // cv::Point2d flow_c;
                    event_flow_velocity flow;
                    if (std::abs(C_) > 1e-6) { // 避免除零错误
                        // flow(0) = -C_ / A_;         // x
                        // flow(1) = -C_ / B_;         // y
                        flow.x = -C_ / A_;         // x
                        flow.y = -C_ / B_;         // y
                        // LOG(ERROR) << "Optical Flow at (" << p.x << "," << p.y << "): u=" << flow.x << ", v=" << flow.y << std::endl;
                        // flow /= (max_t - min_t);
                        // LOG(ERROR) << "Actual Optical Flow at (" << p.x << "," << p.y << "): u=" << flow(0) << ", v=" << flow(1) << std::endl;
                        pre_points.emplace_back(flow);
                        pre_inliers.emplace_back(p);
                    }
                    else
                    {
                        // LOG(ERROR) << "C_ is too small" << std::endl;
                        continue;
                    } 

                    ransac_B = std::chrono::high_resolution_clock::now();
                } 
                

                {
                    std::chrono::duration<double, std::milli> elapsed;
                    elapsed = ransac_A - ransac_iter_decenterize_time;
                    LOG(ERROR) << "Ransac2 SVD Time: " << elapsed.count() << std::endl;
                    elapsed = ransac_B - ransac_A;
                    LOG(ERROR) << "Ransac2 flow calculate Time: " << elapsed.count() << std::endl;
                }

                // pre_error += (A * normal).array().sum();
                // Eigen::VectorXd residuals = A * normal;
                // pre_error += residuals.squaredNorm(); // 或者 residuals.array().abs().sum();

                // pre_error += (A * normal).squaredNorm();
                // LOG(ERROR) << "update error = " << pre_error << std::endl;
                // LOG(ERROR) << "update error = " << (A * normal).array().sum() << std::endl;

                /*std::chrono::time_point<std::chrono::high_resolution_clock> ransac_iter_polyfit_time = std::chrono::high_resolution_clock::now();
                {
                    std::chrono::duration<double, std::milli> elapsed;
                    elapsed = ransac_iter_polyfit_time - ransac_iter_start_time;
                    LOG(ERROR) << "Ransac2 Total Time: " << elapsed.count() << std::endl;
                    elapsed = ransac_iter_init_time - ransac_iter_start_time;
                    LOG(ERROR) << "Ransac2 init: " << elapsed.count() << std::endl;
                    elapsed = ransac_iter_count_time - ransac_iter_init_time;
                    LOG(ERROR) << "Ransac2 count: " << elapsed.count() << std::endl;
                    elapsed = ransac_iter_decenterize_time - ransac_iter_count_time;
                    LOG(ERROR) << "Ransac2 decenterize: " << elapsed.count() << std::endl;
                    elapsed = ransac_iter_polyfit_time - ransac_iter_decenterize_time;
                    LOG(ERROR) << "Ransac2 Polyfit: " << elapsed.count() << std::endl;
                }
                */

            // }
            
            
            /*
            #pragma omp parallel
            {
                // 线程局部变量
                std::vector<event_flow_velocity> local_pre_points;
                std::vector<cv::Point> local_pre_inliers;
                std::mt19937 gen(omp_get_thread_num()); // 每个线程独立随机数生成器

                #pragma omp for nowait
                for (size_t idx = 0; idx < selected_points.size(); ++idx)
                {
                    auto& p = selected_points[idx];

                    std::vector<Eigen::Vector3d> points;
                    int neighbor_count = 0;
                    double p_x, p_y;
                    std::pair<int, int> key;

                    for (int dx = -radius; dx <= radius; ++dx) {
                        for (int dy = -radius; dy <= radius; ++dy) {
                            p_x = p.x + dx;
                            p_y = p.y + dy;
                            key = {p_x, p_y};
                            if (TimeImage1_events.find(key) != TimeImage1_events.end()) {
                                neighbor_count++;
                                for (auto& t : TimeImage1_events.at(key)) {
                                    points.emplace_back(Eigen::Vector3d(p_x, p_y, t));
                                }
                            }
                        }
                    }

                    if (neighbor_count < neighbor_need)
                        continue;

                    // 随机选择最多15个点
                    LOG(ERROR) << "points.size() = " << points.size() << std::endl;
                    std::vector<Eigen::Vector3d> random_points;
                    std::uniform_int_distribution<> dis(0, points.size() - 1);
                    for (size_t i = 0; i < std::min(15, static_cast<int>(points.size())); ++i) {
                        random_points.push_back(points[dis(gen)]);
                    }

                    Eigen::MatrixXd A(random_points.size(), 4);
                    for (size_t i = 0; i < random_points.size(); ++i) {
                        A(i, 0) = random_points[i].x();
                        A(i, 1) = random_points[i].y();
                        A(i, 2) = random_points[i].z();
                        A(i, 3) = 1.0;
                    }

                    Eigen::Vector4d A_mean = A.colwise().mean();
                    A_mean(3) = 0;
                    A.rowwise() -= A_mean.transpose();
                    A.col(0).array() /= cam_info.K[0];
                    A.col(1).array() /= cam_info.K[4];

                    Eigen::JacobiSVD<Eigen::MatrixXd> svd_local(A, Eigen::ComputeThinV);
                    Eigen::Vector4d normal = svd_local.matrixV().col(3);
                    double A_ = normal(0), B_ = normal(1), C_ = normal(2);

                    if (std::abs(C_) > 1e-6) {
                        event_flow_velocity flow;
                        flow.x = -C_ / A_;
                        flow.y = -C_ / B_;
                        local_pre_points.emplace_back(flow);
                        local_pre_inliers.emplace_back(p);
                    }
                }

                #pragma omp critical
                {
                    pre_points.insert(pre_points.end(), local_pre_points.begin(), local_pre_points.end());
                    pre_inliers.insert(pre_inliers.end(), local_pre_inliers.begin(), local_pre_inliers.end());
                }
            }
            */

            /*
            LOG(ERROR) << "opm 1 " << std::endl;
            // OpenMP并行计算
            #pragma omp parallel for schedule(dynamic)
            // 内循环,遍历局部点
            for (int i = 0; i < selected_points.size(); ++i)
            {
                auto& p = selected_points[i];
                std::vector<Eigen::Vector3d> points;
                points.reserve(2 * grid_size * grid_size);
                // points.clear();
                int neighbor_count = 0;
                double p_x, p_y;
                std::pair<int, int> key;

                // 遍历 TimeImage1_events，查找 (p.x ± radius, p.y ± radius) 范围内的点
                for (int dx = -radius; dx <= radius; ++dx) {
                    for (int dy = -radius; dy <= radius; ++dy) {
                        p_x = p.x + dx;
                        p_y = p.y + dy;
                        key = {p_x, p_y};
                        if (TimeImage1_events.find(key) != TimeImage1_events.end()) {
                            neighbor_count ++; // TimeImage1_events[key].size();
                            for(auto& t: TimeImage1_events[key])
                                points.emplace_back(Eigen::Vector3d(p_x, p_y, t));
                        }
                    }
                }

                if(neighbor_count < neighbor_need)
                    continue;

                Eigen::Vector4d normal;

                // 直接构建矩阵,去中心化
                Eigen::MatrixXd A;       // (15, 4);
                {
                    LOG(ERROR) << "points.size = " << points.size() << std::endl;
                    std::vector<Eigen::Vector3d> random_points;
                    // 随机选择 15 个点拟合平面
                    std::random_device rd;
                    std::mt19937 gen(rd());
                    std::uniform_int_distribution<> dis(0, points.size() - 1);
                    for (size_t i = 0; i < std::min(15, static_cast<int>(points.size())); ++i) {
                        random_points.push_back(points[dis(gen)]);
                    }
                    
                    LOG(ERROR) << "random_points.size = " << random_points.size() << std::endl;
                    A.resize(random_points.size(), 4);
                    for (size_t i = 0; i < random_points.size(); ++i) {
                        A(i, 0) = random_points[i].x();
                        A(i, 1) = random_points[i].y();
                        A(i, 2) = random_points[i].z();
                        A(i, 3) = 1.0;
                    }

                    // LOG(ERROR) << "A = " << A << std::endl;
                    Eigen::Vector4d A_mean = A.colwise().mean();  // 计算每列的均值
                    A_mean(3) = 0;
                    A.rowwise() -= A_mean.transpose();
                    // LOG(ERROR) << "A_mean = " << A_mean << std::endl;
                    // LOG(ERROR) << "A decenter = " << A << std::endl;
                    // LOG(ERROR) << "fx = " << cam_info.K[0] << ", fy = " << cam_info.K[4] << std::endl;
                    A.col(0).array() /= cam_info.K[0];
                    A.col(1).array() /= cam_info.K[4];
                    // LOG(ERROR) << "A coordinate = " << A << std::endl;
                }

                {   
                    // 计算 SVD 拟合 Ax + By + Ct + D = 0
                    svd.compute(A, Eigen::ComputeThinV);
                    normal = svd.matrixV().col(3);

                    // 计算光流 u, v
                    double A_ = normal(0);
                    double B_ = normal(1);
                    double C_ = normal(2);

                    // Eigen::Vector2d flow;
                    // cv::Point2d flow_c;
                    event_flow_velocity flow;
                     
                    if (std::abs(C_) > 1e-6) { // 避免除零错误
                        flow.x = -C_ / A_;         // x
                        flow.y = -C_ / B_;         // y
                        // 关键：多个线程写共享容器时需要加锁
                        #pragma omp critical
                        {
                            pre_points.emplace_back(flow);
                            pre_inliers.emplace_back(p);
                            pre_error += (A * normal).squaredNorm();
                        }
                    }
                    else
                    {
                        // LOG(ERROR) << "C_ is too small" << std::endl;
                        continue;
                    }
                } 
            }
            LOG(ERROR) << "opm 2 " << std::endl;
            // * /

            std::chrono::time_point<std::chrono::high_resolution_clock> ransac_end_time = std::chrono::high_resolution_clock::now();

            if(pre_error < ransac_err)
            {
                ransac_err = pre_error;
                best_inliers.clear();
                flow_pre_points.clear();

                best_inliers = pre_inliers;
                flow_pre_points = pre_points;

                LOG(ERROR) << "OpFlow Update: iter = " << iterator << " error = " << ransac_err << " select_poins = " << best_inliers.size() << std::endl;
            }

            std::chrono::time_point<std::chrono::high_resolution_clock> flow_ransac_iter_time = std::chrono::high_resolution_clock::now();
            {
                std::chrono::duration<double, std::milli> elapsed;
                elapsed = flow_ransac_iter_time - ransac_init_time;
                LOG(ERROR) << "Ransac Total Time: " << elapsed.count() << std::endl;
                elapsed = ransac_sample_time - ransac_init_time;
                LOG(ERROR) << "Ransac Sample: " << elapsed.count() << std::endl;
                elapsed = ransac_end_time - ransac_sample_time;
                LOG(ERROR) << "Ransac Run: " << elapsed.count() << std::endl;
                elapsed = flow_ransac_iter_time - ransac_end_time;
                LOG(ERROR) << "Ransac Select: " << elapsed.count() << std::endl;
            }

        }

        
        std::chrono::time_point<std::chrono::high_resolution_clock> flow_ransac_time = std::chrono::high_resolution_clock::now();


        // LOG(ERROR) << "INFO All Result: " << std::endl;
        LOG(ERROR) << "flow_pre_points.size() = " << flow_pre_points.size() << std::endl;
        if(flow_pre_points.size() < 1)
            return false;
        double norm_mean = 0.0;
        double norm_std = 0.0;
        double rad_mean = 0.0;
        double rad_std = 0.0;
        Eigen::MatrixXd flow_matrix(flow_pre_points.size(), 2);
        std::vector<std::pair<double, double>> flow_collect;
        flow_collect.reserve(flow_pre_points.size());
        for(int i = 0; i < flow_pre_points.size(); i++)
        {
            double norm_i = flow_pre_points[i].norm();
            double rad_i = flow_pre_points[i].rad();
            flow_matrix.row(i) << norm_i, rad_i;
            flow_collect.push_back(std::pair<double, double>(norm_i, rad_i));
            // LOG(ERROR) << "flow_collect = " << flow_collect[i].first << ", " << flow_collect[i].second << std::endl;
        }
        // LOG(ERROR) << "M_norm_rad = " << flow_matrix << std::endl;

        std::chrono::time_point<std::chrono::high_resolution_clock> filter_start_time = std::chrono::high_resolution_clock::now();

        // 2 * \sigma = 0.9544
        int skip_norm = 0;
        int skip_rad = 0;
        double num_sigma = 2.0; // 0.5
        int interation = 3;
        for(int iter = 1; iter < interation; iter++)
        {
            // 计算第一列 (norm) 的均值和标准差
            // auto [mean1, stddev1] = computeMeanAndStdDev(flow_matrix.col(0));
            // LOG(ERROR) << "norm means = " << mean1 << " std = " << stddev1 << std::endl;

            // 计算第二列 (rad) 的均值和标准差
            // auto [mean2, stddev2] = computeMeanAndStdDev(flow_matrix.col(1));
            // LOG(ERROR) << "rad means = " << mean2 << " std = " << stddev2 << std::endl;

            std::vector<std::pair<double, double>> mean_std_collect = computeMeanAndStdDev(flow_collect);
            // LOG(ERROR) << "norm means = " << mean_std_collect[0].first << " std = " << mean_std_collect[0].second << std::endl;
            // LOG(ERROR) << "rad means = " << mean_std_collect[1].first << " std = " << mean_std_collect[1].second << std::endl;

            for (int i = flow_pre_points.size() - 1; i >= 0; --i)
            {
                if (std::abs(flow_matrix(i, 0) - mean_std_collect[0].first) > num_sigma * mean_std_collect[0].second)
                {
                    best_inliers.erase(best_inliers.begin() + i);
                    flow_pre_points.erase(flow_pre_points.begin() + i);
                    flow_collect.erase(flow_collect.begin() + i);
                    skip_norm++;
                    continue;
                }
                // if (std::abs(flow_matrix(i, 1) - mean_std_collect[1].first) > 2 * num_sigma * mean_std_collect[1].second)
                // {
                //     best_inliers.erase(best_inliers.begin() + i);
                //     flow_pre_points.erase(flow_pre_points.begin() + i);
                //     flow_collect.erase(flow_collect.begin() + i);
                //     skip_rad++;
                //     continue;
                // }
            }
        }
        LOG(ERROR) << "iteration" << interation << ", "<< num_sigma 
                    << " * σ: skip_norm = " << skip_norm << ", skip_rad = " << skip_rad;
        LOG(ERROR) << "final size = " << flow_collect.size() << std::endl;

        // std::chrono::time_point<std::chrono::high_resolution_clock> filter_end_time = std::chrono::high_resolution_clock::now();

        /*
        Eigen::MatrixXd final_flow_matrix(flow_collect.size(), 2);
        for(int i = 0; i < flow_collect.size(); i++)
        {
            double norm_i = flow_collect[i].first;
            double rad_i = flow_collect[i].second;
            final_flow_matrix.row(i) << norm_i, rad_i;

            LOG(ERROR) << "flow_collect[i] = " << flow_collect[i].first << ", " << flow_collect[i].second << std::endl;
        }
        LOG(ERROR) << "final_flow_matrix = " << final_flow_matrix << std::endl;
        // * /

        std::chrono::time_point<std::chrono::high_resolution_clock> flow_end_time = std::chrono::high_resolution_clock::now();


        {
            std::chrono::duration<double, std::milli> elapsed;
            elapsed = flow_end_time - flow_start_time;
            LOG(ERROR) << "Flow Total Time: " << elapsed.count() << std::endl;
            elapsed = flow_init_time - flow_start_time;
            LOG(ERROR) << "Flow Init: " << elapsed.count() << std::endl;
            elapsed = flow_ransac_time - flow_init_time;
            LOG(ERROR) << "Flow Init2: " << elapsed.count() << std::endl;
            elapsed = filter_start_time - flow_ransac_time;
            LOG(ERROR) << "Flow Ransac: " << elapsed.count() << std::endl;
            // elapsed = filter_end_time - filter_start_time;
            // LOG(ERROR) << "Flow Filter: " << elapsed.count() << std::endl;
            elapsed = flow_end_time - filter_start_time;
            LOG(ERROR) << "Flow Filter: " << elapsed.count() << std::endl;
        }

        if(flow_collect.size() < 6)
        {
            LOG(ERROR) << "not enough flow_collect" << std::endl;
            return false;
        }
        // LOG(ERROR) << "2 * \sigma: skip_norm = " << skip_norm << ", skip_rad = " << skip_rad;


        return true;
    }
*/
/*
    bool CalculateOpFlowPrepointSingleFit()
    {
        std::chrono::time_point<std::chrono::high_resolution_clock> flow_start_time = std::chrono::high_resolution_clock::now();
        // TimeImage1 > 0 要求其中的点必须大于0.5 * radius * radius
        cv::Mat mask = TimeImage1 > 0; // (param.inlier_ratio * param.radius * param.radius);
        std::vector<cv::Point2d> valid_points;
        cv::findNonZero(mask, valid_points);
        // LOG(ERROR) << "valid_points.size = " << valid_points.size() << std::endl;
        if(valid_points.empty())
        {
            LOG(ERROR) << "not valid points" << std::endl;
            return false;
        }
        std::vector<cv::Point2d> select_points;

        std::random_device rd;
        std::mt19937 gen(rd());
        // 复制一份原始数据，用于 shuffle
        std::vector<cv::Point2d> shuffled_points = valid_points;

        // 打乱顺序
        std::shuffle(shuffled_points.begin(), shuffled_points.end(), gen);

        // 选择前 40 个（如果不够 40 个就全选）
        size_t count = std::min(size_t(40), shuffled_points.size());
        select_points.assign(shuffled_points.begin(), shuffled_points.begin() + count);

        // 全局最小二乘法
        {
            std::chrono::time_point<std::chrono::high_resolution_clock> lsq_start_time = std::chrono::high_resolution_clock::now();  
            int valid_points_size = select_points.size();
            LOG(ERROR) << "valid_points.size() = " << valid_points_size << std::endl;
            int grid_size = 2 * radius + 1;
            double neighbor_need = ratio_inliers * (2 * radius + 1) * (2 * radius + 1);
            std::vector<cv::Point2d> pre_inliers;
            pre_inliers.reserve(valid_points_size);
            std::vector<event_flow_velocity> pre_points;
            pre_points.reserve(valid_points_size);
            Eigen::JacobiSVD<Eigen::MatrixXd> svd;

            std::chrono::time_point<std::chrono::high_resolution_clock> lsq_iter_init_time = std::chrono::high_resolution_clock::now();
            for(auto& p: select_points)
            {
                // LOG(ERROR) << "iterator = " << iterator << std::endl;
                // if(iterator++ > 25)
                //     break;
                // std::chrono::time_point<std::chrono::high_resolution_clock> ransac_iter_start_time = std::chrono::high_resolution_clock::now();
                // if(p.x < 2 * radius || p.x > sensor_width - 2 * radius - 1 
                //     || p.y < 2 * radius || p.y > sensor_height - 2 * radius - 1)
                //     continue;

                std::vector<Eigen::Vector3d> points;
                points.reserve(2 * grid_size * grid_size);
                // points.clear();
                int neighbor_count = 0;
                double p_x, p_y;
                std::pair<int, int> key;    
                // 遍历 TimeImage1_events，查找 (p.x ± radius, p.y ± radius) 范围内的点
                for (int dx = -radius; dx <= radius; ++dx) {
                    for (int dy = -radius; dy <= radius; ++dy) {
                        p_x = p.x + dx;
                        p_y = p.y + dy;
                        key = {p_x, p_y};
                        if (TimeImage1_events.find(key) != TimeImage1_events.end()) {
                            // LOG(ERROR) << "key = " << key.first << " " << key.second << std::endl;

                            neighbor_count ++; // TimeImage1_events[key].size();
                            for(auto& t: TimeImage1_events[key])
                                points.emplace_back(Eigen::Vector3d(p_x, p_y, t));
                            // continue;
                        }
                    }
                }

                // LOG(ERROR) << "param.inlier_ratio = " << param.inlier_ratio << std::endl;
                // LOG(ERROR) << "param.radius = " << param.radius << std::endl;

                if(neighbor_count < neighbor_need)
                    continue;

                Eigen::Vector4d normal;

                // LOG(ERROR) << "neighbor_count = " << neighbor_count << " need = " << ratio_inliers * (2 * radius + 1) * (2 * radius + 1) << std::endl;
                // std::chrono::time_point<std::chrono::high_resolution_clock> ransac_iter_count_time = std::chrono::high_resolution_clock::now();
                // LOG(ERROR) << "points.size = " << points.size() << std::endl;
                // 直接构建矩阵,去中心话
                // Eigen::MatrixXd A(points.size(), 4);
                Eigen::MatrixXd A;
                // (15, 4);
                {
                    std::vector<Eigen::Vector3d> random_points;
                    // 随机选择 15 个点拟合平面
                    std::random_device rd;
                    std::mt19937 gen(rd());
                    std::uniform_int_distribution<> dis(0, points.size() - 1);
                    for (size_t i = 0; i < std::min(15, static_cast<int>(points.size())); ++i) {
                        random_points.push_back(points[dis(gen)]);
                    }
                    
                    // LOG(ERROR) << "random_points.size = " << random_points.size() << std::endl;
                    A.resize(random_points.size(), 4);
                    for (size_t i = 0; i < random_points.size(); ++i) {
                        A(i, 0) = random_points[i].x();
                        A(i, 1) = random_points[i].y();
                        A(i, 2) = random_points[i].z();
                        A(i, 3) = 1.0;
                    }

                    // for (size_t i = 0; i < points.size(); ++i) {
                    //     A(i, 0) = points[i].x();
                    //     A(i, 1) = points[i].y();
                    //     A(i, 2) = points[i].z();
                    //     A(i, 3) = 1.0;
                    // }

                    // LOG(ERROR) << "A = " << A << std::endl;
                    Eigen::Vector4d A_mean = A.colwise().mean();  // 计算每列的均值
                    A_mean(3) = 0;
                    A.rowwise() -= A_mean.transpose();
                    // LOG(ERROR) << "A_mean = " << A_mean << std::endl;
                    // LOG(ERROR) << "A decenter = " << A << std::endl;
                    // LOG(ERROR) << "fx = " << cam_info.K[0] << ", fy = " << cam_info.K[4] << std::endl;
                    A.col(0).array() /= cam_info.K[0];
                    A.col(1).array() /= cam_info.K[4];
                    // LOG(ERROR) << "A coordinate = " << A << std::endl;
                }
                std::chrono::time_point<std::chrono::high_resolution_clock> ransac_iter_decenterize_time = std::chrono::high_resolution_clock::now();
                std::chrono::time_point<std::chrono::high_resolution_clock> ransac_A, ransac_B;
                {   
                    // 计算 SVD 拟合 Ax + By + Ct + D = 0
                    // Eigen::JacobiSVD<Eigen::MatrixXd> svd(A, Eigen::ComputeThinV | Eigen::ComputeThinU);
                    // normal = svd.matrixV().col(3); // 选择最小奇异值对应的列
                    svd.compute(A, Eigen::ComputeThinV);
                    normal = svd.matrixV().col(3);

                    // Eigen::Vector4d normal = svd.matrixV().rightCols<1>(); // 最后一列

                    ransac_A = std::chrono::high_resolution_clock::now();

                    // LOG(ERROR) << "normal = " << normal.transpose() << std::endl;

                    // 计算光流 u, v
                    double A_ = normal(0);
                    double B_ = normal(1);
                    double C_ = normal(2);

                    // Eigen::Vector2d flow;
                    // cv::Point2d flow_c;
                    event_flow_velocity flow;
                    if (std::abs(C_) > 1e-6) { // 避免除零错误
                        // flow(0) = -C_ / A_;         // x
                        // flow(1) = -C_ / B_;         // y
                        flow.x = -C_ / A_;         // x
                        flow.y = -C_ / B_;         // y
                        // LOG(ERROR) << "Optical Flow at (" << p.x << "," << p.y << "): u=" << flow.x << ", v=" << flow.y << std::endl;
                        // flow /= (max_t - min_t);
                        // LOG(ERROR) << "Actual Optical Flow at (" << p.x << "," << p.y << "): u=" << flow(0) << ", v=" << flow(1) << std::endl;
                        pre_points.emplace_back(flow);
                        pre_inliers.emplace_back(p);
                    }
                    else
                    {
                        // LOG(ERROR) << "C_ is too small" << std::endl;
                        continue;
                    } 

                    ransac_B = std::chrono::high_resolution_clock::now();
                } 
                

                {
                    std::chrono::duration<double, std::milli> elapsed;
                    elapsed = ransac_A - ransac_iter_decenterize_time;
                    LOG(ERROR) << "Ransac2 SVD Time: " << elapsed.count() << std::endl;
                    elapsed = ransac_B - ransac_A;
                    LOG(ERROR) << "Ransac2 flow calculate Time: " << elapsed.count() << std::endl;
                }
            }


            best_inliers.clear();
            flow_pre_points.clear();

            best_inliers = pre_inliers;
            flow_pre_points = pre_points;
            std::chrono::time_point<std::chrono::high_resolution_clock> lsq_end_time = std::chrono::high_resolution_clock::now();

            {
                std::chrono::duration<double, std::milli> elapsed;
                elapsed = lsq_end_time - lsq_start_time;
                LOG(ERROR) << "LSQ Total Time: " << elapsed.count() << std::endl;
                elapsed = lsq_iter_init_time - lsq_start_time;
                LOG(ERROR) << "LSQ Init: " << elapsed.count() << std::endl;
                elapsed = lsq_end_time - lsq_iter_init_time;
                LOG(ERROR) << "LSQ Solve: " << elapsed.count() << std::endl;
            }

        }



        std::chrono::time_point<std::chrono::high_resolution_clock> flow_init_time = std::chrono::high_resolution_clock::now();

        /*
        // LOG(ERROR) << "flow_pre_points.size = " << flow_pre_points.size() << std::endl;
        // LOG(ERROR) << "best_inliers.size = " << best_inliers.size() << std::endl;
        int grid_size = 2 * radius + 1;
        std::unordered_map<int, std::unordered_map<int, std::vector<cv::Point2d>>> grid_map = 
                                                        InitgridBased(valid_points, grid_size);
        // int iterator = 0;
        int select_event_points = 25;   // 65
        double ransac_err = 1e5;
        std::vector<cv::Point2d> selected_points;
        selected_points.reserve(select_event_points);
        double pre_error;
        std::vector<cv::Point2d> pre_inliers;
        pre_inliers.reserve(select_event_points);
        std::vector<event_flow_velocity> pre_points;
        pre_points.reserve(select_event_points);
        std::vector<Eigen::Vector3d> points;
        points.reserve(2 * grid_size * grid_size);
        // int neighbor_count;
        Eigen::JacobiSVD<Eigen::MatrixXd> svd;

        double neighbor_need = ratio_inliers * (2 * radius + 1) * (2 * radius + 1);

        std::chrono::time_point<std::chrono::high_resolution_clock> flow_init_time = std::chrono::high_resolution_clock::now();
 
        // 外循环,迭代计算
        for(int iterator = 0; iterator < 15; iterator ++) // 25 // 15
        {
            selected_points.clear();
            pre_inliers.clear();
            pre_points.clear();
            /*{
            std::random_device rd;
            std::mt19937 gen(rd()); // 随机数生成器
            std::shuffle(valid_points.begin(), valid_points.end(), gen); // 打乱顺序
            selected_points.assign(valid_points.begin(), valid_points.begin() + 65);
            }

            std::chrono::time_point<std::chrono::high_resolution_clock> ransac_init_time = std::chrono::high_resolution_clock::now();            
            
            // gridBasedSampling(grid_map, valid_points, selected_points, 65);
            gridBasedSampling(grid_map, selected_points, select_event_points);

            std::chrono::time_point<std::chrono::high_resolution_clock> ransac_sample_time = std::chrono::high_resolution_clock::now();

            // LOG(ERROR) << "selected_points.size = " << selected_points.size() << std::endl;
            // last pre error
            pre_error = 0;
            
            // 内循环,遍历局部点
            for(auto& p: selected_points)
            {
                // LOG(ERROR) << "iterator = " << iterator << std::endl;
                // if(iterator++ > 25)
                //     break;
                // std::chrono::time_point<std::chrono::high_resolution_clock> ransac_iter_start_time = std::chrono::high_resolution_clock::now();
                // if(p.x < 2 * radius || p.x > sensor_width - 2 * radius - 1 
                //     || p.y < 2 * radius || p.y > sensor_height - 2 * radius - 1)
                //     continue;

                std::vector<Eigen::Vector3d> points;
                points.reserve(2 * grid_size * grid_size);
                // points.clear();
                int neighbor_count = 0;
                double p_x, p_y;
                std::pair<int, int> key;
                std::chrono::time_point<std::chrono::high_resolution_clock> ransac_iter_init_time = std::chrono::high_resolution_clock::now();
                // 遍历 TimeImage1_events，查找 (p.x ± radius, p.y ± radius) 范围内的点
                for (int dx = -radius; dx <= radius; ++dx) {
                    for (int dy = -radius; dy <= radius; ++dy) {
                        p_x = p.x + dx;
                        p_y = p.y + dy;
                        key = {p_x, p_y};
                        if (TimeImage1_events.find(key) != TimeImage1_events.end()) {
                            // LOG(ERROR) << "key = " << key.first << " " << key.second << std::endl;

                            neighbor_count ++; // TimeImage1_events[key].size();
                            for(auto& t: TimeImage1_events[key])
                                points.emplace_back(Eigen::Vector3d(p_x, p_y, t));
                            // continue;
                        }
                    }
                }

                // LOG(ERROR) << "param.inlier_ratio = " << param.inlier_ratio << std::endl;
                // LOG(ERROR) << "param.radius = " << param.radius << std::endl;

                if(neighbor_count < neighbor_need)
                    continue;

                Eigen::Vector4d normal;

                // LOG(ERROR) << "neighbor_count = " << neighbor_count << " need = " << ratio_inliers * (2 * radius + 1) * (2 * radius + 1) << std::endl;
                // std::chrono::time_point<std::chrono::high_resolution_clock> ransac_iter_count_time = std::chrono::high_resolution_clock::now();
                // LOG(ERROR) << "points.size = " << points.size() << std::endl;
                // 直接构建矩阵,去中心话
                // Eigen::MatrixXd A(points.size(), 4);
                Eigen::MatrixXd A;
                // (15, 4);
                {
                    std::vector<Eigen::Vector3d> random_points;
                    // 随机选择 15 个点拟合平面
                    std::random_device rd;
                    std::mt19937 gen(rd());
                    std::uniform_int_distribution<> dis(0, points.size() - 1);
                    for (size_t i = 0; i < std::min(15, static_cast<int>(points.size())); ++i) {
                        random_points.push_back(points[dis(gen)]);
                    }
                    
                    // LOG(ERROR) << "random_points.size = " << random_points.size() << std::endl;
                    A.resize(random_points.size(), 4);
                    for (size_t i = 0; i < random_points.size(); ++i) {
                        A(i, 0) = random_points[i].x();
                        A(i, 1) = random_points[i].y();
                        A(i, 2) = random_points[i].z();
                        A(i, 3) = 1.0;
                    }

                    // for (size_t i = 0; i < points.size(); ++i) {
                    //     A(i, 0) = points[i].x();
                    //     A(i, 1) = points[i].y();
                    //     A(i, 2) = points[i].z();
                    //     A(i, 3) = 1.0;
                    // }

                    // LOG(ERROR) << "A = " << A << std::endl;
                    Eigen::Vector4d A_mean = A.colwise().mean();  // 计算每列的均值
                    A_mean(3) = 0;
                    A.rowwise() -= A_mean.transpose();
                    // LOG(ERROR) << "A_mean = " << A_mean << std::endl;
                    // LOG(ERROR) << "A decenter = " << A << std::endl;
                    // LOG(ERROR) << "fx = " << cam_info.K[0] << ", fy = " << cam_info.K[4] << std::endl;
                    A.col(0).array() /= cam_info.K[0];
                    A.col(1).array() /= cam_info.K[4];
                    // LOG(ERROR) << "A coordinate = " << A << std::endl;
                }
                std::chrono::time_point<std::chrono::high_resolution_clock> ransac_iter_decenterize_time = std::chrono::high_resolution_clock::now();
                std::chrono::time_point<std::chrono::high_resolution_clock> ransac_A, ransac_B;
                {   
                    // 计算 SVD 拟合 Ax + By + Ct + D = 0
                    // Eigen::JacobiSVD<Eigen::MatrixXd> svd(A, Eigen::ComputeThinV | Eigen::ComputeThinU);
                    // normal = svd.matrixV().col(3); // 选择最小奇异值对应的列
                    svd.compute(A, Eigen::ComputeThinV);
                    normal = svd.matrixV().col(3);

                    // Eigen::Vector4d normal = svd.matrixV().rightCols<1>(); // 最后一列

                    ransac_A = std::chrono::high_resolution_clock::now();

                    // LOG(ERROR) << "normal = " << normal.transpose() << std::endl;

                    // 计算光流 u, v
                    double A_ = normal(0);
                    double B_ = normal(1);
                    double C_ = normal(2);

                    // Eigen::Vector2d flow;
                    // cv::Point2d flow_c;
                    event_flow_velocity flow;
                    if (std::abs(C_) > 1e-6) { // 避免除零错误
                        // flow(0) = -C_ / A_;         // x
                        // flow(1) = -C_ / B_;         // y
                        flow.x = -C_ / A_;         // x
                        flow.y = -C_ / B_;         // y
                        // LOG(ERROR) << "Optical Flow at (" << p.x << "," << p.y << "): u=" << flow.x << ", v=" << flow.y << std::endl;
                        // flow /= (max_t - min_t);
                        // LOG(ERROR) << "Actual Optical Flow at (" << p.x << "," << p.y << "): u=" << flow(0) << ", v=" << flow(1) << std::endl;
                        pre_points.emplace_back(flow);
                        pre_inliers.emplace_back(p);
                    }
                    else
                    {
                        // LOG(ERROR) << "C_ is too small" << std::endl;
                        continue;
                    } 

                    ransac_B = std::chrono::high_resolution_clock::now();
                } 
                

                {
                    std::chrono::duration<double, std::milli> elapsed;
                    elapsed = ransac_A - ransac_iter_decenterize_time;
                    LOG(ERROR) << "Ransac2 SVD Time: " << elapsed.count() << std::endl;
                    elapsed = ransac_B - ransac_A;
                    LOG(ERROR) << "Ransac2 flow calculate Time: " << elapsed.count() << std::endl;
                }

            }

            std::chrono::time_point<std::chrono::high_resolution_clock> ransac_end_time = std::chrono::high_resolution_clock::now();

            if(pre_error < ransac_err)
            {
                ransac_err = pre_error;
                best_inliers.clear();
                flow_pre_points.clear();

                best_inliers = pre_inliers;
                flow_pre_points = pre_points;

                LOG(ERROR) << "OpFlow Update: iter = " << iterator << " error = " << ransac_err << " select_poins = " << best_inliers.size() << std::endl;
            }

            std::chrono::time_point<std::chrono::high_resolution_clock> flow_ransac_iter_time = std::chrono::high_resolution_clock::now();
            {
                std::chrono::duration<double, std::milli> elapsed;
                elapsed = flow_ransac_iter_time - ransac_init_time;
                LOG(ERROR) << "Ransac Total Time: " << elapsed.count() << std::endl;
                elapsed = ransac_sample_time - ransac_init_time;
                LOG(ERROR) << "Ransac Sample: " << elapsed.count() << std::endl;
                elapsed = ransac_end_time - ransac_sample_time;
                LOG(ERROR) << "Ransac Run: " << elapsed.count() << std::endl;
                elapsed = flow_ransac_iter_time - ransac_end_time;
                LOG(ERROR) << "Ransac Select: " << elapsed.count() << std::endl;
            }

        }
        // * /

        
        std::chrono::time_point<std::chrono::high_resolution_clock> flow_ransac_time = std::chrono::high_resolution_clock::now();


        // LOG(ERROR) << "INFO All Result: " << std::endl;
        LOG(ERROR) << "flow_pre_points.size() = " << flow_pre_points.size() << std::endl;
        if(flow_pre_points.size() < 1)
            return false;
        double norm_mean = 0.0;
        double norm_std = 0.0;
        double rad_mean = 0.0;
        double rad_std = 0.0;
        Eigen::MatrixXd flow_matrix(flow_pre_points.size(), 2);
        std::vector<std::pair<double, double>> flow_collect;
        flow_collect.reserve(flow_pre_points.size());
        for(int i = 0; i < flow_pre_points.size(); i++)
        {
            double norm_i = flow_pre_points[i].norm();
            double rad_i = flow_pre_points[i].rad();
            flow_matrix.row(i) << norm_i, rad_i;
            flow_collect.push_back(std::pair<double, double>(norm_i, rad_i));
            // LOG(ERROR) << "flow_collect = " << flow_collect[i].first << ", " << flow_collect[i].second << std::endl;
        }
        // LOG(ERROR) << "M_norm_rad = " << flow_matrix << std::endl;

        std::chrono::time_point<std::chrono::high_resolution_clock> filter_start_time = std::chrono::high_resolution_clock::now();

        // 2 * \sigma = 0.9544
        int skip_norm = 0;
        int skip_rad = 0;
        double num_sigma = 2.0; // 0.5
        int interation = 3;
        for(int iter = 1; iter < interation; iter++)
        {
            // 计算第一列 (norm) 的均值和标准差
            // auto [mean1, stddev1] = computeMeanAndStdDev(flow_matrix.col(0));
            // LOG(ERROR) << "norm means = " << mean1 << " std = " << stddev1 << std::endl;

            // 计算第二列 (rad) 的均值和标准差
            // auto [mean2, stddev2] = computeMeanAndStdDev(flow_matrix.col(1));
            // LOG(ERROR) << "rad means = " << mean2 << " std = " << stddev2 << std::endl;

            std::vector<std::pair<double, double>> mean_std_collect = computeMeanAndStdDev(flow_collect);
            // LOG(ERROR) << "norm means = " << mean_std_collect[0].first << " std = " << mean_std_collect[0].second << std::endl;
            // LOG(ERROR) << "rad means = " << mean_std_collect[1].first << " std = " << mean_std_collect[1].second << std::endl;

            for (int i = flow_pre_points.size() - 1; i >= 0; --i)
            {
                if (std::abs(flow_matrix(i, 0) - mean_std_collect[0].first) > num_sigma * mean_std_collect[0].second)
                {
                    best_inliers.erase(best_inliers.begin() + i);
                    flow_pre_points.erase(flow_pre_points.begin() + i);
                    flow_collect.erase(flow_collect.begin() + i);
                    skip_norm++;
                    continue;
                }
                // if (std::abs(flow_matrix(i, 1) - mean_std_collect[1].first) > 2 * num_sigma * mean_std_collect[1].second)
                // {
                //     best_inliers.erase(best_inliers.begin() + i);
                //     flow_pre_points.erase(flow_pre_points.begin() + i);
                //     flow_collect.erase(flow_collect.begin() + i);
                //     skip_rad++;
                //     continue;
                // }
            }
        }
        LOG(ERROR) << "iteration" << interation << ", "<< num_sigma 
                    << " * σ: skip_norm = " << skip_norm << ", skip_rad = " << skip_rad;
        LOG(ERROR) << "final size = " << flow_collect.size() << std::endl;

        // std::chrono::time_point<std::chrono::high_resolution_clock> filter_end_time = std::chrono::high_resolution_clock::now();

        /*
        Eigen::MatrixXd final_flow_matrix(flow_collect.size(), 2);
        for(int i = 0; i < flow_collect.size(); i++)
        {
            double norm_i = flow_collect[i].first;
            double rad_i = flow_collect[i].second;
            final_flow_matrix.row(i) << norm_i, rad_i;

            LOG(ERROR) << "flow_collect[i] = " << flow_collect[i].first << ", " << flow_collect[i].second << std::endl;
        }
        LOG(ERROR) << "final_flow_matrix = " << final_flow_matrix << std::endl;
        // * /

        std::chrono::time_point<std::chrono::high_resolution_clock> flow_end_time = std::chrono::high_resolution_clock::now();


        {
            std::chrono::duration<double, std::milli> elapsed;
            elapsed = flow_end_time - flow_start_time;
            LOG(ERROR) << "Flow Total Time: " << elapsed.count() << std::endl;
            elapsed = flow_init_time - flow_start_time;
            LOG(ERROR) << "Flow Init: " << elapsed.count() << std::endl;
            elapsed = flow_ransac_time - flow_init_time;
            LOG(ERROR) << "Flow Init2: " << elapsed.count() << std::endl;
            elapsed = filter_start_time - flow_ransac_time;
            LOG(ERROR) << "Flow Ransac: " << elapsed.count() << std::endl;
            // elapsed = filter_end_time - filter_start_time;
            // LOG(ERROR) << "Flow Filter: " << elapsed.count() << std::endl;
            elapsed = flow_end_time - filter_start_time;
            LOG(ERROR) << "Flow Filter: " << elapsed.count() << std::endl;
        }

        if(flow_collect.size() < 6)
        {
            LOG(ERROR) << "not enough flow_collect" << std::endl;
            return false;
        }
        // LOG(ERROR) << "2 * \sigma: skip_norm = " << skip_norm << ", skip_rad = " << skip_rad;


        return true;
    }
*/
    bool CalculateOpFlowPrepointSingleFit()
    {
        std::chrono::time_point<std::chrono::high_resolution_clock> flow_start_time = std::chrono::high_resolution_clock::now();
        // TimeImage1 > 0 要求其中的点必须大于0.5 * radius * radius
        cv::Mat mask = TimeImage1 > 0; // (param.inlier_ratio * param.radius * param.radius);
        std::vector<cv::Point2d> valid_points;
        cv::findNonZero(mask, valid_points);
        // LOG(ERROR) << "valid_points.size = " << valid_points.size() << std::endl;
        if(valid_points.empty())
        {
            LOG(ERROR) << "not valid points" << std::endl;
            return false;
        }
        std::vector<cv::Point2d> select_points;

        std::random_device rd;
        std::mt19937 gen(rd());
        // 复制一份原始数据，用于 shuffle
        std::vector<cv::Point2d> shuffled_points = valid_points;

        // 打乱顺序
        std::shuffle(shuffled_points.begin(), shuffled_points.end(), gen);

        // 选择前 40 个（如果不够 40 个就全选）
        size_t count = std::min(size_t(20), shuffled_points.size());
        select_points.assign(shuffled_points.begin(), shuffled_points.begin() + count);

        int valid_points_size = select_points.size();
        int grid_size = 2 * radius + 1;
        double neighbor_need = ratio_inliers * (2 * radius + 1) * (2 * radius + 1);
        std::vector<cv::Point2d> pre_inliers;
        pre_inliers.reserve(valid_points_size);
        std::vector<event_flow_velocity> pre_points;
        pre_points.reserve(valid_points_size);
        Eigen::JacobiSVD<Eigen::MatrixXd> svd;
        LOG(ERROR) << "Start Ransac to Fit Optical Flow" << std::endl;
        flow_pre_points.clear();
        best_inliers.clear();

        // 全局最小二乘法
        auto all_start = std::chrono::high_resolution_clock::now();

        #pragma omp parallel for schedule(dynamic, 5)
        for (int i = 0; i < select_points.size(); ++i)
        {
            auto start = std::chrono::high_resolution_clock::now();
            const auto& p = select_points[i];
            std::vector<Eigen::Vector3d> points;
            points.reserve(2 * grid_size * grid_size);

            int neighbor_count = 0;
            for (int dx = -radius; dx <= radius; ++dx) {
                for (int dy = -radius; dy <= radius; ++dy) {
                    double p_x = p.x + dx;
                    double p_y = p.y + dy;
                    auto key = std::make_pair((int)p_x, (int)p_y);
                    auto it = TimeImage1_events.find(key);
                    if (it != TimeImage1_events.end()) {
                        for (const auto& t : it->second)
                            points.emplace_back(p_x, p_y, t);
                        neighbor_count++;
                    }
                }
            }

            if (neighbor_count < neighbor_need || points.size() < 3)
                continue;

            const int N = points.size();
            Eigen::MatrixXd Pp(N, 4);
            for (int k = 0; k < N; ++k) {
                Pp(k, 0) = points[k].x() / cam_info.K[0];
                Pp(k, 1) = points[k].y() / cam_info.K[4];
                Pp(k, 2) = points[k].z();
                Pp(k, 3) = 1.0;
            }

            Pp.rowwise() -= Pp.colwise().mean();

            int best_inlier_count = -1;
            Eigen::Vector4d best_normal;

            const int ransac_iters = 10;
            const double inlier_thresh = 0.002;

            std::minstd_rand rng(std::random_device{}() + omp_get_thread_num());
            std::vector<int> indices(N);
            std::iota(indices.begin(), indices.end(), 0);

            Eigen::MatrixXd A(3, 4);
            Eigen::VectorXd errors(N);

            for (int iter = 0; iter < ransac_iters; ++iter) {
                auto start_local = std::chrono::high_resolution_clock::now();

                std::shuffle(indices.begin(), indices.end(), rng);
                for (int j = 0; j < 3; ++j)
                    A.row(j) = Pp.row(indices[j]);

                Eigen::JacobiSVD<Eigen::MatrixXd> svd(A, Eigen::ComputeThinV);
                Eigen::Vector4d normal = svd.matrixV().col(2);

                errors.noalias() = Pp * normal;
                int inliers = (errors.array().abs() < inlier_thresh).count();

                if (inliers > best_inlier_count) {
                    best_inlier_count = inliers;
                    best_normal = normal;
                }

                auto end_local = std::chrono::high_resolution_clock::now();
                std::chrono::duration<double> duration = end_local - start_local;
                LOG(ERROR) << "local total = " << duration.count() << " s";
            }

            LOG(ERROR) << "best_inlier_count = " << best_inlier_count;

            const double A_ = best_normal(0);
            const double B_ = best_normal(1);
            const double C_ = best_normal(2);

            LOG(ERROR) << "C_ = " << std::abs(C_);

            if (std::abs(C_) > 1e-6) {
                event_flow_velocity flow;
                flow.x = -C_ / A_;
                flow.y = -C_ / B_;
                LOG(ERROR) << "local flow = " << flow.x << ", " << flow.y;

                #pragma omp critical
                {
                    flow_pre_points.push_back(flow);
                    best_inliers.push_back(p);
                }
            }

            auto end = std::chrono::high_resolution_clock::now();
            std::chrono::duration<double> duration = end - start;
            LOG(ERROR) << "global total = " << duration.count() << " s";
        }

        auto all_end = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> duration = all_end - all_start;
        LOG(ERROR) << "total_total = " << duration.count() << " s";



        // if(flow_collect.size() < 6)
        if(best_inliers.size() < 6)
        {
            LOG(ERROR) << "not enough flow_collect" << std::endl;
            return false;
        }
        // LOG(ERROR) << "2 * \sigma: skip_norm = " << skip_norm << ", skip_rad = " << skip_rad;


        return true;
    }



    // 计算均值和标准差，并返回 (mean, stddev)
    std::pair<double, double> computeMeanAndStdDev(const Eigen::VectorXd& col_vals) {
        double mean = col_vals.mean();
        double variance = (col_vals.array() - mean).square().sum() / (col_vals.size() - 1);
        double stddev = std::sqrt(variance);
        return {mean, stddev};
    }

    // 计算均值和标准差，并返回 (mean, stddev) vector
    /*std::vector<std::pair<double, double>> computeMeanAndStdDev(const std::vector<std::pair<double, double>>& col_vals) {
        if (col_vals.empty()) return {}; // 处理空输入

        double sum_x = 0.0, sum_y = 0.0;
        double sum_x2 = 0.0, sum_y2 = 0.0;
        size_t n = col_vals.size();

        // 计算均值
        for (const auto& p : col_vals) {
            sum_x += p.first;
            sum_y += p.second;
        }
        double mean_x = sum_x / n;
        double mean_y = sum_y / n;

        // 计算方差
        for (const auto& p : col_vals) {
            sum_x2 += (p.first - mean_x) * (p.first - mean_x);
            sum_y2 += (p.second - mean_y) * (p.second - mean_y);
        }
        double stddev_x = std::sqrt(sum_x2 / n);
        double stddev_y = std::sqrt(sum_y2 / n);

        return {{mean_x, stddev_x}, {mean_y, stddev_y}};
    }*/

    std::vector<std::pair<double, double>> computeMeanAndStdDev(const std::vector<std::pair<double, double>>& col_vals) {
        if (col_vals.empty()) return {};

        double mean_x = 0.0, mean_y = 0.0;
        double m2_x = 0.0, m2_y = 0.0;
        size_t n = 0;

        for (const auto& p : col_vals) {
            ++n;
            double delta_x = p.first - mean_x;
            mean_x += delta_x / n;
            m2_x += delta_x * (p.first - mean_x);

            double delta_y = p.second - mean_y;
            mean_y += delta_y / n;
            m2_y += delta_y * (p.second - mean_y);
        }

        double stddev_x = std::sqrt(m2_x / n);
        double stddev_y = std::sqrt(m2_y / n);

        return {{mean_x, stddev_x}, {mean_y, stddev_y}};
    }


/*
    double AccumuConstractEventsWindows(double timestamp, cv::Mat& time_img, std::unordered_map<std::pair<int, int>, double, 
                                                pair_hash>& event_link, long int need_events = 1000)
    {
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
*/
    // 时间图像压缩函数
    void AccumulateEvents(double start_time, double end_time, cv::Mat& TimeImage, long int& count) {
        
        long int events_count = 0;
        int end_index = -1;

        if(event_stream.back()->events.back().ts.toSec() < end_time) return;
        // for(std::iterator<dvs_msgs::EventArray>::)

        // int count = 0;
        for (const auto& event_array : event_stream) {
            double first_event_time = event_array->events.front().ts.toSec();
            double last_event_time = event_array->events.back().ts.toSec();

            if (first_event_time > end_time) break;
            if (last_event_time < start_time) continue;

            for (const auto& event : event_array->events) {
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

    std::unordered_map<int, std::unordered_map<int, std::vector<cv::Point2d>>> InitgridBased
        (const std::vector<cv::Point2d>& nonZeroPoints, int grid_size)
    {
        std::unordered_map<int, std::unordered_map<int, std::vector<cv::Point2d>>> grid_map;
        // **1. 先收集每个网格中的所有点**
        for (const auto& p : nonZeroPoints) {
            // int grid_x = static_cast<int>(p.x) / grid_size;
            // int grid_y = static_cast<int>(p.y) / grid_size;
            // grid_map[grid_x][grid_y].push_back(p);

            if(p.x < 2 * radius || p.x > sensor_width - 2 * radius - 1 
                    || p.y < 2 * radius || p.y > sensor_height - 2 * radius - 1)
                    continue;

            int cell_x = static_cast<int>(std::floor(p.x / grid_size));
            int cell_y = static_cast<int>(std::floor(p.y / grid_size));
            grid_map[cell_x][cell_y].push_back(p);
        }

        return grid_map;
    }

    void gridBasedSampling(std::unordered_map<int, std::unordered_map<int, std::vector<cv::Point2d>>> grid_map,
                        // const std::vector<cv::Point2d>& nonZeroPoints,
                        std::vector<cv::Point2d>& sampledPoints,
                        int points_num) {
        sampledPoints.clear();
        
        // std::unordered_map<int, std::unordered_map<int, std::vector<cv::Point2d>>> grid_map;
        std::random_device rd;
        std::mt19937 g(rd());

        /*{

        // **1. 先收集每个网格中的所有点**
        for (const auto& p : nonZeroPoints) {
            int grid_x = static_cast<int>(p.x) / grid_size;
            int grid_y = static_cast<int>(p.y) / grid_size;
            grid_map[grid_x][grid_y].push_back(p);
        }
        }*/    

        // **2. 从每个网格随机挑选一个点**
        int point_count = 0;
        for (auto& row : grid_map) {
            for (auto& cell : row.second) {
                auto& points = cell.second;
                if (!points.empty()) {
                    std::shuffle(points.begin(), points.end(), g); // 乱序
                    sampledPoints.push_back(points.front()); // 取第一个点
                    point_count ++;
                }
                if(point_count > points_num)
                    break;
            }
            if(point_count > points_num)
                break;
        }

        // **3. 如果点数超出限制，随机裁剪**
        // if (sampledPoints.size() > points_num) {
        //     std::shuffle(sampledPoints.begin(), sampledPoints.end(), g);
        //     sampledPoints.resize(points_num);
        // }

        // LOG(ERROR) << "sampledPoints.size = " << sampledPoints.size() << std::endl;
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


    // 计算角速度
    bool LSQAugularVelocityEsti(geometry_msgs::TwistWithCovarianceStamped& radar_vel) {
        // assert(best_inliers.size() > 2);

        Eigen::Matrix3d K_inv = K.inverse();
        Eigen::Matrix3d R_re = T_re.block(0, 0, 3, 3);


        Eigen::MatrixXd A(best_inliers.size(), 3);
        Eigen::VectorXd b(best_inliers.size());
        Eigen::Vector3d linear_vel(radar_vel.twist.twist.linear.x, radar_vel.twist.twist.linear.y, radar_vel.twist.twist.linear.z);

        LOG(ERROR) << "linear_vel = " << linear_vel.transpose() << std::endl;
        if(linear_vel.norm() < 1e-6)
        {
            LOG(ERROR) << "radar ego velocity is valid!" << std::endl;
            return false;
        }

        LOG(ERROR) << "Mid Debug for flow: " << best_inliers.size() << std::endl;

        LOG(ERROR) << "K.inverse() = " << K_inv << std::endl;

        for (int i = 0; i < best_inliers.size(); ++i) {
            // Eigen::Vector3d pixel(best_inliers[i].x , best_inliers[i].y, 1.0);
            // Eigen::Vector3d pixel_cord = K_inv * pixel;

            Eigen::Vector3d pixel_cord = K_inv * Eigen::Vector3d(best_inliers[i].x , best_inliers[i].y, 1.0);

            // Eigen::Vector3d pixel_cord = pixel;
            Eigen::Matrix3d pixel_skew = skew(pixel_cord);
            Eigen::Vector3d prefix = pixel_skew * R_re * linear_vel;
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
            // flow = K.inverse() * flow;

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
        LOG(INFO) << "First Angular Velocity Residual = " << residual << std::endl;

        // 加权迭代一次
        Eigen::VectorXd weights = residual.array().square().inverse();
        Eigen::MatrixXd W = weights.asDiagonal();
        Eigen::MatrixXd A_weighted = W * A;
        Eigen::VectorXd b_weighted = W * b;
        LOG(ERROR) << "Debug Angular Velocity, weights = " << W
                    << "A_weighted = " << A_weighted
                    << "b_weighted = " << b_weighted << std::endl;

        angular_vec = A_weighted.colPivHouseholderQr().solve(b_weighted);

        LOG(ERROR) << "Second Angular Velocity = " << std::endl;
        LOG(ERROR) << angular_vec.transpose() << std::endl;

        residual = A_weighted * angular_vec - b_weighted;
        LOG(ERROR) << "Second Angular Velocity Residual = " << residual << std::endl;

        // 将事件系的角速度投影回雷达系
        angular_vec = R_re.transpose() * angular_vec;

        radar_vel.twist.twist.angular.x = angular_vec(0);
        radar_vel.twist.twist.angular.y = angular_vec(1);
        radar_vel.twist.twist.angular.z = angular_vec(2);

        // 计算协方差矩阵
        // residual = A_weighted * angular_vec - b_weighted;
        double sigma_r_squared = (residual.squaredNorm()) / (A_weighted.rows() - A_weighted.cols());
        Eigen::Matrix3d covariance_matrix = sigma_r_squared * (A_weighted.transpose() * A_weighted).inverse();
        // LOG(INFO) << "Angular Velocity covariance_matrix = " << covariance_matrix << std::endl;

        // 将协方差矩阵填充到 Covariance 中
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                radar_vel.twist.covariance[21 + i * 6 + j] = covariance_matrix(i, j);  // 填充角速度的协方差
            }
        }

        // std::fstream optical_flow_file("/media/hao/hao2/228/test/lab/doc/optical_flow.txt", std::ios::out | std::ios::app);
        // optical_flow_file << "calculate w: A * w = b " << std::endl;
        // optical_flow_file << "inliners: " << best_inliers.size() << std::endl;
        // optical_flow_file << "A = " << A << std::endl;
        // optical_flow_file << "b = " << b << std::endl;
        // optical_flow_file << "res = " << residual.transpose() << std::endl;
        // optical_flow_file.close();

        // dji seq outut
        // LOG_Velocity(radar_vel, "/media/hao/hao2/228/test/lab/detector.tum");

        // dvs seq output
        // LOG_Velocity(radar_vel, "/home/hao/Desktop/radar-event-new/src/TwistEstimator/output/detector.tum");

        return true;
    }

    // Huber回归的实现
    double huber_loss(double residual, double delta) {
        if (std::abs(residual) <= delta) {
            return 0.5 * residual * residual;
        } else {
            return delta * (std::abs(residual) - 0.5 * delta);
        }
    }

    // 计算角速度
    bool NormalAugularVelocityEsti(geometry_msgs::TwistWithCovarianceStamped& radar_vel) {
        // assert(best_inliers.size() > 2);

        Eigen::Matrix3d K_inv = K.inverse();
        Eigen::Matrix3d R_re = T_re.block(0, 0, 3, 3);

        Eigen::MatrixXd A(3 * best_inliers.size(), 3);
        Eigen::VectorXd b(3 * best_inliers.size());

        // Eigen::MatrixXd A(best_inliers.size(), 3);
        // Eigen::VectorXd b(best_inliers.size());

        // Eigen::VectorXd b(best_inliers.size());
        Eigen::Vector3d linear_vel(radar_vel.twist.twist.linear.x, radar_vel.twist.twist.linear.y, radar_vel.twist.twist.linear.z);

        LOG(ERROR) << "linear_vel = " << linear_vel.transpose() << std::endl;
        if(linear_vel.norm() < 1e-6)
        {
            LOG(ERROR) << "radar ego velocity is valid!" << std::endl;
            return false;
        }

        LOG(ERROR) << "Mid Debug for flow: " << best_inliers.size() << std::endl;

        LOG(ERROR) << "K.inverse() = " << K_inv << std::endl;

        for (int i = 0; i < best_inliers.size(); ++i) {
            // Eigen::Vector3d pixel(best_inliers[i].x , best_inliers[i].y, 1.0);
            // Eigen::Vector3d pixel_cord = K_inv * pixel;

            Eigen::Vector3d pixel_cord = K_inv * Eigen::Vector3d(best_inliers[i].x , best_inliers[i].y, 1.0);

            // Eigen::Vector3d pixel_cord = pixel;
            Eigen::Matrix3d pixel_skew = skew(pixel_cord);
            Eigen::Vector3d prefix = pixel_skew * R_re * linear_vel;
            
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
            // flow = K.inverse() * flow;

            // LOG(INFO) << "flow cordinate = " << flow.transpose() << std::endl;

            // Eigen::Vector3d norm_i;
            // norm_i = flow / flow.norm();    // 法向光流
            // Eigen::Vector3d norm_flow_i = norm_i / flow.norm();

            // Eigen::Vector2d flow2d(flow_pre_points[i].x, flow_pre_points[i].y);
            // if (flow2d.norm() < 1e-6) continue;
            // Eigen::Vector3d normal_flow;
            // normal_flow.head<2>() = -flow2d.normalized();
            // normal_flow(2) = 0.0;


            if (std::abs(flow(0)) < 1e-6 || std::abs(flow(1)) < 1e-6) {
                continue;  // 或者填充默认值跳过该点
            }

            // 从光流中恢复梯度
            Eigen::Vector3d grad;
            grad << -1.0 / flow(0), -1.0 / flow(1), 0.0;
            double normal_norm = 1.0 / grad.norm();
            Eigen::Vector3d normal_flow = grad * normal_norm;


            LOG(ERROR) << "debug normal flow = " << normal_flow.transpose() << " norm = " << normal_norm << std::endl; 
            // normal velocity
            // HAO TODO： 5-23修改
            A.block(3 * i, 0, 3, 3) = prefix * normal_flow.transpose() * pixel_skew;
            b.block(3 * i, 0, 3, 1) = -1.0 * prefix * normal_norm;    //TODO: modify in paper

            // HAO TODO: 这个会不完整
            // A.block(i, 0, 1, 3) = normal_flow.transpose() * pixel_skew;
            // b(i) = -1.0 * normal_norm;    //TODO: modify in paper

            // A.row(i) = prefix.transpose() * pixel_skew;
            // b(i) = -1.0 * prefix.transpose() * flow;    //TODO: modify in paper

            // LOG(INFO) << "A(i) = " << A.row(i).transpose() << std::endl;
            // LOG(INFO) << "b(i) = " << b(i) << std::endl;
        }

        std::fstream angular_file("/home/hao/Desktop/twist_ws/src/TwistEstimator/output/LSQ_Ang.txt", 
                                    std::ios::out | std::ios::app);
        angular_file << "A = [" << A << "]" << std::endl;
        angular_file << "b = [" << b << "]" << std::endl;
        angular_file.close();
        Eigen::Vector3d angular_vec = A.colPivHouseholderQr().solve(b);

        // LOG(ERROR) << "A = " << A << std::endl;
        // LOG(ERROR) << "b = " << b << std::endl; 

        // angular_vec = lpf.filter(angular_vec);

        LOG(ERROR) << "First Angular Velocity = " << std::endl;
        LOG(ERROR) << angular_vec.transpose() << std::endl;

        // 计算残差
        Eigen::VectorXd residual = A * angular_vec - b;
        LOG(ERROR) << "First Angular Velocity Residual = " << residual << std::endl;

        // Huber损失权重
        /*
        double delta = 0.01; // Huber损失的阈值
        Eigen::VectorXd weights(residual.size());
        for (int i = 0; i < residual.size(); ++i) {
            double residual_value = residual(i);
            weights(i) = huber_loss(residual_value, delta);
        }
        weights = weights.array().square().inverse();
        */
        double delta = 0.01; // Huber损失的阈值
        Eigen::VectorXd weights(residual.size());
        for (int i = 0; i < residual.size(); ++i)
            weights(i) = 1.0 / (huber_loss(residual(i), delta) * huber_loss(residual(i), delta));

        // 加权迭代一次
        // Eigen::VectorXd weights = residual.array().square().inverse();
        Eigen::MatrixXd W = weights.asDiagonal();
        Eigen::MatrixXd A_weighted = W * A;
        Eigen::VectorXd b_weighted = W * b;
        // LOG(ERROR) << "Debug Angular Velocity, weights = " << W
        //             << "A_weighted = " << A_weighted
        //             << "b_weighted = " << b_weighted << std::endl;

        angular_vec = A_weighted.colPivHouseholderQr().solve(b_weighted);

        LOG(ERROR) << "Second Angular Velocity = " << std::endl;
        LOG(ERROR) << angular_vec.transpose() << std::endl;

        residual = A_weighted * angular_vec - b_weighted;
        LOG(ERROR) << "Second Angular Velocity Residual = " << residual << std::endl;

        // 将事件系的角速度投影回雷达系
        angular_vec = R_re.transpose() * angular_vec;

        radar_vel.twist.twist.angular.x = angular_vec(0);
        radar_vel.twist.twist.angular.y = angular_vec(1);
        radar_vel.twist.twist.angular.z = angular_vec(2);

        // 计算协方差矩阵
        // residual = A_weighted * angular_vec - b_weighted;
        double sigma_r_squared = (residual.squaredNorm()) / (A_weighted.rows() - A_weighted.cols());
        Eigen::Matrix3d covariance_matrix = sigma_r_squared * (A_weighted.transpose() * A_weighted).inverse();
        // LOG(INFO) << "Angular Velocity covariance_matrix = " << covariance_matrix << std::endl;

        // 将协方差矩阵填充到 Covariance 中
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                radar_vel.twist.covariance[21 + i * 6 + j] = covariance_matrix(i, j);  // 填充角速度的协方差
            }
        }
        return true;
    }



    bool init_output = false;
    void LOG_Velocity(geometry_msgs::TwistWithCovarianceStamped& radar_vel, std::string filename)
    {
        if(!init_output)
        {
            std::fstream ss(filename, std::ios::out | std::ios::trunc);
            ss.clear();
            ss.close();
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
        sensor_msgs::PointCloud2 cur_inliers;
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

            // 如果事件相机不足，返回, 等待
            // if (event_stream.back()->header.stamp < process_time) {
            if (event_stream.back()->events.back().ts < process_time) {
                // std::cout << "event data is not new than radar" << std::endl;
                // LOG(ERROR) << "event data is not new than radar" << std::endl;
                // event_stream.clear();
                return false;
            }

            // 事件数据比较新，删除雷达数据  
            // if (event_stream.front()->header.stamp > process_time) {
            if (event_stream.front()->events.front().ts > process_time) {
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
            /*if (process_time > cur_inliers.header.stamp) {
                // std::cout << "event data is not new than radar" << std::endl;
                // LOG(ERROR) << "radar data is not new than event" << std::endl;
                radar_inliers.pop_front();
                return false;
            } */     

            twist_ = radar_doppler_velocity.front();    
        }
        

        radar_doppler_velocity.pop_front(); 
        radar_inliers.pop_front();  

        {
            std::chrono::time_point<std::chrono::high_resolution_clock> start_time = std::chrono::high_resolution_clock::now();
            double process_time_sec = process_time.toSec();

            EventArray2EventVec();
            std::chrono::time_point<std::chrono::high_resolution_clock> time1 = std::chrono::high_resolution_clock::now();
            bool accumu_done = AccumulateTimeImage(process_time_sec);
            std::chrono::time_point<std::chrono::high_resolution_clock> time2 = std::chrono::high_resolution_clock::now();
            bool have_flow = false;
            if(accumu_done)
                have_flow = CalculateOpFlowPrepointSingleFit();
            else
            {
                LOG(ERROR) << "Not enough event data!" << std::endl;
                // return false;
            }
            std::chrono::time_point<std::chrono::high_resolution_clock> time3 = std::chrono::high_resolution_clock::now();
            
            
            
            std::chrono::time_point<std::chrono::high_resolution_clock> time4;    
            std::chrono::time_point<std::chrono::high_resolution_clock> time5; 
            // if(!(have_flow && LSQAugularVelocityEsti(twist_)))

            if(!have_flow)
            {
                LOG(ERROR) << "No flow " << std::endl;
            }

            // 法向光流
            if(!(have_flow && NormalAugularVelocityEsti(twist_)))
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

                return false;
            } 
            else
            {
                LOG(ERROR) << "PublishTimeImages" << std::endl;
                time4 = std::chrono::high_resolution_clock::now();
                PublishTimeImages(TimeImage1, TimeImage1_time);
                time5 = std::chrono::high_resolution_clock::now();
            }
            // std::chrono::time_point<std::chrono::high_resolution_clock> time4 = std::chrono::high_resolution_clock::now();
            
            // std::chrono::time_point<std::chrono::high_resolution_clock> time5 = std::chrono::high_resolution_clock::now();

            assert(twist_.header.stamp == cur_inliers.header.stamp 
                && "radar_inliers not valid Or Time is not correct!");

            // INFO_Velocity(twist_);
            // LOG_Velocity(twist_, "/home/hao/Desktop/twist_ws/src/TwistEstimator/output/detector.tum");


            LOG(ERROR) << "Final Debug for flow: " << best_inliers.size() << std::endl;
            twist_result2_.push_back(TwistData2(twist_, cur_inliers, best_inliers, flow_pre_points));
            std::chrono::time_point<std::chrono::high_resolution_clock> end_time = std::chrono::high_resolution_clock::now();

            {
                std::chrono::duration<double, std::milli> elapsed;
                elapsed = end_time - start_time;
                LOG(ERROR) << "Total Time: " << elapsed.count() << std::endl;
                elapsed = time1 - start_time;
                LOG(ERROR) << "EventArray2EventVec: " << elapsed.count() << std::endl;
                elapsed = time2 - time1;
                LOG(ERROR) << "AccumulateTimeImage: " << elapsed.count() << std::endl;
                elapsed = time3 - time2;
                LOG(ERROR) << "CalculateOpFlowPrepointSingleFit: " << elapsed.count() << std::endl;
                elapsed = time4 - time3;
                LOG(ERROR) << "LSQAugularVelocityEsti: " << elapsed.count() << std::endl;
                elapsed = time5 - time4;
                LOG(ERROR) << "PublishTimeImages: " << elapsed.count() << std::endl;
            }

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

    double GetProcessTime()
    {
        return process_time.toSec();
    }

// 成员变量
public:
    std::vector<sensor_msgs::Image::Ptr> raw_img_buffer;
    std::deque<dvs_msgs::EventArray::Ptr> event_stream;
    std::deque<geometry_msgs::TwistWithCovarianceStamped> radar_doppler_velocity;
    std::deque<sensor_msgs::PointCloud2> radar_inliers;
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
    // cv::Mat distort_cv;
    Eigen::Matrix3d K;      // intrincs matrix
    Eigen::Matrix3d distort;      // distort parameters
    int sensor_width;
    int sensor_height;
    Eigen::Matrix4d T_re;

    cv::Mat TimeImage1;

    cv::Mat TimeImage1_time;

    cv::Mat TimeImage2;

    std::unordered_map<std::pair<int, int>, std::vector<double>, pair_hash> TimeImage1_events;

    cv::Mat TimeImage1_color;

    cv::Mat TimeImage1_on_event;
    cv::Mat TimeImage1_off_event;
    
    cv::Point2d flow_velocity;
    // std::vector<double*> flow_velocity;
    std::vector<event_flow_velocity> flow_pre_points;
    int radius;
    double ratio_inliers;
    bool ignore_polarity;

    std::vector<dvs_msgs::Event> event_buffer;
    image_geometry::PinholeCameraModel camera_model;

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

    ros::Publisher pub_raw_image_;

    // 存储所有有效光流
    std::vector<Eigen::Vector2d> valid_flows;  

    sensor_msgs::CameraInfo cam_info;
    /*s
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
    */
    // 构建反对称矩阵
    Eigen::Matrix3d skew(const Eigen::Vector3d& vec) const {
        Eigen::Matrix3d result;
        result << 0, -vec(2), vec(1), vec(2), 0, -vec(0), -vec(1), vec(0), 0;
        return result;
    }
};


#endif