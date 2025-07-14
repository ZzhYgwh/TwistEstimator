#pragma once

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

extern std::mutex detector_data_mutex;
// std::mutex radar_callback_mutex;     // 回调的互斥
// std::mutex event_callback_mutex;


// 定义光流速度结构体
struct event_flow_velocity {
    double x;
    double y;
    event_flow_velocity() : x(0), y(0) {}
    event_flow_velocity(double _x, double _y) : x(_x), y(_y) {}
    event_flow_velocity(Eigen::Vector2d vel) : x(vel.x()), y(vel.y()) {}
    double norm()   // 光流长度
    {
        return std::sqrt(x * x + y * y);
    }

    double rad()
    {
        if(x == 0)
            if(y == 0)
                return 0;
            else
                if(y > 0)
                    return M_PI / 2;
                else
                    return -M_PI / 2;

        return std::tan(y / x);
    }
};

struct EventParams {
    std::string topic;
    int freq;
    double fx;
    double fy;
    double cx;
    double cy;
    int resolution[2];
    double deltaT;
    double t1;
    int mesh_size;
    double timeshift_cam_imu;

    double k1;
    double k2;
    double p1; 
    double p2;
    double k3;
    int t1_count;
    int t2_count;
    double sae_time;
};

struct RadarEventParams {
    Eigen::Matrix4d T_re;
};

struct RadarParams{
    std::string topic;
    std::string type;
    // type: ti_mmwave or ars548
};

struct TwistData
{
  TwistData(){};
  TwistData(geometry_msgs::TwistWithCovarianceStamped twist, sensor_msgs::PointCloud2& points,
   std::vector<cv::Point2d> best_inliers, event_flow_velocity best_flow): 
   best_inliers(best_inliers), best_flow(best_flow), point_cloud(points), is_relative_time(false)
  {
    timestamp = twist.header.stamp.toSec();
    angular_vel_vec_ << twist.twist.twist.angular.x, twist.twist.twist.angular.y, twist.twist.twist.angular.z;
    linear_vel_vec_ << twist.twist.twist.linear.x, twist.twist.twist.linear.y, twist.twist.twist.linear.z;
    twist_vec_ << angular_vel_vec_, linear_vel_vec_;

    linear_cov << twist.twist.covariance[0], twist.twist.covariance[1], twist.twist.covariance[2],
                  twist.twist.covariance[3], twist.twist.covariance[4], twist.twist.covariance[5],
                  twist.twist.covariance[6], twist.twist.covariance[7], twist.twist.covariance[8];

    angular_cov << twist.twist.covariance[21], twist.twist.covariance[22], twist.twist.covariance[23],
                  twist.twist.covariance[27], twist.twist.covariance[28], twist.twist.covariance[29],
                  twist.twist.covariance[33], twist.twist.covariance[34], twist.twist.covariance[35];
  }
  bool is_relative_time;
  double timestamp;
  Eigen::Vector3d angular_vel_vec_;
  Eigen::Vector3d linear_vel_vec_; 
  Eigen::Matrix<double, 6, 1> twist_vec_;

  std::vector<cv::Point2d> best_inliers;
  event_flow_velocity best_flow;

  sensor_msgs::PointCloud2 point_cloud;

  Eigen::Matrix3d linear_cov;
  Eigen::Matrix3d angular_cov; 
};

struct TwistData2
{
  TwistData2(){};
  TwistData2(double timestamp_)
  {
    is_relative_time = false;
    timestamp = timestamp_;
    angular_vel_vec_.setZero();
    linear_vel_vec_.setZero();
    twist_vec_.setZero();
    linear_cov.setZero();
    angular_cov.setZero();
  };

  TwistData2(geometry_msgs::TwistWithCovarianceStamped twist, const sensor_msgs::PointCloud2& points,
   std::vector<cv::Point2d> best_inliers, std::vector<event_flow_velocity> best_flow): 
   best_inliers(best_inliers), best_flow(best_flow), point_cloud(points), is_relative_time(false)
  {
    timestamp = twist.header.stamp.toSec();
    angular_vel_vec_ << twist.twist.twist.angular.x, twist.twist.twist.angular.y, twist.twist.twist.angular.z;
    linear_vel_vec_ << twist.twist.twist.linear.x, twist.twist.twist.linear.y, twist.twist.twist.linear.z;
    twist_vec_ << angular_vel_vec_, linear_vel_vec_;

    linear_cov << twist.twist.covariance[0], twist.twist.covariance[1], twist.twist.covariance[2],
                  twist.twist.covariance[3], twist.twist.covariance[4], twist.twist.covariance[5],
                  twist.twist.covariance[6], twist.twist.covariance[7], twist.twist.covariance[8];

    angular_cov << twist.twist.covariance[21], twist.twist.covariance[22], twist.twist.covariance[23],
                  twist.twist.covariance[27], twist.twist.covariance[28], twist.twist.covariance[29],
                  twist.twist.covariance[33], twist.twist.covariance[34], twist.twist.covariance[35];
  }
  bool is_relative_time;
  double timestamp;
  Eigen::Vector3d angular_vel_vec_;
  Eigen::Vector3d linear_vel_vec_; 
  Eigen::Matrix<double, 6, 1> twist_vec_;

  std::vector<cv::Point2d> best_inliers;
  std::vector<event_flow_velocity> best_flow;
  std::vector<double> norm_pre_points;

  sensor_msgs::PointCloud2 point_cloud;

  Eigen::Matrix3d linear_cov;
  Eigen::Matrix3d angular_cov; 
};

struct TwistVelocity
{
public:
    TwistVelocity(){};
    TwistVelocity(double timestamp, Eigen::Vector3d linear_vec, Eigen::Vector3d angular_vec):
        timestamp(timestamp), linear_vec(linear_vec), angular_vec(angular_vec){};
    double timestamp;
    Eigen::Vector3d linear_vec; 
    Eigen::Vector3d angular_vec;
    bool is_relative_time = false;
}; // struct TwistVelocity

// 低通滤波器
class LowPassFilter {
public:
    LowPassFilter(double alpha) : alpha_(alpha), prev_v_(Eigen::Vector3d::Zero()) {}
    
    Eigen::Vector3d filter(const Eigen::Vector3d& v) {
        // 计算低通滤波后的值
        Eigen::Vector3d filterd_v = alpha_ * v + (1.0 - alpha_) * prev_v_;
        prev_v_ = filterd_v;  // 保存当前的输出，作为下一次滤波的历史值
        return filterd_v;
    }

private:
    double alpha_;  // 平滑系数
    Eigen::Vector3d prev_v_;  // 上一次滤波输出值
};

class EventFlowDetector {
public:
    EventFlowDetector(ros::NodeHandle& nh_this, EventParams& e_params, RadarEventParams r_params, bool show_events, double smooth, int filter_num = 3, int median_radius = 3, bool ignore_polarity = false, bool use_gauss = false, double ratio_inliers = 0.1)
        : nh_this_(nh_this), event_dt(e_params.deltaT), event_t1(e_params.t1), sensor_width(e_params.resolution[0]), lpf(smooth), 
         lpf_v(smooth), filter_num(filter_num), median_radius(median_radius), ignore_polarity(ignore_polarity), use_gauss(use_gauss), ratio_inliers(ratio_inliers),
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
            LOG(ERROR) << "ratio_inliers = " << ratio_inliers << std::endl;
        }

    // 时间图像压缩函数
    void CompressTimeImageForWindow(double start_time, double end_time, cv::Mat& TimeImage, long int& count) {
        
        // int count = 0;
        for (const auto& event_array : event_stream) {
            double first_event_time = event_array.events.front().ts.toSec();
            double last_event_time = event_array.events.back().ts.toSec();

            // std::cout << "first_event_time = " << std::setprecision(20) << first_event_time << std::endl;
            // std::cout << "last_event_time = " << std::setprecision(20) << last_event_time << std::endl;
            // std::cout << std::setprecision(20) << "time = [" << start_time << ", " << end_time << "]" << std::endl;            

            if (first_event_time > end_time) break;
            if (last_event_time < start_time) continue;

            // std::cout << "loop events = " << event_array.events.size() << std::endl;
            // std::cout << "events time diff = " << (event_array.events.back().ts - event_array.events.front().ts).toSec() 
            //             / event_array.events.size() << " p/s " << std::endl;

            for (const auto& event : event_array.events) {
                double event_time = event.ts.toSec();
                if (event_time >= start_time && event_time <= end_time &&
                    event.x >= 0 && event.x < sensor_width && event.y >= 0 && event.y < sensor_height) {
                    // TimeImage.at<double>(event.y, event.x)++;
                    // TimeImage.at<double>(event.y, event.x)++;
                    // count++;

                    // 2-5 修改 区分正负极性
                    if(event.polarity)
                        TimeImage.at<double>(event.y, event.x)++;
                    // 2-14 修改 不使用负极性
                    // else
                    //     TimeImage.at<double>(event.y, event.x)--;
                    count++;
                }
            }

            std::cout << "TImage count = " << count << std::endl;
 

            // std::cout << "done!" << std::endl;
        }

        // cv::GaussianBlur(TimeImage, TimeImage, cv::Size(5, 5), 0.5);
        LOG(ERROR) << "TImage compress count = " << count << std::endl;

        /*{
            // 假设 TimeImage1 已经被定义为 cv::Mat
            cv::Mat mask;
            cv::threshold(TimeImage, mask, 0, 1, cv::THRESH_BINARY); // 将大于 0 的像素设为 1，其他设为 0
            // 统计非零像素的数量
            int pix_count = cv::countNonZero(mask);
            LOG(ERROR) << "Debug: before ditort timeimage " << pix_count << std::endl;
        }*/

        // 去畸变
        // cv::Mat undistor_TimeImage;
        // cv::undistort(TimeImage, undistor_TimeImage, K_cv, distort_cv);
        // TimeImage = undistor_TimeImage.clone();

        // // 显示 undistor_TimeImage 之前的 TimeImage
        // cv::imshow("Original TimeImage", TimeImage);
        // cv::waitKey(0);  // 按任意键继续

        // // 显示 undistor_TimeImage
        // cv::imshow("Undistorted TimeImage", undistor_TimeImage);
        // cv::waitKey(0);  // 按任意键继续

        // LOG(ERROR) << "K_cv = " << K_cv << std::endl;
        // LOG(ERROR) << "distort_cv = " << distort_cv << std::endl;
        
        /*{
        cv::threshold(TimeImage, mask, 0, 1, cv::THRESH_BINARY); // 将大于 0 的像素设为 1，其他设为 0
        pix_count = cv::countNonZero(mask);
        LOG(ERROR) << "Debug: after ditort timeimage " << pix_count << std::endl;
        }*/


        // std::fstream output("/home/hao/Desktop/output.txt", std::ios::app | std::ios::out);
        // for(int i = 0;i < TimeImage.rows; i++){
        //     for(int j = 0;j < TimeImage.cols; j++)
        //     {
        //         output << TimeImage.at<double>(i,j) << " ";
        //     }
        //     output << "\n";
        // }
        // output << std::endl;
        // output.close();
    }

    // 删除旧事件数据
    void RemoveOldEvents(double threshold_time) {
        // std::lock_guard<std::mutex> lock(detector_data_mutex);
        while (!event_stream.empty()) {
            dvs_msgs::EventArray& event_array = event_stream.front();

            if(event_array.events.front().ts.toSec() >= threshold_time)
                break;

            double end_event_time = event_array.events.back().ts.toSec();

            if (end_event_time < threshold_time) {
                event_stream.pop_front();
                continue;
            }

            auto& events = event_array.events;
            events.erase(std::remove_if(events.begin(), events.end(), 
                       [threshold_time](const dvs_msgs::Event& e) { return e.ts.toSec() < threshold_time; }), 
                       events.end());

            if (events.empty()) event_stream.pop_front();
        }
    }

    void VisualizeTimeImages(const cv::Mat& TimeImage1, const cv::Mat& TimeImage2) {
        // 确保两个图像的大小相同
        if (TimeImage1.size() != TimeImage2.size()) {
            LOG(ERROR) << "Error: Images must be of the same size" << std::endl;
            return;
        }

        // 归一化 TimeImage1
        cv::Mat normalizedTimeImage1;
        double minVal1, maxVal1;

        cv::minMaxLoc(TimeImage1, &minVal1, &maxVal1); // 获取 TimeImage1 的最小值和最大值
        cv::normalize(TimeImage1, normalizedTimeImage1, 0, 255, cv::NORM_MINMAX); // 将 TimeImage1 归一化到 0-255 范围
        std::cout << "TimeImage1 min = " << minVal1 << ", max = " << maxVal1 << std::endl;

        // 归一化 TimeImage2
        cv::Mat normalizedTimeImage2;
        double minVal2, maxVal2;
        cv::minMaxLoc(TimeImage2, &minVal2, &maxVal2); // 获取 TimeImage2 的最小值和最大值
        cv::normalize(TimeImage2, normalizedTimeImage2, 0, 255, cv::NORM_MINMAX); // 将 TimeImage2 归一化到 0-255 范围
        std::cout << "TimeImage2 min = " << minVal2 << ", max = " << maxVal2 << std::endl;

        // 合并图像到一个图像中
        cv::Mat combinedImage;
        cv::hconcat(normalizedTimeImage1, normalizedTimeImage2, combinedImage);

        // 对 combinedImage 进行二值化处理
        // double thresh = 128;  // 设置阈值，取值范围 0 - 255
        // double maxValue = 255;  // 二值化后的最大值
        // cv::Mat binaryImage;
        // cv::threshold(combinedImage, binaryImage, thresh, maxValue, cv::THRESH_BINARY);

        // 显示图像
        cv::imshow("Time Image 1 (Left) and Time Image 2 (Right)", combinedImage); //binaryImage);

        // 等待按键以关闭窗口
        cv::waitKey(10);
    }

    void VisualizeTimeImages(cv::Mat TimeImage1) {
        // 归一化 TimeImage1
        cv::Mat normalizedTimeImage1;
        double minVal1, maxVal1;

        cv::minMaxLoc(TimeImage1, &minVal1, &maxVal1); // 获取 TimeImage1 的最小值和最大值
        cv::normalize(TimeImage1, TimeImage1, 0, 255, cv::NORM_MINMAX); // 将 TimeImage1 归一化到 0-255 范围
        // std::cout << "TimeImage1 min = " << minVal1 << ", max = " << maxVal1 << std::endl;
        cv::imshow("Event Window", TimeImage1); //binaryImage);

        // 等待按键以关闭窗口
        cv::waitKey(10);
    }


    // void PublishTimeImages(cv::Mat pub_img) {
    //     // 归一化 TimeImage1
    //     cv::Mat normalized_pub_img;
    //     double minVal1, maxVal1;

    //     cv::minMaxLoc(pub_img, &minVal1, &maxVal1); // 获取 TimeImage1 的最小值和最大值
    //     cv::normalize(pub_img, normalized_pub_img, 0, 255, cv::NORM_MINMAX); // 将 TimeImage1 归一化到 0-255 范围

    //     // 转为 sensor_msgs/Image 中
    //     // cv_bridge 是一个将 OpenCV 的 Mat 转换为 ROS 中的 sensor_msgs::Image 的工具
    //     std_msgs::Header header;
    //     header.stamp = ros::Time::now();
    //     header.frame_id = "event";
    //     sensor_msgs::ImagePtr msg = cv_bridge::CvImage(header, "mono8", normalized_pub_img).toImageMsg();

    //     // 你可以在这里发布这个 msg 或者进一步处理它
    //     // 假设你有一个 ROS 发布器：
    //     pub_event_image_.publish(msg);
        
    //     // 如果需要打印最小值和最大值
    //     // std::cout << "TimeImage1 min = " << minVal1 << ", max = " << maxVal1 << std::endl;
    //     ROS_INFO("Publish Time Image");
    //     LOG(ERROR) << "Publish Time Image" << std::endl;
    // }

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

            // 打印光流速度
            // std::string flow = "velocity.x: " + std::to_string(velocity.x) + ", velocity.y: " + std::to_string(velocity.y);
            // cv::Point textOrg(2, 1);  // 文本起始位置
            // int fontFace = cv::FONT_HERSHEY_SIMPLEX;  // 字体类型
            // double fontScale = 0.1;  // 字体大小
            // int thickness = 1;  // 字体厚度
            // cv::Scalar color(0, 0, 0);  // 文字颜色（绿色）
            // cv::putText(color_img_temp, flow, textOrg, fontFace, fontScale, color, thickness);

            // cv::Point2d end_point = point + cv::Point2d(velocity.x, velocity.y);
            // cv::arrowedLine(color_img_temp, point, end_point, cv::Scalar(0, 255, 0), 1, 8, 0, 0.1);
            // end_point.x = std::min(std::max(0.0, end_point.x), static_cast<double>(color_img_temp.cols - 1));
            // end_point.y = std::min(std::max(0.0, end_point.y), static_cast<double>(color_img_temp.rows - 1));
            

            // 从 color_img 中分割 [point - radius, point + radius] 范围大小的局部邻域为flow_patch
            double radius_test = 1.0 * radius;
            // 计算局部邻域的矩形区域
            int x_start = std::max(static_cast<double>(0), point.x - radius_test);
            int y_start = std::max(static_cast<double>(0), point.y - radius_test);
            int x_end = std::min(static_cast<double>(color_img_temp.cols), point.x + radius_test + 1);
            int y_end = std::min(static_cast<double>(color_img_temp.rows), point.y + radius_test + 1);
            cv::Rect roi(x_start, y_start, x_end - x_start, y_end - y_start);
            // cv::Scalar green_color(0, 255, 0);
            // cv::circle(color_img_temp, point, 1, green_color, -1); 
            cv::Mat flow_patch = color_img(roi).clone();

            // 填充 Eigen 矩阵
            // for (int i = 0; i < flow_patch.rows; ++i) {
            //     for (int j = 0; j < flow_patch.cols; ++j) {
            //         LOG(ERROR) << flow_patch.at<double>(i, j);
            //     }
            // }

            
            // cv::Mat flow_backgroud = (flow_patch != cv::Vec3b(0, 0, 255));
            // flow_backgroud.convertTo(flow_backgroud, CV_8U);
            // flow_patch.setTo(cv::Vec3b(255, 255, 255), flow_backgroud);

            // 用掩膜设置 flow_patch 的值
            // flow_patch.setTo(cv::Vec3b(255, 255, 255), flow_backgroud_three_channel);

            // 检查是否仍然报错
            // cv::cvtColor(flow_patch, flow_patch, CV_RGB2GRAY);
            // cv::cvtColor(flow_patch, flow_patch, cv::COLOR_RGB2GRAY);


            // cv::Mat flow_backgroud = (flow_patch != cv::Vec3b(0, 0, 255));
            // flow_backgroud.convertTo(flow_backgroud, CV_8U);  // 转换为单通道 8U 类型
            // 扩展 flow_backgroud 为三通道，填充值为 255 或 0
            // cv::Mat flow_backgroud_three_channel;
            // cv::cvtColor(flow_backgroud, flow_backgroud_three_channel, cv::COLOR_GRAY2BGR);
            // 用 flow_backgroud_three_channel 作为掩膜，设置 flow_patch 中对应区域的颜色
            // flow_patch.setTo(cv::Vec3b(255, 255, 255), flow_backgroud);

            // cv::Mat resized_flow_patch;
            // cv::Mat flow_patch resize 10 * 10
            // cv::resize(flow_patch, resized_flow_patch, target_size);
            // cv::Point2d center;
            // center.x = radius_test; center.y = radius_test;


            // for (int i = 0; i < flow_patch.rows; ++i) {
            //     for (int j = 0; j < flow_patch.cols; ++j) {
            //         uchar pixel_value = flow_patch.at<uchar>(i, j);  // 获取每个像素的灰度值
            //         std::string text = std::to_string(pixel_value);  // 将灰度值转换为字符串
                    
            //         // 在图像上标注灰度值
            //         cv::Point position(i, j);  // 在 (x, y) 位置标注
            //         cv::putText(flow_patch, text, position, font_face, font_scale, color, thickness);
            //     }
            // }

            cv::imwrite("/media/hao/hao2/228/test/lab/flow_img/flowimage_" + std::to_string(img_index) 
            + "_" + std::to_string(i) + "_" + std::to_string(velocity.x) + "_" + std::to_string(velocity.y) + ".png", flow_patch);
        }
        img_index ++;

        // 在图像上绘制光流点和光流分量
        LOG(ERROR) << "visualize points = " << flow_pre_points.size() << std::endl;
        for (size_t i = 0; i < flow_pre_points.size(); ++i) {
            cv::Point2d point = best_inliers[i];
            event_flow_velocity velocity = flow_pre_points[i];

            // 绘制光流点（用绿色的圆圈标出）
            cv::circle(color_img, point, 1, cv::Scalar(0, 255, 0), -1);
            // drawSquareBorder(color_img, point, radius);

            // 绘制光流向量（用绿色的箭头标出）
            // LOG(ERROR) << "flow = [" << velocity.x << ", " << velocity.y << "]" << std::endl;
            // if(abs(velocity.x) + abs(velocity.y) < 5)
            //     continue;

            cv::Point2d end_point = point + cv::Point2d(velocity.x, velocity.y);
            end_point.x = std::min(std::max(0.0, end_point.x), static_cast<double>(color_img.cols - 1));
            end_point.y = std::min(std::max(0.0, end_point.y), static_cast<double>(color_img.rows - 1));
            
            // LOG(ERROR) << "start_point = [" << point.x << ", " << point.y << "]" << std::endl;
            // LOG(ERROR) << "end_point = [" << end_point.x << ", " << end_point.y << "]" << std::endl;
            cv::arrowedLine(color_img, point, end_point, cv::Scalar(0, 255, 0), 2, 8, 0, 0.35);
        }

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


    void PublishTimeImages2(cv::Mat pub_img) {
        // 创建一个与 pub_img 相同尺寸的彩色图像，初始化为黑色
        cv::Mat color_img = cv::Mat::zeros(pub_img.size(), CV_8UC3);
        // cv::Mat color_img = pub_img.clone();
        // 创建掩膜，分别表示大于0、小于0和等于0的区域
        cv::Mat mask_pos = pub_img > 0;  // 大于0的区域
        cv::Mat mask_neg = pub_img < 0;  // 小于0的区域
        cv::Mat mask_zero = pub_img == 0; // 等于0的区域

        // // 将大于0的区域设置为红色 (BGR: 0, 0, 255)
        // color_img.setTo(cv::Vec3b(0, 0, 255), mask_pos);

        // // 将小于0的区域设置为蓝色 (BGR: 255, 0, 0)
        // color_img.setTo(cv::Vec3b(255, 0, 0), mask_neg);

        // // 将等于0的区域设置为黑色 (BGR: 0, 0, 0) -> 已经是默认值
        // color_img.setTo(cv::Vec3b(255, 255, 255), mask_zero);

        std::vector<cv::Mat> channels(3);
        color_img.convertTo(color_img, CV_32F);  // 转换为 float 类型，方便归一化
        cv::split(color_img, channels);
        for (int i = 2; i < 3; i++) 
            cv::normalize(channels[i], channels[i], 225, 255, cv::NORM_MINMAX);
        cv::merge(channels, color_img);
        color_img.convertTo(color_img, CV_8UC3); // 转换回 8 位

        color_img.setTo(cv::Vec3b(255, 0, 0), mask_neg);
        color_img.setTo(cv::Vec3b(255, 255, 255), mask_zero);
        
        // 设置字体参数
        int font_face = cv::FONT_HERSHEY_SIMPLEX;
        double font_scale = 0.3;  // 字体大小
        int thickness = 1;        // 字体厚度
        cv::Scalar color(255, 255, 255);  // 文本颜色：白色


        static long int img_index = 0;
        cv::Size target_size(25, 25);
        for (size_t i = 0; i < best_inliers.size(); ++i) {
            // cv::Mat color_img_temp = color_img.clone();
            cv::Point2d point = best_inliers[i];
            double normal_flow = normal_flow_pre_points[i];                 // 投影长度
            Eigen::Vector2d norm_vec = normal_pre_points[i];                // 单位法线,
            Eigen::Vector2d edge_vec(-norm_vec[1], norm_vec[0]);                // 正常来说应该是边缘,
            Eigen::Vector2d delta_xy = norm_vec * normal_flow * event_t1;     // 法线向量,带大小
            LOG(ERROR) << "normal_vec = " << norm_vec.transpose() << std::endl;
            LOG(ERROR) << "normal_flow = " << delta_xy.transpose() << std::endl;
            LOG(ERROR) << "edge_vec = " << edge_vec.transpose() << std::endl;
            cv::Point2d normal_start_point;
            cv::Point2d normal_end_point;
            normal_start_point.x = std::max(point.x - 1.0 * delta_xy(0), 0.0);
            normal_start_point.y = std::max(point.y - 1.0 * delta_xy(1), 0.0);
            normal_end_point.x = std::min(point.x + 1.0 * delta_xy(0), static_cast<double>(color_img.cols));
            normal_end_point.y = std::min(point.y + 1.0 * delta_xy(1), static_cast<double>(color_img.rows));
            // 画法向
            cv::arrowedLine(color_img, normal_start_point, normal_end_point, cv::Scalar(0, 255, 0), 3, 8, 0, 0.15);
            
            cv::Point2d start_point;
            cv::Point2d end_point;
            // cv::Point2d start_point(point.x - 2.0 * edge_vec(0), point.y - 2.0 * edge_vec(1));
            // cv::Point2d end_point(point.x + 2.0 * edge_vec(0), point.y + 2.0 * edge_vec(1));
            start_point.x = std::max(point.x - 10.0 * edge_vec(0), 0.0);
            start_point.y = std::max(point.y - 10.0 * edge_vec(1), 0.0);
            end_point.x = std::min(point.x + 10.0 * edge_vec(0), static_cast<double>(color_img.cols));
            end_point.y = std::min(point.y + 10.0 * edge_vec(1), static_cast<double>(color_img.rows));
            // 画边缘
            cv::arrowedLine(color_img, start_point, end_point, cv::Scalar(255, 0, 0), 2, 8, 0, 0.0);
            // 画点
            cv::circle(color_img, point, 1, cv::Scalar(0, 255, 0), -1);
            drawSquareBorder(color_img, point, radius);

            // 从 color_img 中分割 [point - radius, point + radius] 范围大小的局部邻域为flow_patch
            double radius_test = 3.0 * radius;
            // 计算局部邻域的矩形区域
            int x_start = std::max(static_cast<double>(0), point.x - radius_test);
            int y_start = std::max(static_cast<double>(0), point.y - radius_test);
            int x_end = std::min(static_cast<double>(color_img.cols), point.x + radius_test + 1);
            int y_end = std::min(static_cast<double>(color_img.rows), point.y + radius_test + 1);
            cv::Rect roi(x_start, y_start, x_end - x_start, y_end - y_start);
            cv::Mat flow_patch = color_img(roi).clone();
            
            cv::imwrite("/media/hao/hao2/228/test/lab/flow_img/flowimage_" + std::to_string(img_index) 
            + "_" + std::to_string(i) + "_" + std::to_string(normal_flow) + ".png", flow_patch);
        }
        img_index ++;

        // 在图像上绘制光流点和光流分量
        /*LOG(ERROR) << "visualize points = " << flow_pre_points.size() << std::endl;
        for (size_t i = 0; i < flow_pre_points.size(); ++i) {
            cv::Point2d point = best_inliers[i];
            event_flow_velocity velocity = flow_pre_points[i];

            // 绘制光流点（用绿色的圆圈标出）
            cv::circle(color_img, point, 1, cv::Scalar(0, 255, 0), -1);
            // drawSquareBorder(color_img, point, radius);

            cv::Point2d end_point = point + cv::Point2d(velocity.x, velocity.y);
            end_point.x = std::min(std::max(0.0, end_point.x), static_cast<double>(color_img.cols - 1));
            end_point.y = std::min(std::max(0.0, end_point.y), static_cast<double>(color_img.rows - 1));
            
            cv::arrowedLine(color_img, point, end_point, cv::Scalar(0, 255, 0), 2, 8, 0, 0.35);
        }*/

        std::string angular_data = "Angular: x=" + std::to_string(twist_.twist.twist.angular.x) +
                            " y=" + std::to_string(twist_.twist.twist.angular.y) +
                            " z=" + std::to_string(twist_.twist.twist.angular.z);
        cv::Point text_position(20, 10); // 选择一个适当的位置
        cv::putText(color_img, angular_data, text_position, cv::FONT_HERSHEY_SIMPLEX, 0.35, cv::Scalar(0, 0, 0), 0.75);
        

        cv::imwrite("/media/hao/hao2/228/test/lab/img/timeimage_" + std::to_string(img_index) + ".png", color_img);
        

        // 转为 sensor_msgs::Image 中
        std_msgs::Header header;
        header.stamp = ros::Time::now();
        header.frame_id = "event";

        // 将 color_img 转换为 ROS 消息
        sensor_msgs::ImagePtr msg = cv_bridge::CvImage(header, "bgr8", color_img).toImageMsg();

        // 你可以在这里发布这个 msg 或者进一步处理它
        pub_event_image_.publish(msg);

        // ROS_INFO("Publish Time Image");
        LOG(ERROR) << "Publish Time Image" << std::endl;
        LOG(ERROR) << "Publish point size = " << best_inliers.size() << std::endl;
    }


    void PubRedBlueEvents(cv::Mat on_events, cv::Mat off_events)
    {
        double max_on, max_off, dummy;
        cv::minMaxLoc(on_events, &dummy, &max_on);
        cv::minMaxLoc(off_events, &dummy, &max_off);
        const double max_abs_val = std::max(max_on, max_off);
        const double scale = 127 / max_abs_val;
        // Image of balance of polarities
        cv::Mat gray_image = cv::Mat(on_events.rows, off_events.cols, CV_8U, cv::Scalar(128));
        gray_image += scale * on_events;
        gray_image -= scale * off_events;

        // Use a colormap
        cv::Mat cm_img;
        // if (event_colormap_idx_ >=0 && event_colormap_idx_ <12)
        {
            // Colormaps from OpenCV
            // cv::applyColorMap(gray_image, cm_img, event_colormap_idx_ );
            cv::applyColorMap(gray_image, cm_img, 6 );
        }
        // else
        // {
        // // Use a custom colormap
        // cv::Mat gray_image3ch;
        // cv::cvtColor(gray_image, gray_image3ch, CV_GRAY2BGR);
        // cv::LUT(gray_image3ch, cmap_, cm_img);
        // }
    }

    double AccumuConstractEventsWindows(double timestamp, cv::Mat& time_img, long int need_events = 1000)
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
                    if(e.ts.toSec() <= timestamp && (!ignore_polarity && e.polarity))
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
                        if(!ignore_polarity && event_it->polarity)
                            time_img.at<double>(event_it->y, event_it->x)++;        // 只使用正极性
                        else
                            time_img.at<double>(event_it->y, event_it->x)++;        // 忽略极性影响
                        
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

    // 压缩时间图像
    void CompressTimeImage(double timestamp) {
        TimeImage1 = cv::Mat::zeros(sensor_height, sensor_width, CV_64FC1);
        TimeImage2 = cv::Mat::zeros(sensor_height, sensor_width, CV_64FC1);

        // TimeImage1_on_event = cv::Mat::zeros(sensor_height, sensor_width, CV_8UC1);
        // TimeImage1_off_event = cv::Mat::zeros(sensor_height, sensor_width, CV_8UC1);

        LOG(ERROR) << "event_dt = " << event_dt << ", event_t1 = " << event_t1 << std::endl; 

        // TimeImage1 = cv::Mat::zeros(sensor_height, sensor_width, CV_8UC1);
        // TimeImage2 = cv::Mat::zeros(sensor_height, sensor_width, CV_8UC1);
        double window1_start = timestamp - event_dt;
        double window1_end = timestamp;

        double window2_start = timestamp - event_t1;
        double window2_end = timestamp;

        // std::cout << std::setprecision(20) << "[" << window1_start << ", " << window1_end << "]" << std::endl;
        // std::cout << std::setprecision(20) << "[" << window2_start << ", " << window2_end << "]" << std::endl;

        t1_image_count = 0;
        t2_image_count = 0;
        CompressTimeImageForWindow(window1_start, window1_end, TimeImage1, t1_image_count);
        CompressTimeImageForWindow(window2_start, window2_end, TimeImage2, t2_image_count);

        RemoveOldEvents(std::max(window1_start - 0.5, 0.0));

        // std::cout << "RemoveOldEvents Done!" << std::endl;

        // assert(!event_stream.empty() && event_stream.front().events.front().ts.toSec() >= window2_start);
        
        // if(false)
        // {
        //     ROS_INFO("Visualize Events");
        //     VisualizeTimeImages(TimeImage1, TimeImage2);
        // }

        /*
        static long int img_index = 0;
        std::string filename = "/media/hao/hao2/228/test/lab/flow_img/";
        double minVal, maxVal;
        cv::Mat normalizedTimeImage1, normalizedTimeImage2;
        cv::minMaxLoc(TimeImage1, &minVal, &maxVal); // 获取 TimeImage1 的最小值和最大值
        cv::normalize(TimeImage1, normalizedTimeImage1, 0, 255, cv::NORM_MINMAX); // 将 TimeImage1 归一化到 0-255 范围
        cv::imwrite(filename + std::to_string(img_index) + "_1.png", normalizedTimeImage1);

        cv::minMaxLoc(TimeImage2, &minVal, &maxVal); // 获取 TimeImage1 的最小值和最大值
        cv::normalize(TimeImage2, normalizedTimeImage2, 0, 255, cv::NORM_MINMAX); // 将 TimeImage1 归一化到 0-255 范围
        cv::imwrite(filename + std::to_string(img_index) + "_2.png", normalizedTimeImage2);
        img_index++;
        */
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
        TimeImage1 = cv::Mat::zeros(sensor_height, sensor_width, CV_64FC1);
        TimeImage2 = cv::Mat::zeros(sensor_height, sensor_width, CV_64FC1);

        // double window1_start = timestamp - event_dt;
        // double window1_end = timestamp;

        // double window2_start = timestamp - event_t1;
        // double window2_end = timestamp;

        // event_dt = 
        // event_t1 = 

        // double window1_start = 0;
        // double window2_start = 0;


        // CompressTimeImageForWindow(window1_start, window1_end, TimeImage1, t1_image_count);
        // CompressTimeImageForWindow(window2_start, window2_end, TimeImage2, t2_image_count);

        // AccumulateEvents(window1_start, timestamp, TimeImage1, t1_image_count);
        // AccumulateEvents(window2_start, timestamp, TimeImage2, t2_image_count);

        // dji
        // t1_image_count = 2000;
        // t2_image_count = 1600;
        
        // dvs
        // t1_image_count = 14000;
        // t2_image_count = 12000;
        
        event_dt = AccumuConstractEventsWindows(timestamp, TimeImage1, t1_image_count);
        event_t1 = AccumuConstractEventsWindows(timestamp, TimeImage2, t2_image_count);
        // AccumulateEvents from [timestamp - event_dt,  timestamp - 0.3 * event_dt]
        // AccumulateEvents(timestamp - event_dt, timestamp - 0.3 * event_dt, TimeImage2, t2_image_count);

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


    double huber_weight(double residual, double delta) {
        if (std::abs(residual) <= delta) {
            return 1.0;  // 对小残差使用二次损失（与普通最小二乘一致）
        } else {
            return delta / std::abs(residual);  // 对大残差使用线性损失，减少权重
        }
    }

    // RANSAC光流计算
    std::fstream output;
    bool CalculateOpFlowRANSAC() {
        int points_num = 15;
        const int max_iterations = 50;
        double min_cov_points = std::numeric_limits<double>::max();
        
        // int iter_count = 0;
        for (int iter = 0; iter < max_iterations; ++iter) {
            std::vector<cv::Point2d> selected_points;
            RandomlySelectPoints(TimeImage1, selected_points, points_num);
            // std::cout << "points_num = " << points_num << std::endl;
            // std::cout << "selected_points.size = " << selected_points.size() 
            //         << " points_num = " << points_num << std::endl;
            if(selected_points.size() < points_num)
            {
                // std::cout << "skip" << std::endl;
                // LOG(WARNING) << "selected points num is not valid!" << std::endl;
                LOG(ERROR) << "selected points num is not valid!" << std::endl;
                // return false;
                continue;
            }
            // assert(points_num == selected_points.size());

            // Eigen::Matrix<double, Eigen::Dynamic, 2> grad_A;

            // grad_A.resize(points_num, 2);  // 在运行时设置矩阵大小
            // Eigen::Matrix<double, Eigen::Dynamic, 1> grad_b;
            // grad_b.resize(points_num, 1);  // 在运行时设置矩阵大小

            // 定义 grad_A 和 grad_b
            // Eigen::Matrix<double, Eigen::Dynamic, 2> grad_A(points_num, 2); // m x 2 矩阵
            // Eigen::Matrix<double, Eigen::Dynamic, 1> grad_b(points_num, 1); // m x 1 向量

            std::vector<Eigen::Vector2d> flow_temp_vec;
            Eigen::MatrixXd grad_A(points_num, 2);
            Eigen::MatrixXd grad_b(points_num, 1);
            
            // std::cout << "Traversal selected_points: " << selected_points.size() << std::endl;
            int rows = 0;
            int zeros_count = 0;
            for (const auto& pt : selected_points) {
                int x = static_cast<int>(pt.x);
                int y = static_cast<int>(pt.y);

                double grad_x = 0;
                double grad_y = 0;

                // std::cout << "p = " << TimeImage1.at<double>(y, x) << ", px = " << TimeImage1.at<double>(y , x - 1)
                //           << ", py = " << TimeImage1.at<double>(y - 1, x) << std::endl;

                // grad_x += (TimeImage1.at<double>(y, x) - TimeImage1.at<double>(y , x - 1));
                // grad_y += (TimeImage1.at<double>(y, x) - TimeImage1.at<double>(y - 1, x));

                // output.open("/home/hao/Desktop/output.txt", std::ios::app | std::ios::out);
                for (int i = -radius; i <= radius; ++i) {
                    for (int j = -radius; j <= radius; ++j) {
                        // 确保不越界
                        if (x + j > 0 && x + j < TimeImage1.cols - 1 && y + i > 0 && y + i < TimeImage1.rows - 1) {
                            // 输出像素值
                            // std::cout << "Pixel value at (" << x + j << ", " << y + i << ") = " 
                            //         << TimeImage1.at<double>(y + i, x + j) << std::endl;
                            // output << "Pixel value at (" << x + j << ", " << y + i << ") = " 
                            //         << TimeImage1.at<double>(y + i, x + j) << std::endl;
                            // 水平方向梯度
                            grad_x += (TimeImage1.at<double>(y + i, x + j + 1) - TimeImage1.at<double>(y + i, x + j - 1)) / 2;
                            // grad_x += (TimeImage1.at<double>(y + i, x + j) - TimeImage1.at<double>(y + i, x + j - 1));
                            // output << "delta_x = " << (TimeImage1.at<double>(y + i, x + j + 1) - TimeImage1.at<double>(y + i, x + j)) << std::endl;
                            // 垂直方向梯度
                            grad_y += (TimeImage1.at<double>(y + i + 1, x + j) - TimeImage1.at<double>(y + i - 1, x + j)) / 2;
                            // grad_y += (TimeImage1.at<double>(y + i, x + j) - TimeImage1.at<double>(y + i - 1, x + j));
                            // output << "delta_y = " << (TimeImage1.at<double>(y + i + 1, x + j) - TimeImage1.at<double>(y + i - 1, x + j)) << std::endl;
                        }
                    }
                    // std::cout << std::endl;  // 分隔每一行像素输出
                }

                // double grad_x = 0.0, grad_y = 0.0;
                // int radius = 2;  // 5x5 区域的半径
                // for (int i = -radius; i <= radius; ++i) {
                //     for (int j = -radius; j <= radius; ++j) {
                //         // 确保不越界
                //         if (x + j > 0 && x + j < TimeImage1.cols - 1 && y + i > 0 && y + i < TimeImage1.rows - 1) {
                //             // 计算水平方向梯度
                //             grad_x += (TimeImage1.at<double>(y + i, x + j + 1) - TimeImage1.at<double>(y + i, x + j - 1)) / 2;
                //             std::cout << "grad_x_i = "
                //                     << TimeImage1.at<double>(y + i, x + j + 1) - TimeImage1.at<double>(y + i, x + j - 1) << std::endl;
                            
                //             // 计算垂直方向梯度
                //             grad_y += (TimeImage1.at<double>(y + i + 1, x + j) - TimeImage1.at<double>(y + i - 1, x + j)) / 2;
                //             std::cout << "grad_y_i = "
                //                     << TimeImage1.at<double>(y + i + 1, x + j) - TimeImage1.at<double>(y + i - 1, x + j) << std::endl;
                //         }
                //     }
                // }

                // std::cout << "grad = [" << grad_x << "," << grad_y << "]" << std::endl;
                // assert(TimeImage1.at<double>(y, x) > 0 && "pixel is empty");
 

                // output.close();

                // 计算5x5模板的平均梯度
                // grad_x /= (5 * 5);  // 使用25个像素的平均
                // grad_y /= (5 * 5);

                // std::cout << "grad = [" << grad_x << "," << grad_y << "]" << std::endl;

                // double grad_x = (TimeImage1.at<double>(y, x) - TimeImage1.at<double>(y, x - 1));
                // double grad_y = (TimeImage1.at<double>(y, x) - TimeImage1.at<double>(y - 1, x));
                // double grad_b_i = (TimeImage2.at<double>(y, x) - TimeImage1.at<double>(y, x)) / event_t1;   // 1

                double grad_b_i = TimeImage2.at<double>(y, x) / event_t1;

                // 1 * 2 
                // grad_A.block<rows,0>(1,2) << grad_x, grad_y;
                // grad_b.block<rows,0>(1,1) << grad_b_i;

                // if(grad_x == 0 || grad_y == 0)  // 存在缺失参数,丢到核空间中
                // {
                //     grad_x = grad_y = 0;
                //     grad_b_i = 0;
                //     zeros_count++;
                // }

                if(grad_x == 0 && grad_y == 0)
                {
                    grad_b_i = 0;
                    zeros_count++;
                }
                else if(grad_b_i == 0)
                {
                    grad_x = grad_y = 0;
                    zeros_count++;
                }

                grad_A.block(rows, 0, 1, 2) << grad_x, grad_y;
                grad_b.block(rows, 0, 1, 1) << grad_b_i;
                // std::cout << "rows = " << rows << std::endl;
                rows ++;
            }

            // std::cout << "rows = " << rows << std::endl;
            // LOG(INFO) << "zeros_count = " << zeros_count << std::endl;
            LOG(ERROR) << "zeros_count = " << zeros_count << std::endl;

            // 本次迭代失败
            if (rows - zeros_count < 2)
                continue;

            // std::cout << "A = \n" 
            //           << grad_A << std::endl;

            // std::cout << "b = \n" 
            //           << grad_b << std::endl;

            // LOG(INFO) << "best flow : A * flow = b";
            // LOG(INFO) << "A = \n" 
            //           << grad_A << std::endl;

            // LOG(INFO) << "b = \n" 
            //           << grad_b << std::endl;            

            LOG(ERROR) << "best flow : A * flow = b";
            LOG(ERROR) << "A = \n" 
                      << grad_A << std::endl;

            LOG(ERROR) << "b = \n" 
                      << grad_b << std::endl;     

            // Eigen::Vector2d flow = grad_A.colPivHouseholderQr().solve(grad_b); 
            Eigen::VectorXd flow = grad_A.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(grad_b);

        {
            // Eigen::JacobiSVD<Eigen::MatrixXd> svd(grad_A, Eigen::ComputeThinU | Eigen::ComputeThinV);
            // double cond = svd.singularValues()(0) / svd.singularValues().tail(1)(0);
            // LOG(INFO) << "Cond = " << cond << std::endl;
            int rank = grad_A.fullPivLu().rank();
            LOG(ERROR) << "Rank = " << rank << std::endl;

            Eigen::VectorXd residuals = grad_A * flow - grad_b;
            LOG(ERROR) << "First Residuals = " << residuals.transpose() << std::endl;
            Eigen::VectorXd weights = (residuals.array().abs() + 1e-6).inverse();
            LOG(ERROR) << "weight = " << weights.transpose() << std::endl;
            Eigen::MatrixXd weighted_grad_A = grad_A.array().colwise() * weights.array();  // 按列调整 A
            LOG(ERROR) << "weight_A = " << weighted_grad_A << std::endl;
            Eigen::VectorXd weighted_grad_b = grad_b.array() * weights.array();  // 调整 b
            LOG(ERROR) << "weight_B = " << weighted_grad_b << std::endl;
            // 再次进行加权最小二乘法求解
            // Eigen::Vector2d flow_weighted = weighted_grad_A.colPivHouseholderQr().solve(weighted_grad_b);
            // Eigen::VectorXd flow_weighted = weighted_grad_A.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(weighted_grad_b);
            // Eigen::VectorXd flow_weighted;
            Eigen::VectorXd flow_weighted = weighted_grad_A.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(weighted_grad_b);

            event_flow_velocity flow_velocity_temp(flow_weighted);
            Eigen::VectorXd weight_residuals = weighted_grad_A * flow_weighted - weighted_grad_b;
            double cov_points = (weight_residuals).norm(); 
            LOG(ERROR) << "Second Residuals = " << weight_residuals.transpose() << std::endl;
            LOG(ERROR) << "best flow = " << flow_weighted.transpose() << std::endl;

            if (cov_points < min_cov_points) {
                min_cov_points = cov_points;
                best_inliers.clear();
                best_inliers = selected_points;
                flow_velocity = flow_velocity_temp;
                
            }
        }

    /*{
            // 二次优化
            Eigen::VectorXd residuals = grad_A * flow - grad_b;
            LOG(INFO) << "First Residuals = " << residuals.transpose() << std::endl;
            // 3. 设置剔除离群点的阈值（根据具体情况调整）
            double threshold = 0.1;
            std::vector<int> inliers;  // 存储误差较小的内点索引

            for (int i = 0; i < residuals.size(); ++i) {
                if (std::abs(residuals(i)) < threshold) {
                    inliers.push_back(i);  // 选择残差小于阈值的点
                }
            }
            // 4. 使用内点重新构建 grad_A 和 grad_b
            int new_size = inliers.size();
            Eigen::MatrixXd grad_A_selected(new_size, grad_A.cols());
            Eigen::VectorXd grad_b_selected(new_size);
            for (int i = 0; i < new_size; ++i) {
                grad_A_selected.row(i) = grad_A.row(inliers[i]);
                grad_b_selected(i) = grad_b(inliers[i]);
            }
            // 5. 再次进行最小二乘优化，得到新的光流
            Eigen::Vector2d refined_flow = grad_A_selected.colPivHouseholderQr().solve(grad_b_selected);
            Eigen::VectorXd refined_residuals = grad_A_selected * refined_flow - grad_b_selected;
            LOG(INFO) << "Second Residuals = " << refined_residuals.transpose() << std::endl;

            // flow /= (event_dt - event_t1 + 1e-3);
            event_flow_velocity flow_velocity_temp(refined_flow);
            double cov_points = (grad_A_selected * refined_flow - grad_b_selected).norm(); 

            LOG(INFO) << "best flow = " << refined_flow.transpose() << std::endl;

            std::vector<cv::Point2d> inliers_points;
            for (int i : inliers) {
                inliers_points.push_back(selected_points[i]);
            }

            if (cov_points < min_cov_points) {
                min_cov_points = cov_points;
                best_inliers = inliers_points;
                flow_velocity = flow_velocity_temp;
            }
        }*/

            // // flow /= (event_dt - event_t1 + 1e-3);
            // event_flow_velocity flow_velocity_temp(flow);
            // double cov_points = (grad_A * flow - grad_b).norm(); 

            // LOG(INFO) << "best flow = " << flow.transpose() << std::endl;

            // if (cov_points < min_cov_points) {
            //     min_cov_points = cov_points;
            //     best_inliers = selected_points;
            //     flow_velocity = flow_velocity_temp;
            // }
        }

        // 投影到归一化场景
        // flow_velocity.x /= K(0,0);
        // flow_velocity.y /= K(1,1);       

        LOG(INFO) << "RANSAC OpFlow: \n"
                  << "\t min_cov: " << min_cov_points << std::endl;

        LOG(INFO) << "Best Flow: \n" 
                  << "[" << flow_velocity.x << ", "<< flow_velocity.y << "]" 
                  << std::endl;  

        // std::cout << "RANSAC OpFlow: \n"
        //           << "\t min_cov: " << min_cov_points << std::endl;

        // std::cout << "Best Flow: \n" 
        //           << "[" << flow_velocity.x << ", "<< flow_velocity.y << "]" 
        //           << std::endl;  

        if(best_inliers.size() < 5)
        {
            LOG(WARNING) << "inliers is not enough!" << std::endl;
            return false; 
        }

        return true;       
    }

    void DVS_activity_filter()
    {

    }

    void OritationFilter(std::vector<double>& flow_norm_vec,
             std::vector<Eigen::Vector2d>& flow_vec, std::vector<cv::Point2d>& selected_points_temp)
    {
        assert(flow_norm_vec.size() == flow_vec.size());

        double norm_mean = 0.0;
        double angle_mean = 0.0;

        // 计算角度
        std::vector<double> flow_angle_vec;
        for(size_t i = 0; i < flow_vec.size(); i++)
        {
            Eigen::Vector2d f = flow_vec[i];
            if(f(0) == 0){
                flow_angle_vec.push_back(((f(0) > 0)? 0: std::copysign(M_PI, f(1))));
                continue;
            }
            flow_angle_vec.push_back(std::atan2(f(1), f(0)));

            norm_mean+= flow_norm_vec[i];
            angle_mean+= flow_angle_vec[i];
        }
        norm_mean /= flow_vec.size();
        angle_mean /= flow_vec.size();

        double norm_std = 0.0;
        double angle_std = 0.0;
        for(size_t i = 0; i < flow_vec.size(); i++)
        {
            norm_std = std::pow(flow_norm_vec[i] - norm_mean, 2);
            angle_std = std::pow(flow_angle_vec[i] - angle_mean, 2);
        }
        norm_std /= flow_vec.size();
        angle_std /= flow_vec.size();
        LOG(ERROR) << "norm_mean = " << norm_mean << ", norm_std = " << norm_std << std::endl;
        LOG(ERROR) << "angle_mean = " << angle_mean << ", angle_std = " << angle_std << std::endl;

        for(size_t i = 0; i < flow_vec.size(); )
        {
            if((std::abs(flow_norm_vec[i]) > norm_mean + norm_std))
            //  || (std::abs(flow_angle_vec[i]) > angle_mean + angle_std))
            {
                flow_vec.erase(flow_vec.begin() + i);
                flow_norm_vec.erase(flow_norm_vec.begin() + i);
                flow_angle_vec.erase(flow_angle_vec.begin() + i);
                selected_points_temp.erase(selected_points_temp.begin() + i);
            }
            else
            {
                i++;
            }   
        }

        // 保存到csv文件中
        static bool clear_angle_norm_file = false;
        std::fstream angle_norm_file;
        if(!clear_angle_norm_file)
        {
            angle_norm_file.open("/home/hao/Desktop/norm.csv", std::ios::out | std::ios::trunc);
            angle_norm_file.clear();
            angle_norm_file.close();
            clear_angle_norm_file = true;
        }
        angle_norm_file.open("/home/hao/Desktop/norm.csv", std::ios::out | std::ios::app);
        for(size_t i = 0; i < flow_angle_vec.size(); i++)
        {
            angle_norm_file << flow_angle_vec[i] << ", " << flow_norm_vec[i] << std::endl;
        }
        angle_norm_file << "----" << std::endl;
        angle_norm_file.close();
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

    // RANSAC光流计算
    // std::fstream output;
    // 探讨法向光流
    bool CalculateOpFlowPrepointRANSAC() {
        int points_num = 120;
        const int max_iterations = 50;
        double min_cov_points = std::numeric_limits<double>::max();

        // 中值滤波

        // 进行 5x5 中值滤波
        // cv::medianBlur(TimeImage1, TimeImage1, 5);
        // cv::medianBlur(TimeImage2, TimeImage2, 5);
        LOG(ERROR) << "event image check: median_radius = " << median_radius << std::endl; 
        TimeImage1.convertTo(TimeImage1, CV_8U);
        if (median_radius > 0)
            cv::medianBlur(TimeImage1, TimeImage1, median_radius);

        TimeImage2.convertTo(TimeImage2, CV_8U);
        if (median_radius > 0)
            cv::medianBlur(TimeImage2, TimeImage2, median_radius);
        TimeImage1.convertTo(TimeImage1, CV_64FC1);
        TimeImage2.convertTo(TimeImage2, CV_64FC1);

        /*
        if (median_radius > 0)
        {
            // 双边滤波参数设置
            //int d = 9;                // 邻域直径
            double sigmaColor = 20;      // 颜色空间的标准差（影响灰度值差异）
            double sigmaSpace = 50;      // 坐标空间的标准差（影响空间距离
            cv::Mat TimeImageTemp;
            cv::bilateralFilter(TimeImage1, TimeImageTemp, median_radius, sigmaColor, sigmaSpace);
            TimeImage1 = TimeImageTemp.clone();
            cv::bilateralFilter(TimeImage2, TimeImageTemp, median_radius, sigmaColor, sigmaSpace);
            TimeImage2 = TimeImageTemp.clone();
        }*/
        if (use_gauss)
            cv::GaussianBlur(TimeImage1, TimeImage1, cv::Size(5, 5), 0.75);
        if (use_gauss)  
            cv::GaussianBlur(TimeImage2, TimeImage2, cv::Size(5, 5), 0.75);


        // cv::Mat kernel = cv::Mat::ones(5, 5, CV_8U);
        // cv::morphologyEx(TimeImage1, TimeImage1, cv::MORPH_CLOSE, kernel);
        // cv::morphologyEx(TimeImage2, TimeImage2, cv::MORPH_CLOSE, kernel);  


        // 按位与操作
        cv::Mat result;
        cv::bitwise_and(TimeImage1, TimeImage2, result);

        // TimeImage1 = TimeImage1(result);
        // TimeImage2 = TimeImage2(result);

        TimeImage1.setTo(0, result == 0);
        TimeImage2.setTo(0, result == 0);

        // TimeImage1 和 TimeImage2 是两个图像，取其中的与，统计非零的像素数量
        std::vector<cv::Point2d> nonZeroPoints;  
        cv::findNonZero(result, nonZeroPoints);
        int nonZeroResult = nonZeroPoints.size();
        LOG(ERROR) << nonZeroResult << " pixels is same for motion " << std::endl; 

        LOG(ERROR) << "filter_num = " << filter_num << std::endl;
 
        // 筛选出事件数 > 1 的点
        // 为了筛选孤立点
        {
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
                else
                {
                    TimeImage1.at<float>(pt.y, pt.x) = 0;
                    TimeImage2.at<float>(pt.y, pt.x) = 0;
                }
            }
            LOG(ERROR) << "Choose " << filteredPoints.size() << " pixels for Optical Flow " << std::endl; 
            nonZeroPoints = filteredPoints;
        }

        if(nonZeroResult < points_num)
        {
            LOG(ERROR) << "Event points are not enough, size = " << nonZeroPoints.size() 
            << "need for = " << points_num << std::endl;
            return false;
        }    

        // std::unordered_map<int, std::unordered_map<int, cv::Point2d>> grid;
        // std::vector<cv::Point2d> filteredPoints;
        for (int iter = 0; iter < max_iterations; ++iter) {
            std::vector<cv::Point2d> selected_points;
            std::vector<cv::Point2d> selected_points_temp;
            std::vector<event_flow_velocity> flow_temp_vec;

            gridBasedSampling(nonZeroPoints, selected_points, 2 * radius + 1, points_num);


            // 2-28 修改: 边缘光流
            std::vector<double> normal_flow_temp_vec;
            std::vector<Eigen::Vector2d> normal_temp_vec;

            std::shuffle(nonZeroPoints.begin(), nonZeroPoints.end(), std::mt19937{std::random_device{}()});
            // selected_points.assign(nonZeroPoints.begin(), nonZeroPoints.begin()+ points_num);


            int rows = 0;
            int zeros_count = 0;
            double cov_points = 0;
            for (const auto& pt : selected_points) {
                int x = static_cast<int>(pt.x);
                int y = static_cast<int>(pt.y);

                double grad_x = 0;
                double grad_y = 0;

                // 初始化光流分量
                double u = 0.0;
                double v = 0.0;

                // 构建用于计算光流的矩阵
                // double Ix = 0.0, Iy = 0.0, It = 0.0;
                // double Ixx = 0.0, Ixy = 0.0, Iyy = 0.0, Ixt = 0.0, Iyt = 0.0;
                // double It_current = 0.0;

                double Grad_x = 0.0, Grad_y = 0.0, Grad_t = 0.0;
                double Grad_xx = 0.0, Grad_xy = 0.0, Grad_yy = 0.0, Grad_xt = 0.0, Grad_yt = 0.0;
                double It_current = 0.0;


                // 遍历局部窗口并计算梯度
                bool skip_cur_point = false;


                // std::vector<double> Grad_x_vec;
                // std::vector<double> Grad_y_vec;
                // std::vector<double> Grad_i_vec;

                // DEBUG: 检查TimeImage1像素邻域
                int patch_rows = 0;
                int patch_cols = 0;
                
                std::vector<double> patch_pixels;
                for (int i = -radius; i <= radius; i++) {
           
                    int pixel_y = y + i;
                    if(!(pixel_y > -1 && pixel_y < TimeImage1.rows))
                        continue;
                    patch_cols = 0;
                    for (int j = -radius; j <= radius; j++) {
                        int pixel_x = x + j;
                        
                        if (!(pixel_x > -1 && pixel_x < TimeImage1.cols))
                            continue;
                        patch_cols++;
                        double pixel = TimeImage1.at<double>(pixel_y, pixel_x); 
                        patch_pixels.push_back(pixel); 
                    }

                    patch_rows++;
                }
                LOG(ERROR) << "Patch size:[" << patch_rows << ", " << patch_cols << "]" << std::endl;
                LOG(ERROR) << "Pixel size:" << patch_pixels.size() << std::endl;
                assert(patch_pixels.size() == patch_rows * patch_cols);
                Eigen::MatrixXd patch(patch_rows, patch_cols);
                
                // 将 vector 数据填充到 Eigen::MatrixXd
                for (int i = 0; i < patch_rows; i++) {
                    for (int j = 0; j < patch_cols; j++) {
                        patch(i, j) = patch_pixels[i * patch_cols + j];
                    }
                }
                LOG(ERROR) << "patch = " << patch << std::endl;



                // DEBUG: 检查TimeImage2像素邻域
                patch_rows = 0;
                patch_cols = 0;
                patch_pixels.clear();
                for (int i = -radius; i <= radius; i++) {
           
                    int pixel_y = y + i;
                    if(!(pixel_y > -1 && pixel_y < TimeImage2.rows))
                        continue;
                    patch_cols = 0;
                    for (int j = -radius; j <= radius; j++) {
                        int pixel_x = x + j;
                        
                        if (!(pixel_x > -1 && pixel_x < TimeImage2.cols))
                            continue;
                        patch_cols++;
                        double pixel = TimeImage2.at<double>(pixel_y, pixel_x); 
                        patch_pixels.push_back(pixel); 
                    }

                    patch_rows++;
                }
                LOG(ERROR) << "Patch size:[" << patch_rows << ", " << patch_cols << "]" << std::endl;
                LOG(ERROR) << "Pixel size:" << patch_pixels.size() << std::endl;
                assert(patch_pixels.size() == patch_rows * patch_cols);
                Eigen::MatrixXd patch2(patch_rows, patch_cols);
                
                // 将 vector 数据填充到 Eigen::MatrixXd
                for (int i = 0; i < patch_rows; i++) {
                    for (int j = 0; j < patch_cols; j++) {
                        patch2(i, j) = patch_pixels[i * patch_cols + j];
                    }
                }
                LOG(ERROR) << "patch2 = " << patch2 << std::endl;



                for (int i = -radius + 1; i < radius; i++) {
                    for (int j = -radius + 1; j < radius; j++) {
                        int pixel_x = x + j;
                        int pixel_y = y + i;
                        // LOG(ERROR) << "i = " << i << " j = " << j << std::endl;

                        // 确保不越界
                        if (pixel_x > 0 && pixel_x < TimeImage1.cols && pixel_y > 0 && pixel_y < TimeImage1.rows) {
                            // 计算空间梯度（水平和垂直）
                            double Ix_left = TimeImage1.at<double>(pixel_y, pixel_x - 1);
                            // double Ix_right = TimeImage1.at<double>(pixel_y, pixel_x + 1);
                            double Iy_top = TimeImage1.at<double>(pixel_y - 1, pixel_x);
                            // double Iy_bottom = TimeImage1.at<double>(pixel_y + 1, pixel_x);

                            // // 2-15 修改
                            double Ix_right = TimeImage1.at<double>(pixel_y, pixel_x);
                            double Iy_bottom = TimeImage1.at<double>(pixel_y, pixel_x);   


                            // 2-25 修改 Sobel计算
                            /*{
                                double I_top = TimeImage1.at<double>(pixel_y - 1, pixel_x);
                                double Ic = TimeImage1.at<double>(pixel_y, pixel_x);
                                double I_bottom = TimeImage1.at<double>(pixel_y + 1, pixel_x);
                                
                                double I_left_top = TimeImage1.at<double>(pixel_y - 1, pixel_x - 1);
                                double I_left = TimeImage1.at<double>(pixel_y, pixel_x - 1);
                                double I_left_bottom = TimeImage1.at<double>(pixel_y + 1, pixel_x - 1);

                                double I_right_top = TimeImage1.at<double>(pixel_y - 1, pixel_x + 1);
                                double I_right = TimeImage1.at<double>(pixel_y, pixel_x + 1);
                                double I_right_bottom = TimeImage1.at<double>(pixel_y + 1, pixel_x + 1); 

                                double delta_x = - I_left_top -2 * I_left - I_left_bottom + I_right_top + 2 * I_right + I_right_bottom;
                                double delta_y = - I_left_top -2 * I_top - I_right_top + I_left_bottom + 2 * I_bottom + I_right_bottom;
                                double delta_i = TimeImage2.at<double>(pixel_y, pixel_x) / event_t1;
                                // Grad_x += delta_x / 8.0;
                                // Grad_y += delta_y / 8.0;

                                // 最小二乘法计算
                                Grad_x_vec.push_back(delta_x / 8.0);
                                Grad_y_vec.push_back(delta_y / 8.0);
                                Grad_i_vec.push_back(delta_i);
                            }*/

                            // 计算梯度
                            // Ix += (Ix_right - Ix_left) / 2.0;
                            // Iy += (Iy_bottom - Iy_top) / 2.0;
                            // 2-8修改:
                            // LOG(ERROR) << "delta_x = " << (Ix_right - Ix_left);
                            // LOG(ERROR) << "delta_y = " << (Iy_bottom - Iy_top);
                            Grad_x += (Ix_right - Ix_left);
                            Grad_y += (Iy_bottom - Iy_top);
                            // LOG(ERROR) << "Grad_x = " << Grad_x;
                            // LOG(ERROR) << "Grad_y = " << Grad_y;

                            // 计算时间梯度
                            // double It_current = TimeImage2.at<double>(y, x) / event_t1;
                            // It_current = TimeImage2.at<double>(y, x) / event_t1;
                            It_current = TimeImage2.at<double>(pixel_y, pixel_x); //  / event_t1;
                            // LOG(ERROR) << "It_current = " << It_current;
                            // double It_current = TimeImage2.at<double>(y, x);
                            // 2-8 测试
                            // double It_current = TimeImage2.at<double>(y, x) / (event_t1 / event_dt);
                            Grad_t += It_current;
                            // LOG(ERROR) << "delta_I = " << It_current;
                        }
                        else            // 取消边缘点的光流的计算
                        {
                            // LOG(ERROR) << "skip cur point" << std::endl;
                            skip_cur_point = true;
                        }
                        
                        // size_index++;
                    }
                }

                // assert(Grad_x_vec.size() == Grad_y_vec.size() == Grad_i_vec.size() && "size is not same!"); 
                // int grad_size = Grad_x_vec.size();
                Grad_t /= event_t1;
                Grad_xx += Grad_x * Grad_x;
                Grad_xy += Grad_x * Grad_y;
                Grad_yy += Grad_y * Grad_y;
                Grad_xt += Grad_x * Grad_t;
                Grad_yt += Grad_y * Grad_t;

                LOG(ERROR) << "dim = " << Grad_xx * Grad_yy - Grad_xy * Grad_xy << " required = " << 1 << std::endl;
                LOG(ERROR) << "Grad_xx = " <<  Grad_xx;
                LOG(ERROR) << "Grad_xy = " <<  Grad_xy;
                LOG(ERROR) << "Grad_yy = " <<  Grad_yy;
                LOG(ERROR) << "Grad_t = " <<  Grad_yy;
                // if(Grad_xx * Grad_yy - Grad_xy * Grad_xy < 1)
                //     continue;

                // LOG(ERROR) << "final Ix = " << Grad_x;
                // LOG(ERROR) << "final Iy = " << Grad_y;
                // LOG(ERROR) << "final It = " << Grad_t;

                // LOG(ERROR) << "first Grid_I = \n" << Grad_xx << " " << Grad_xy << "\n" << Grad_xy << " " << Grad_yy << std::endl;
                // LOG(ERROR) << "first Delta_I = \n" << Grad_xt << "\n" << Grad_yt << std::endl;

                if(skip_cur_point)
                // {
                    // event_flow_velocity flow_temp(0, 0);
                    // flow_temp_vec.push_back(flow_temp);
                    continue;
                // }

                // LOG(ERROR) << "Grid_I = \n" << Grad_xx << " " << Grad_xy << "\n" << Grad_xy << " " << Grad_yy << std::endl;
                
                // if(Grad_xx * Grad_yy == Grad_xy * Grad_xy)
                //     continue;

                // 计算光流
                // 计算伪逆 (Moore-Penrose Pseudoinverse)
                // Eigen::Matrix2d A;
                // A << Grad_xx, Grad_xy, Grad_xy, Grad_yy;
                // Eigen::Vector2d b;
                // b << Grad_xt, Grad_yt;

                // 计算法向光流
                Eigen::Vector2d Grad_I; // 本身就是边缘的法向量
                Grad_I << Grad_x , Grad_y;
                LOG(ERROR) << "Grad_I = " << Grad_I << std::endl;
                LOG(ERROR) << "Grad_t = " << Grad_t << std::endl;
                LOG(ERROR) << "event_dt = " << event_dt << std::endl;
                double Grad_I_norm = Grad_I.norm();
                if(Grad_I_norm == 0)       // 不存在梯度
                    continue;

                LOG(ERROR) << "L2 norm for Grad_I = " << Grad_I_norm << std::endl;
                // 法向光流
                // double flow_strength =  - Grad_t * Grad_I / Grad_I_norm;
                // LOG(ERROR) << "flow_strength = " << flow_strength << std::endl;
                // Grad_I /= Grad_I_norm;
                // Eigen::Vector2d flow_eigen = flow_strength * Grad_I;
                // Grad_t = Grad_t / event_t1; // 在上面已经加入
                // 沿梯度方向的光流,
                Eigen::Vector2d flow_eigen = - Grad_t * Grad_I / (Grad_I_norm * Grad_I_norm);
                LOG(ERROR) << "Grad_I_norm vector = " << Grad_I / Grad_I_norm << std::endl;
                u = flow_eigen[0];
                v = flow_eigen[1];  
                LOG(ERROR) << "flow = " << u << " " << v << std::endl;
            

                // 投影向量的法向光流 (一维)
                double normal_flow = flow_eigen.norm();             // 法向光流大小
                Eigen::Vector2d normal_vec = Grad_I / Grad_I_norm;  // 法向光流方向

                // 抑制小光流
                if(normal_flow < 1) 
                    continue;

                normal_flow_temp_vec.push_back(normal_flow);
                normal_temp_vec.push_back(normal_vec);

                LOG(ERROR) << "normal_flow = " << normal_flow << std::endl;
                LOG(ERROR) << "normal_vec = " << normal_vec.transpose() << std::endl;   
                // 奇异情况
                // 伪逆
                // Eigen::JacobiSVD<Eigen::MatrixXd> svd(A, Eigen::ComputeThinU | Eigen::ComputeThinV);
                // Eigen::Matrix2d A_pseudo_inverse = svd.solve(Eigen::MatrixXd::Identity(2,2));  
                // 计算解 x
                // Eigen::Vector2d flow_eigen = A_pseudo_inverse * b;



                // 最小二乘求解
                {
                    // Eigen::Vector2d flow_eigen = (A.transpose() * A).inverse() * (A.transpose() * b);   
                    // u = flow_eigen[0];
                    // v = flow_eigen[1];
                    // 在像素系下计算光流误差
                    event_flow_velocity flow(u, v);
                    if(!(u == 0 && v == 0))
                    {
                        flow_temp_vec.push_back(flow);
                        selected_points_temp.push_back(pt);
                    }
                    // double It_predicted = Grad_x * u + Grad_y * v;
                    // 法向光流残差
                    double It_predicted = (Grad_I.transpose() * flow_eigen + Grad_t); 
                    LOG(ERROR) << "It_predicted = " << It_predicted << std::endl;
                    
                    cov_points += std::abs(It_predicted - Grad_t);

                    // LOG(ERROR) << "Grid_I = \n" << Grad_xx << " " << Grad_xy << "\n" << Grad_xy << " " << Grad_yy << std::endl;
                    // LOG(ERROR) << "Delta_I = \n" << Grad_xt << "\n" << Grad_yt << std::endl;
                    // LOG(ERROR) << "flow = " << u << " " << v << std::endl;

                    LOG(ERROR) << "m_sq_flow = " << u << " " << v << std::endl;
                }

            }

            // TODO: 是否可以进一步筛选
            {
                LOG(ERROR) << "cov_points = " << cov_points << ", min_cov_points = " << min_cov_points << std::endl;
                LOG(ERROR) << "selected_points.size() = " << selected_points.size() << std::endl;
                if (cov_points < min_cov_points) {
                    min_cov_points = cov_points;
                    best_inliers.clear();

                    /*{
                        flow_pre_points.clear();
                        flow_pre_points = flow_temp_vec; 
                        best_inliers = selected_points_temp;
                        assert(flow_pre_points.size() == best_inliers.size());
                    }*/


                    // 滤波光流大小和方向
                    if(selected_points_temp.size() > 10)
                        OritationFilter(normal_flow_temp_vec, normal_temp_vec, selected_points_temp);

                    LOG(ERROR) << "normal_flow_temp_vec.size = " << normal_flow_temp_vec.size() << std::endl;
                    LOG(ERROR) << "normal_temp_vec.size = " << normal_temp_vec.size() << std::endl;

                    // 2-28 修改 边缘光流
                    best_inliers = selected_points_temp;
                    normal_flow_pre_points.clear(); normal_pre_points.clear();
                    normal_flow_pre_points = normal_flow_temp_vec;   
                    normal_pre_points = normal_temp_vec;
                    assert(normal_flow_pre_points.size() == normal_pre_points.size() && "normal value and vector size is not same!");

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

    bool CalculateOpFlowPyramidRANSAC()
    {
        // 创建图像金字塔
        std::vector<cv::Mat> pyramid1, pyramid2;
        // cv::buildPyramid(TimeImage1, pyramid1, 3);  // 假设3层金字塔
        // cv::buildPyramid(TimeImage2, pyramid2, 3);
        // TimeImage1
        // std::vector<cv::Mat> pyramid1; // 用于存储金字塔的所有层
        int num_levels = 3; // 设定金字塔层数
        // 第一层是原始图像
        pyramid1.push_back(TimeImage1);
        // 构建金字塔，直接进行降采样，不使用高斯模糊
        for (int level = 1; level < num_levels; ++level) {
            cv::Mat downsampled_img;
            cv::pyrDown(pyramid1[level - 1], downsampled_img); // 直接降采样
            pyramid1.push_back(downsampled_img);
        }
        pyramid2.push_back(TimeImage2);
        // 构建金字塔，直接进行降采样，不使用高斯模糊
        for (int level = 1; level < num_levels; ++level) {
            cv::Mat downsampled_img;
            cv::pyrDown(pyramid2[level - 1], downsampled_img); // 直接降采样
            pyramid2.push_back(downsampled_img);
        }

        /*{
        static long int pyramid_img_count = 0;
        for (int level = pyramid1.size() - 1; level >= 0; --level) {
            std::string file_name = "/media/hao/hao2/228/test/lab/pyramid_img/" + std::to_string(pyramid_img_count) + ".png";
            cv::Mat img1 = pyramid1[level].clone();
            cv::Mat img2 = pyramid2[level].clone();

            cv::Mat color_img1 = cv::Mat::zeros(img1.size(), CV_8UC3);
            cv::Mat color_img2 = cv::Mat::zeros(img2.size(), CV_8UC3);

            cv::Mat mask_pos = img1 > 0;  // 大于0的区域
            cv::Mat mask_neg = img1 < 0;  // 小于0的区域
            cv::Mat mask_zero = img1 == 0; // 等于0的区域
            // mask_pos.convertTo(mask_pos, CV_8U);
            // mask_neg.convertTo(mask_neg, CV_8U);
            // mask_zero.convertTo(mask_zero, CV_8U);
            // 将大于0的区域设置为红色 (BGR: 0, 0, 255)
            color_img1.setTo(cv::Vec3b(0, 0, 255), mask_pos);
            // 将小于0的区域设置为蓝色 (BGR: 255, 0, 0)
            color_img1.setTo(cv::Vec3b(255, 0, 0), mask_neg);
            // 将等于0的区域设置为黑色 (BGR: 0, 0, 0) -> 已经是默认值
            color_img1.setTo(cv::Vec3b(255, 255, 255), mask_zero);

            mask_pos = img2 > 0;  // 大于0的区域
            mask_neg = img2 < 0;  // 小于0的区域
            mask_zero = img2 == 0; // 等于0的区域
            // mask_pos.convertTo(mask_pos, CV_8U);
            // mask_neg.convertTo(mask_neg, CV_8U);
            // mask_zero.convertTo(mask_zero, CV_8U);
            // 将大于0的区域设置为红色 (BGR: 0, 0, 255)
            color_img2.setTo(cv::Vec3b(0, 0, 255), mask_pos);
            // 将小于0的区域设置为蓝色 (BGR: 255, 0, 0)
            color_img2.setTo(cv::Vec3b(255, 0, 0), mask_neg);
            // 将等于0的区域设置为黑色 (BGR: 0, 0, 0) -> 已经是默认值
            color_img2.setTo(cv::Vec3b(255, 255, 255), mask_zero);


            cv::Mat result;
            // cv::bitwise_and(img1, img2, result);
            // cv::Mat local_window;
            // result.convertTo(result, CV_8U);
            // img1.copyTo(local_window, result);  // 将重叠区域提取到 local_window 中
            // local_window.convertTo(local_window, CV_8UC1);
            // cv::imwrite(file_name, local_window); // 保存为 PNG 图像
            // img2.copyTo(local_window, result);  // 将重叠区域提取到 local_window 中
            // local_window.convertTo(local_window, CV_8UC1);
            // std::string file_name2 = "/media/hao/hao2/228/test/lab/pyramid_light/" + std::to_string(pyramid_img_count++) + ".png";
            // cv::imwrite(file_name2, local_window); // 保存为 PNG 图像    

            cv::bitwise_and(img1, img2, result);
            cv::Mat local_window;
            result.convertTo(result, CV_8U);
            img1.copyTo(local_window);  // 将重叠区域提取到 local_window 中
            local_window.convertTo(local_window, CV_8UC1);
            cv::imwrite(file_name, color_img1); // 保存为 PNG 图像
            img2.copyTo(local_window);  // 将重叠区域提取到 local_window 中
            local_window.convertTo(local_window, CV_8UC1);
            std::string file_name2 = "/media/hao/hao2/228/test/lab/pyramid_light/" + std::to_string(pyramid_img_count++) + ".png";
            cv::imwrite(file_name2, color_img2); // 保存为 PNG 图像             
        }

        return true;
        }*/

        cv::Mat img1 = pyramid1.back();
        cv::Mat img2 = pyramid2.back();

        cv::Mat result;
        cv::bitwise_and(img1, img2, result);
        std::vector<cv::Point2d> selected_points;
        std::vector<cv::Point2d> temp_points;
        cv::findNonZero(result, selected_points);
        LOG(ERROR) << "selected_points size = " << selected_points.size() << std::endl;

        std::vector<event_flow_velocity> flow_temp_vec;
        // 计算当前层的光流
        // 遍历选定的点
        for (const auto& pt : selected_points) {
            // 将点坐标从上层金字塔转换到当前金字塔层级
            // cv::Point2f pt_scaled(pt.x / pow(2, level), pt.y / pow(2, level));

            int x = static_cast<int>(pt.x);
            int y = static_cast<int>(pt.y);

            double grad_x = 0;
            double grad_y = 0;

            // 初始化光流分量
            double u = 0.0;
            double v = 0.0;

            // 构建用于计算光流的矩阵
            double Ix = 0.0, Iy = 0.0, It = 0.0;
            double Ixx = 0.0, Ixy = 0.0, Iyy = 0.0, Ixt = 0.0, Iyt = 0.0;

            // 遍历局部窗口并计算梯度
            for (int i = -radius; i <= radius; ++i) {
                for (int j = -radius; j <= radius; ++j) {
                    int pixel_x = x + j;
                    int pixel_y = y + i;

                    // 确保不越界
                    // if (pixel_x > 0 && pixel_x < img1.cols - 1 && pixel_y > 0 && pixel_y < img1.rows - 1) {
                    if (pixel_x > 0 && pixel_x < img1.cols && pixel_y > 0 && pixel_y < img1.rows) {
                        // 计算空间梯度（水平和垂直）
                        double Ix_left = img1.at<double>(pixel_y, pixel_x - 1);
                        // double Ix_right = img1.at<double>(pixel_y, pixel_x + 1);
                        double Iy_top = img1.at<double>(pixel_y - 1, pixel_x);
                        // double Iy_bottom = img1.at<double>(pixel_y + 1, pixel_x);

                        double Ix_right = img1.at<double>(pixel_y, pixel_x);
                        double Iy_bottom = img1.at<double>(pixel_y, pixel_x);
                        
                        // 计算梯度
                        Ix += (Ix_right - Ix_left);
                        Iy += (Iy_bottom - Iy_top);

                        // 计算时间梯度
                        double It_current = img2.at<double>(y, x) / event_t1;
                        It += It_current;

                        // 计算梯度的二次矩阵项（用于最小二乘法）
                        Ixx += Ix * Ix;
                        Ixy += Ix * Iy;
                        Iyy += Iy * Iy;
                        Ixt += Ix * It_current;
                        Iyt += Iy * It_current;
                    }
                }
            }

            // 计算光流分量（u, v）使用最小二乘法解线性方程
            double denominator = Ixx * Iyy - Ixy * Ixy;

            if (denominator == 0) {
                std::cerr << "Warning: Singular matrix encountered during optical flow calculation." << std::endl;
                denominator = 1e-3; // 避免除零错误
            }

            u = (Iyy * Ixt - Ixy * Iyt) / denominator;
            v = (Ixx * Iyt - Ixy * Ixt) / denominator;

            // 使用预测的时间梯度计算误差
            double It_predicted = Ix * u + Iy * v;
            double cov_points = std::abs(It_predicted - It);

            u = u * pow(2, num_levels - 1);
            v = v * pow(2, num_levels - 1);
            cv::Point2d new_pt;
            new_pt.x = pt.x * pow(2, num_levels - 1);
            new_pt.y = pt.y * pow(2, num_levels - 1);

            event_flow_velocity flow(u, v);
            flow_temp_vec.push_back(flow);

            temp_points.push_back(new_pt);
        }


        // 循环金字塔每一层
        /*
        for (int level = pyramid1.size() - 1; level >= 0; --level) {
            cv::Mat img1 = pyramid1[level];
            cv::Mat img2 = pyramid2[level];

            // 查找满足光流约束的
            cv::Mat result;
            cv::bitwise_and(img1, img2, result);


            std::vector<cv::Point2d> selected_points;
            cv::findNonZero(result, selected_points);
            LOG(ERROR) << "selected_points size = " << selected_points.size() << std::endl;

            // selected_points 是当前层的参考点
            // 计算当前层的光流
            // 遍历选定的点
            for (const auto& pt : selected_points) {
                // 将点坐标从上层金字塔转换到当前金字塔层级
                cv::Point2f pt_scaled(pt.x / pow(2, level), pt.y / pow(2, level));

                int x = static_cast<int>(pt_scaled.x);
                int y = static_cast<int>(pt_scaled.y);

                double grad_x = 0;
                double grad_y = 0;

                // 初始化光流分量
                double u = 0.0;
                double v = 0.0;

                // 构建用于计算光流的矩阵
                double Ix = 0.0, Iy = 0.0, It = 0.0;
                double Ixx = 0.0, Ixy = 0.0, Iyy = 0.0, Ixt = 0.0, Iyt = 0.0;

                // 遍历局部窗口并计算梯度
                for (int i = -radius; i <= radius; ++i) {
                    for (int j = -radius; j <= radius; ++j) {
                        int pixel_x = x + j;
                        int pixel_y = y + i;

                        // 确保不越界
                        // if (pixel_x > 0 && pixel_x < img1.cols - 1 && pixel_y > 0 && pixel_y < img1.rows - 1) {
                        if (pixel_x > 0 && pixel_x < img1.cols && pixel_y > 0 && pixel_y < img1.rows) {
                            // 计算空间梯度（水平和垂直）
                            double Ix_left = img1.at<double>(pixel_y, pixel_x - 1);
                            // double Ix_right = img1.at<double>(pixel_y, pixel_x + 1);
                            double Iy_top = img1.at<double>(pixel_y - 1, pixel_x);
                            // double Iy_bottom = img1.at<double>(pixel_y + 1, pixel_x);

                            double Ix_right = img1.at<double>(pixel_y, pixel_x);
                            double Iy_bottom = img1.at<double>(pixel_y, pixel_x);
                            
                            // 计算梯度
                            Ix += (Ix_right - Ix_left);
                            Iy += (Iy_bottom - Iy_top);

                            // 计算时间梯度
                            double It_current = img2.at<double>(y, x) / event_t1;
                            It += It_current;

                            // 计算梯度的二次矩阵项（用于最小二乘法）
                            Ixx += Ix * Ix;
                            Ixy += Ix * Iy;
                            Iyy += Iy * Iy;
                            Ixt += Ix * It_current;
                            Iyt += Iy * It_current;
                        }
                    }
                }

                // 计算光流分量（u, v）使用最小二乘法解线性方程
                double denominator = Ixx * Iyy - Ixy * Ixy;

                if (denominator == 0) {
                    std::cerr << "Warning: Singular matrix encountered during optical flow calculation." << std::endl;
                    denominator = 1e-3; // 避免除零错误
                }

                u = (Iyy * Ixt - Ixy * Iyt) / denominator;
                v = (Ixx * Iyt - Ixy * Ixt) / denominator;

                // 使用预测的时间梯度计算误差
                double It_predicted = Ix * u + Iy * v;
                double cov_points = std::abs(It_predicted - It);

                event_flow_velocity flow(u, v);
                // flow_temp_vec.push_back(flow);
            }

            // 传递到下一层
        }
        */

        // 直接在低分辨率上进行光流估计
        best_inliers.clear();
        // best_inliers = selected_points;
        best_inliers = temp_points;
        flow_pre_points = flow_temp_vec; 

        return true; // 返回成功
    }

    // RANSAC光流计算
    // std::fstream output;
    bool CalculateOpFlowPrepointWithPolarityRANSAC() {
        int points_num = 15;
        const int max_iterations = 50;
        double min_cov_points = std::numeric_limits<double>::max();

        // 取出其中 TimeImage1 是 正数的,组成图像
        cv::Mat positive_img;
        {
            // 创建掩膜，提取 TimeImage1 中正数的位置
            cv::Mat mask_pos = TimeImage1 > 0;  // 大于0的部分
            // 创建一个新的图像，并将掩膜对应的位置保留下来，其他位置设为0
            cv::Mat positive_img;
            TimeImage1.copyTo(positive_img, mask_pos);  // 仅复制正数部分
        }
        
        // int iter_count = 0;
        for (int iter = 0; iter < max_iterations; ++iter) {
            std::vector<cv::Point2d> selected_points;
            // RandomlySelectPoints(TimeImage1, selected_points, points_num);
            SelectTopEventPoints(positive_img, selected_points, points_num); // 选择事件数更多的点

            if(selected_points.size() < points_num)
            {
                // std::cout << "skip" << std::endl;
                // LOG(WARNING) << "selected points num is not valid!" << std::endl;
                LOG(ERROR) << "selected points num is not valid!" << std::endl;

                LOG(ERROR) << "selected_points.size = " << selected_points.size() << std::endl;
                // return false;
                continue;
            }
            else
            {
                LOG(ERROR) << "start to Ransac " << std::endl;
            }

            std::vector<event_flow_velocity> flow_temp_vec;
            // Eigen::MatrixXd grad_A(points_num, 2);
            // Eigen::MatrixXd grad_b(points_num, 1);
            
            // std::cout << "Traversal selected_points: " << selected_points.size() << std::endl;
            int rows = 0;
            int zeros_count = 0;
            double cov_points = 0;
            for (const auto& pt : selected_points) {
                int x = static_cast<int>(pt.x);
                int y = static_cast<int>(pt.y);

                double grad_x = 0;
                double grad_y = 0;

                // Eigen::Vector2d grad_x, grad_y;

                // 初始化光流分量
                double u = 0.0;
                double v = 0.0;

                // 构建用于计算光流的矩阵
                double Ix = 0.0, Iy = 0.0, It = 0.0;
                double Ixx = 0.0, Ixy = 0.0, Iyy = 0.0, Ixt = 0.0, Iyt = 0.0;


                // 遍历局部窗口并计算梯度
                for (int i = -radius; i <= radius; ++i) {
                    for (int j = -radius; j <= radius; ++j) {
                        int pixel_x = x + j;
                        int pixel_y = y + i;

                        // 确保不越界
                        if (pixel_x > 0 && pixel_x < TimeImage1.cols - 1 && pixel_y > 0 && pixel_y < TimeImage1.rows - 1) {
                            // 计算空间梯度（水平和垂直）
                            double Ix_left = TimeImage1.at<double>(pixel_y, pixel_x - 1);
                            double Ix_right = TimeImage1.at<double>(pixel_y, pixel_x + 1);
                            double Iy_top = TimeImage1.at<double>(pixel_y - 1, pixel_x);
                            double Iy_bottom = TimeImage1.at<double>(pixel_y + 1, pixel_x);
                            
                            // 计算梯度
                            Ix += (Ix_right - Ix_left) / 2.0;
                            Iy += (Iy_bottom - Iy_top) / 2.0;

                            // 计算时间梯度
                            double It_current = TimeImage2.at<double>(y, x) / event_t1;
                            It += It_current;

                            // 计算梯度的二次矩阵项（用于最小二乘法）
                            Ixx += Ix * Ix;
                            Ixy += Ix * Iy;
                            Iyy += Iy * Iy;
                            Ixt += Ix * It_current;
                            Iyt += Iy * It_current;

                            // save_grad << 
                        }
                    }
                }


                /*
                // 将 [-radius, radius] [-radius, radius]  区间内的图像保存到 /media/hao/hao2/228/test/lab/test_flow中
                static long int index = 0;
                // 保存为 img_index.png

                // 定义裁剪区域 [-radius, radius] (左上角到右下角)
                cv::Rect crop_region(x - radius - 1, y - radius - 1, 2 * (radius) + 1, 2 * (radius) + 1);
                // 裁剪图像，避免越界
                cv::Mat cropped_image = TimeImage1(crop_region & cv::Rect(0, 0, TimeImage1.cols, TimeImage1.rows));  // 防止越界裁剪
                std::cout << "cropped_image.size = " << cropped_image.cols << ", " << cropped_image.rows << std::endl;

                // std::fstream save_grad("/media/hao/hao2/228/test/lab/test_flow/grad.txt", std::ios::out | std::ios::app);

                // 确保保存目录存在
                std::string save_dir = "/media/hao/hao2/228/test/lab/test_flow";
                // if (!std::filesystem::exists(save_dir)) {
                //     std::filesystem::create_directories(save_dir);
                // }
                std::string filename = save_dir + "/img_" + std::to_string(index++) + ".png";
                double minVal, maxVal;
                cv::Mat cropped_image_normalized;
                cv::minMaxLoc(cropped_image, &minVal, &maxVal); // 获取 TimeImage1 的最小值和最大值
                cv::normalize(cropped_image, cropped_image_normalized, 0, 255, cv::NORM_MINMAX); // 将 TimeImage1 归一化到 0-255 范围
                if (!cv::imwrite(filename, cropped_image_normalized)) {
                        LOG(ERROR) << "Failed to save image: " << filename << std::endl;
                }
                */

                // 计算光流分量（u, v）使用最小二乘法解线性方程
                double denominator = Ixx * Iyy - Ixy * Ixy;
                if (denominator != 0) {
                    u = (Iyy * Ixt - Ixy * Iyt) / denominator;
                    v = (Ixx * Iyt - Ixy * Ixt) / denominator;
                } else {
                    LOG(ERROR) << "Warning: Singular matrix encountered during optical flow calculation." << std::endl;
                    u = v = 0.0;
                }

                double It_predicted = Ix * u + Iy * v;
                cov_points += std::abs(It_predicted - It);

                event_flow_velocity flow(u, v);
                flow_temp_vec.push_back(flow);
            }

            // TODO: 是否可以进一步筛选
            {
                LOG(ERROR) << "cov_points = " << cov_points << ", min_cov_points = " << min_cov_points << std::endl;
                LOG(ERROR) << "selected_points.size() = " << selected_points.size() << std::endl;
                if (cov_points < min_cov_points) {
                    min_cov_points = cov_points;
                    best_inliers.clear();
                    best_inliers = selected_points;
                    flow_pre_points = flow_temp_vec; 

                    LOG(ERROR) << "First Debug for flow: best_inliers.size = " << best_inliers.size() << std::endl;
                    LOG(ERROR) << "First Debug for flow: selected_points.size = " << best_inliers.size() << std::endl;
                }
            }
        }

        // 投影到归一化场景
        // flow_velocity.x /= K(0,0);
        // flow_velocity.y /= K(1,1);  

        // for(auto& flow: flow_pre_points)
        // {
        //     flow.x /= K(0,0); flow.y /= K(1,1);
        // }     

        LOG(ERROR) << "RANSAC OpFlow: \n"
                  << "\t min_cov: " << min_cov_points 
                  << "\t inlier: " <<  best_inliers.size()
                  << std::endl;

        if(best_inliers.size() < 5)
        {
            LOG(ERROR) << "inliers is not enough!" << std::endl;
            return false; 
        }

        return true;       
    }

    bool CalculateOpFlowForWindows(double event_window_dt) {
        int points_num = 15;
        const int max_iterations = 50;
        double min_cov_points = std::numeric_limits<double>::max();
        
        // LOG(ERROR) << "Start to Ransac" << std::endl;
        std::cout << "Start to Ransac = " << event_window_dt << std::endl;

        // int iter_count = 0;
        for (int iter = 0; iter < max_iterations; ++iter) {
            std::vector<cv::Point2d> selected_points;
            RandomlySelectPoints(TimeImage1, selected_points, points_num);
            // std::cout << "points_num = " << points_num << std::endl;
            // std::cout << "selected_points.size = " << selected_points.size() 
            //         << " points_num = " << points_num << std::endl;
            if(selected_points.size() < points_num)
            {
                // std::cout << "skip" << std::endl;
                // LOG(ERROR) << "selected points num is not valid!" << std::endl;
                std::cout << "selected points num is not valid!" << std::endl;
                // return false;
                continue;
            }
          
            std::vector<Eigen::Vector2d> flow_temp_vec;
            Eigen::MatrixXd grad_A(points_num, 2);
            Eigen::MatrixXd grad_b(points_num, 1);
            
            // std::cout << "Traversal selected_points: " << selected_points.size() << std::endl;
            int rows = 0;
            int zeros_count = 0;
            for (const auto& pt : selected_points) {
                int x = static_cast<int>(pt.x);
                int y = static_cast<int>(pt.y);

                double grad_x = 0;
                double grad_y = 0;

                // output.open("/home/hao/Desktop/output.txt", std::ios::app | std::ios::out);
                for (int i = -radius; i <= radius; ++i) {
                    for (int j = -radius; j <= radius; ++j) {
                        // 确保不越界
                        if (x + j > 0 && x + j < TimeImage1.cols - 1 && y + i > 0 && y + i < TimeImage1.rows - 1) {
                            // 输出像素值
                            // std::cout << "Pixel value at (" << x + j << ", " << y + i << ") = " 
                            //         << TimeImage1.at<double>(y + i, x + j) << std::endl;
                            // output << "Pixel value at (" << x + j << ", " << y + i << ") = " 
                            //         << TimeImage1.at<double>(y + i, x + j) << std::endl;
                            // 水平方向梯度
                            grad_x += (TimeImage1.at<double>(y + i, x + j + 1) - TimeImage1.at<double>(y + i, x + j - 1)) / 2;
                            // grad_x += (TimeImage1.at<double>(y + i, x + j) - TimeImage1.at<double>(y + i, x + j - 1));
                            // output << "delta_x = " << (TimeImage1.at<double>(y + i, x + j + 1) - TimeImage1.at<double>(y + i, x + j)) << std::endl;
                            // 垂直方向梯度
                            grad_y += (TimeImage1.at<double>(y + i + 1, x + j) - TimeImage1.at<double>(y + i - 1, x + j)) / 2;
                            // grad_y += (TimeImage1.at<double>(y + i, x + j) - TimeImage1.at<double>(y + i - 1, x + j));
                            // output << "delta_y = " << (TimeImage1.at<double>(y + i + 1, x + j) - TimeImage1.at<double>(y + i - 1, x + j)) << std::endl;
                        }
                    }
                    // std::cout << std::endl;  // 分隔每一行像素输出
                }

                double grad_b_i = TimeImage1.at<double>(y, x) / event_window_dt;

                if(grad_x == 0 && grad_y == 0)
                {
                    grad_b_i = 0;
                    zeros_count++;
                }
                else if(grad_b_i == 0)
                {
                    grad_x = grad_y = 0;
                    zeros_count++;
                }

                grad_A.block(rows, 0, 1, 2) << grad_x, grad_y;
                grad_b.block(rows, 0, 1, 1) << grad_b_i;
                // std::cout << "rows = " << rows << std::endl;
                rows ++;
            }

            // std::cout << "rows = " << rows << std::endl;
            // LOG(ERROR) << "zeros_count = " << zeros_count << std::endl;
            std::cout << "zeros_count = " << zeros_count << std::endl;

            // 本次迭代失败
            if (rows - zeros_count < 2)
                continue;

            // std::cout << "A = \n" 
            //           << grad_A << std::endl;

            // std::cout << "b = \n" 
            //           << grad_b << std::endl;

            // LOG(INFO) << "best flow : A * flow = b";
            // LOG(INFO) << "A = \n" 
            //           << grad_A << std::endl;

            // LOG(INFO) << "b = \n" 
            //           << grad_b << std::endl;            


            // LOG(ERROR) << "best flow : A * flow = b";
            // LOG(ERROR) << "A = \n" 
            //           << grad_A << std::endl;

            // LOG(ERROR) << "b = \n" 
            //           << grad_b << std::endl;     

            std::cout << "best flow : A * flow = b";
            std::cout << "A = \n" 
                      << grad_A << std::endl;

            std::cout << "b = \n" 
                      << grad_b << std::endl;     

            // Eigen::Vector2d flow = grad_A.colPivHouseholderQr().solve(grad_b); 

        {
            Eigen::VectorXd flow = grad_A.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(grad_b);
            // Eigen::JacobiSVD<Eigen::MatrixXd> svd(grad_A, Eigen::ComputeThinU | Eigen::ComputeThinV);
            // double cond = svd.singularValues()(0) / svd.singularValues().tail(1)(0);
            // LOG(INFO) << "Cond = " << cond << std::endl;
            // LOG(ERROR) << "Second Optimization" << std::endl;
            int rank = grad_A.fullPivLu().rank();
            LOG(ERROR) << "Rank = " << rank << std::endl;
            std::cout << "Second Optimization" << std::endl;
            Eigen::VectorXd residuals = grad_A * flow - grad_b;
            LOG(ERROR) << "First Residuals = " << residuals.transpose() << std::endl;
            Eigen::VectorXd weights = (residuals.array().abs() + 1e-6).inverse();
            Eigen::MatrixXd weighted_grad_A = grad_A.array().colwise() * weights.array();  // 按列调整 A
            Eigen::VectorXd weighted_grad_b = grad_b.array() * weights.array();  // 调整 b
            // 再次进行加权最小二乘法求解
            // Eigen::Vector2d flow_weighted = weighted_grad_A.colPivHouseholderQr().solve(weighted_grad_b);
            Eigen::VectorXd flow_weighted = weighted_grad_A.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(weighted_grad_b);
            
            event_flow_velocity flow_velocity_temp(flow_weighted);
            Eigen::VectorXd weight_residuals = weighted_grad_A * flow_weighted - weighted_grad_b;
            double cov_points = (weight_residuals).norm(); 
            LOG(ERROR) << "Second Residuals = " << weight_residuals.transpose() << std::endl;
            LOG(ERROR) << "best flow = " << flow_weighted.transpose() << std::endl;

            if (cov_points < min_cov_points) {
                min_cov_points = cov_points;
                best_inliers.clear();
                best_inliers = selected_points;
                flow_velocity = flow_velocity_temp;

                LOG(ERROR) << "Flow Debug: selected_points.size() = " << selected_points.size() << std::endl;

                LOG(ERROR) << "Flow Debug: best_inliers.size() = " << best_inliers.size() << std::endl;
            }
        }

        return false;

    /*{
            // 二次优化
            Eigen::VectorXd residuals = grad_A * flow - grad_b;
            LOG(INFO) << "First Residuals = " << residuals.transpose() << std::endl;
            // 3. 设置剔除离群点的阈值（根据具体情况调整）
            double threshold = 0.1;
            std::vector<int> inliers;  // 存储误差较小的内点索引

            for (int i = 0; i < residuals.size(); ++i) {
                if (std::abs(residuals(i)) < threshold) {
                    inliers.push_back(i);  // 选择残差小于阈值的点
                }
            }
            // 4. 使用内点重新构建 grad_A 和 grad_b
            int new_size = inliers.size();
            Eigen::MatrixXd grad_A_selected(new_size, grad_A.cols());
            Eigen::VectorXd grad_b_selected(new_size);
            for (int i = 0; i < new_size; ++i) {
                grad_A_selected.row(i) = grad_A.row(inliers[i]);
                grad_b_selected(i) = grad_b(inliers[i]);
            }
            // 5. 再次进行最小二乘优化，得到新的光流
            Eigen::Vector2d refined_flow = grad_A_selected.colPivHouseholderQr().solve(grad_b_selected);
            Eigen::VectorXd refined_residuals = grad_A_selected * refined_flow - grad_b_selected;
            LOG(INFO) << "Second Residuals = " << refined_residuals.transpose() << std::endl;

            // flow /= (event_dt - event_t1 + 1e-3);
            event_flow_velocity flow_velocity_temp(refined_flow);
            double cov_points = (grad_A_selected * refined_flow - grad_b_selected).norm(); 

            LOG(INFO) << "best flow = " << refined_flow.transpose() << std::endl;

            std::vector<cv::Point2d> inliers_points;
            for (int i : inliers) {
                inliers_points.push_back(selected_points[i]);
            }

            if (cov_points < min_cov_points) {
                min_cov_points = cov_points;
                best_inliers = inliers_points;
                flow_velocity = flow_velocity_temp;
            }
        }*/

            // // flow /= (event_dt - event_t1 + 1e-3);
            // event_flow_velocity flow_velocity_temp(flow);
            // double cov_points = (grad_A * flow - grad_b).norm(); 

            // LOG(INFO) << "best flow = " << flow.transpose() << std::endl;

            // if (cov_points < min_cov_points) {
            //     min_cov_points = cov_points;
            //     best_inliers = selected_points;
            //     flow_velocity = flow_velocity_temp;
            // }
        }

        // 投影到归一化场景
        // flow_velocity.x /= K(0,0);
        // flow_velocity.y /= K(1,1);       

        LOG(ERROR) << "RANSAC OpFlow: \n"
                  << "\t min_cov: " << min_cov_points << std::endl;

        LOG(ERROR) << "Best Flow: \n" 
                  << "[" << flow_velocity.x << ", "<< flow_velocity.y << "]" 
                  << std::endl;  

        // std::cout << "RANSAC OpFlow: \n"
        //           << "\t min_cov: " << min_cov_points << std::endl;

        // std::cout << "Best Flow: \n" 
        //           << "[" << flow_velocity.x << ", "<< flow_velocity.y << "]" 
        //           << std::endl;  

        if(best_inliers.size() < 5)
        {
            LOG(ERROR) << "inliers is not enough!" << std::endl;
            return false; 
        }

        return true;    
    }

    bool CalculateSparseOpFlowOpenCV() {

        // 检测第一个图像中的关键点（特征点）
        std::vector<cv::Point2f> keypoints1;
        // cv::goodFeaturesToTrack(TimeImage1, keypoints1, 100, 0.01, 10);

        // 归一化到0-255范围，再转换为CV_8UC1
        cv::Mat TimeImage1_8UC1, TimeImage2_8UC1;
        cv::normalize(TimeImage1, TimeImage1, 0, 255, cv::NORM_MINMAX);
        cv::normalize(TimeImage2, TimeImage2, 0, 255, cv::NORM_MINMAX);

        // 将归一化后的图像转换为8位无符号整型
        TimeImage1.convertTo(TimeImage1_8UC1, CV_8UC1);
        TimeImage2.convertTo(TimeImage2_8UC1, CV_8UC1);
        // cv::imshow("origin", TimeImage1_8UC1);
        // cv::imshow("origin2", TimeImage2_8UC1);
        // cv::waitKey(1);  // 等待键盘输入

        cv::goodFeaturesToTrack(TimeImage1_8UC1, keypoints1, 100, 0.35, 10);

        if (keypoints1.empty()) {
            // std::cerr << "错误：在第一个时空图中未找到关键点。" << std::endl;
            return false;
        }

        // 计算光流，得到第二个图像中的对应关键点
        // std::vector<cv::Point2f> keypoints2;
        // std::vector<uchar> status;
        // std::vector<float> err;

        std::vector<cv::Point2f> keypoints2, keypoints1_back;
        std::vector<uchar> status, status_back;
        std::vector<float> err, err_back;

        cv::calcOpticalFlowPyrLK(TimeImage1_8UC1, TimeImage2_8UC1, keypoints1, keypoints2, status, err);

        // Step 2: Backward calculation from TimeImage2 to TimeImage1
        cv::calcOpticalFlowPyrLK(TimeImage2_8UC1, TimeImage1_8UC1, keypoints2, keypoints1_back, status_back, err_back);

        // 初始化统一光流和有效点计数
        Eigen::Vector2d unified_flow = Eigen::Vector2d(0.0f, 0.0f);
        int valid_points = 0;

        // 创建用于可视化的图像
        cv::Mat output;
        cv::cvtColor(TimeImage2_8UC1, output, cv::COLOR_GRAY2BGR);

        std::vector<double> flow_magnitudes;  // 存储所有有效光流的模长
        // std::vector<Eigen::Vector2d> valid_flows;  // 存储所有有效光流
        valid_flows.clear();
        best_inliers.clear();

        // 遍历所有关键点，计算统一光流并进行可视化
        for (size_t i = 0; i < keypoints1.size(); i++) {
            // if (status[i]) {
            if (status[i] && status_back[i]) {
                float dist = cv::norm(keypoints1[i] - keypoints1_back[i]);
                // std::cout << "dist = " << dist << std::endl;
                if (dist < 1.0 && dist > 0.01) {
                    // 计算单个点的光流向量
                    Eigen::Vector2d flow(keypoints2[i].x - keypoints1[i].x, keypoints2[i].y - keypoints1[i].y);
                    // unified_flow += flow;  // 累加光流
                    // unified_flow = ((flow.norm() > unified_flow.norm())? flow:unified_flow);  // 选择最近的光流,变化更大
                    // valid_points++;  // 增加有效点计数

                    flow_magnitudes.push_back(flow.norm());
                    valid_flows.push_back(flow);  // 保存有效的光流向量
                    valid_points++;  // 增加有效点计数

                    best_inliers.push_back(keypoints2[i]);

                    // LOG(INFO) << "pre flow = " << flow.transpose() << std::endl;

                    // 可视化：绘制光流矢量
                    cv::line(output, keypoints1[i], keypoints2[i], cv::Scalar(0, 255, 0), 2);  // 绿色线表示光流
                    cv::circle(output, keypoints2[i], 3, cv::Scalar(0, 0, 255), -1);  // 红色圆表示目标点
                }
            }
        }

        // 计算光流的中位数
        if (!flow_magnitudes.empty()) {
            // 找到flow_magnitudes的中位数
            size_t mid_index = flow_magnitudes.size() / 2;
            std::nth_element(flow_magnitudes.begin(), flow_magnitudes.begin() + mid_index, flow_magnitudes.end());
            double median_flow_magnitude = flow_magnitudes[mid_index];

            // 找到与中位数模长最接近的光流向量
            Eigen::Vector2d median_flow = valid_flows[0];
            double min_diff = std::abs(valid_flows[0].norm() - median_flow_magnitude);

            for (const auto& flow : valid_flows) {
                double diff = std::abs(flow.norm() - median_flow_magnitude);
                if (diff < min_diff) {
                    min_diff = diff;
                    median_flow = flow;
                }
            }

            unified_flow = median_flow;  // 设置统一光流为中位数光流
            LOG(INFO) << "median_flow = " << median_flow.transpose() << std::endl;
        }

        // 如果没有有效点，返回错误
        if (valid_points < 5) {
            // std::cerr << "fault: not enough points" << std::endl;
            return false;
        }

        // 计算平均光流并进行时间归一化
        // unified_flow /= static_cast<double>(valid_points);
        // unified_flow /= (event_dt - event_t1 + 1e-5f);

        // unified_flow = unified_flow / (event_dt - event_t1 + 1e-5);

        unified_flow = unified_flow / event_t1;

        if(unified_flow.norm() < 1e-5)
            return false;

        // std::copy_if(keypoints2.begin(), keypoints2.end(), std::back_inserter(best_inliers),
        //             [&status, idx = 0](const cv::Point2f&) mutable { return status[idx++]; });

        // best_inliers = keypoints2 [status];
        event_flow_velocity flow_velocity_temp(unified_flow);
        flow_velocity = flow_velocity_temp;

        // 输出统一光流值
        std::cout << "valid points: " << valid_points << std::endl;
        std::cout << "unified_flow: " << unified_flow.transpose() << std::endl; 

        LOG(INFO) << "unified_flow: " << unified_flow.transpose() << std::endl;   

        // 显示可视化结果
        if(show_events_)
        {
            // 转换为 ROS 消息并发布
            sensor_msgs::Image flow_image;
            

            // 更新 CvImage 对象的 cv::Mat 数据
            cv_image_.header.frame_id = "event";
            cv_image_.header.stamp = process_time;
            cv_image_.encoding = "bgr8";  // 根据图像格式选择合适的编码
            cv_image_.image = output;

            cv_image_.toImageMsg(flow_image);
            image_pub_.publish(flow_image);


            // cv::imshow("flow", output);
            // cv::waitKey(1);  // 等待键盘输入
        }      


        LOG(INFO) << "Best Flow: \n" 
                  << "[" << flow_velocity.x << ", "<< flow_velocity.y << "]" 
                  << std::endl;  

        // if(best_inliers.size() < 5)
        // {
        //     LOG(WARNING) << "inliers is not enough!" << std::endl;
        //     return false; 
        // }

        return true;
    }

    bool CalculateDenseOpFlowOpenCV()
    {
        // 创建 RLOF 对象
        cv::Ptr<cv::DenseOpticalFlow> rlof = cv::optflow::createOptFlow_DenseRLOF();
        cv::Mat flow_mat;

        // 计算光流
        cv::Mat TimeImage1_color, TimeImage2_color;
        cv::Mat TimeImage1_8UC1, TimeImage2_8UC1;
        cv::normalize(TimeImage1, TimeImage1, 0, 255, cv::NORM_MINMAX);
        cv::normalize(TimeImage2, TimeImage2, 0, 255, cv::NORM_MINMAX);
        TimeImage1.convertTo(TimeImage1_8UC1, CV_8UC1);
        TimeImage2.convertTo(TimeImage2_8UC1, CV_8UC1);

        cv::cvtColor(TimeImage1_8UC1, TimeImage1_color, cv::COLOR_GRAY2BGR);
        cv::cvtColor(TimeImage2_8UC1, TimeImage2_color, cv::COLOR_GRAY2BGR);
        rlof->calc(TimeImage1_color, TimeImage2_color, flow_mat);

        // 存储光流点
        std::vector<cv::Point2d> flow_points;

        // 计算统一光流向量并存储光流点
        cv::Scalar avg_flow = cv::mean(flow_mat);
        Eigen::Vector2d unified_flow(avg_flow[0], avg_flow[1]);  // 将光流转换为 Eigen 向量

        std::cout << "Unified Flow: (" << unified_flow[0] << ", " << unified_flow[1] << ")" << std::endl;

        // 遍历 flow_mat 矩阵，存储光流点
        for (int y = 0; y < flow_mat.rows; y += 5) {
            for (int x = 0; x < flow_mat.cols; x += 5) {
                const cv::Point2f flow_at_xy = flow_mat.at<cv::Point2f>(y, x);
                flow_points.emplace_back(static_cast<double>(x + flow_at_xy.x), static_cast<double>(y + flow_at_xy.y));
            }
        }

        // 可视化光流
        // cv::Mat flow_vis;
        // cv::cvtColor(TimeImage1, flow_vis, cv::COLOR_GRAY2BGR); // 将灰度图转换为彩色图以可视化

        // for (int y = 0; y < flow_mat.rows; y += 5) {
        //     for (int x = 0; x < flow_mat.cols; x += 5) {
        //         const cv::Point2f flow_at_xy = flow_mat.at<cv::Point2f>(y, x);
        //         cv::line(flow_vis, cv::Point(x, y), 
        //                 cv::Point(cvRound(x + flow_at_xy.x), cvRound(y + flow_at_xy.y)), 
        //                 cv::Scalar(0, 255, 0));
        //         cv::circle(flow_vis, cv::Point(x, y), 1, cv::Scalar(0, 255, 0), -1);
        //     }
        // }

        // 显示结果
        // cv::imshow("Optical Flow RLOF", flow_vis);
        // cv::waitKey(1);

        unified_flow = unified_flow / (event_dt - event_t1 + 1e-5);

        if(unified_flow.norm() < 1e-5)
            return false;
        best_inliers.clear();
        best_inliers = flow_points;
        event_flow_velocity flow_velocity_temp(unified_flow);
        flow_velocity = flow_velocity_temp;

        LOG(INFO) << "Best Flow: \n" 
                  << "[" << flow_velocity.x << ", "<< flow_velocity.y << "]" 
                  << std::endl;  

        if(best_inliers.size() < 5)
        {
            LOG(WARNING) << "inliers is not enough!" << std::endl;
            return false; 
        }



        return true;
    }


    // 计算角速度
    void AugularVelocityEstiRANSAC(geometry_msgs::TwistWithCovarianceStamped& radar_vel) {
        // assert(best_inliers.size() > 2);

        std::cout << "valid_flows.size = " << valid_flows.size() 
                    << " best_inliers.size = " << best_inliers.size() << std::endl;
        LOG(ERROR) << "valid_flows.size = " << valid_flows.size() 
                    << " best_inliers.size = " << best_inliers.size() << std::endl;
        assert(valid_flows.size() == best_inliers.size() && "size of flows is not same");

        // Eigen::MatrixXd A(best_inliers.size(), 3);
        // Eigen::VectorXd b(best_inliers.size());
        Eigen::Vector3d linear_vel(radar_vel.twist.twist.linear.x, radar_vel.twist.twist.linear.y, radar_vel.twist.twist.linear.z);

        linear_vel = lpf_v.filter(linear_vel);
        radar_vel.twist.twist.linear.x = linear_vel(0);
        radar_vel.twist.twist.linear.y = linear_vel(1);
        radar_vel.twist.twist.linear.z = linear_vel(2);


        Eigen::Vector3d angular_vec;

        if(linear_vel.norm() < 1e-6)
        {
            LOG(WARNING) << "radar ego velocity is valid!" << std::endl;
            return ;
        }

        // Parameters for RANSAC
        int max_iterations = 15;  // Maximum RANSAC iterations
        double threshold = 1.0;    // Error threshold to consider a point as an inlier
        int best_inlier_count = 0;  // Track the highest inlier count
        Eigen::Vector3d best_angular_vec;  // Best estimated angular velocity

        std::vector<size_t> indices(best_inliers.size());
        std::iota(indices.begin(), indices.end(), 0); // 生成 0 到 best_inliers.size()-1 的索引

        for (int iteration = 0; iteration < max_iterations; ++iteration) {
            // Step 1: Randomly sample points
            std::vector<int> sample_indices;
            // std::sample(best_inliers.begin(), best_inliers.end(), 
            //             std::back_inserter(sample_indices), 3, std::mt19937{std::random_device{}()});

            // 随机打乱索引
            std::shuffle(indices.begin(), indices.end(), std::mt19937{std::random_device{}()});

            // 选择前 3 个索引
            int nums = std::min(size_t(8), indices.size());
            sample_indices.assign(indices.begin(), indices.begin() + nums);

            Eigen::MatrixXd A(nums, 3);  // Adjust for RANSAC sample size
            Eigen::VectorXd b(nums);
            
            for (int i = 0; i < nums; ++i) {
                int idx = sample_indices[i];
                Eigen::Vector3d pixel(best_inliers[idx].x, best_inliers[idx].y, 1.0);
                Eigen::Vector3d pixel_cord = K.inverse() * pixel;
                Eigen::Matrix3d pixel_skew = skew(pixel_cord);
                Eigen::Vector3d prefix = pixel_skew * T_re.block(0, 0, 3, 3) * linear_vel;

                Eigen::Vector3d flow(valid_flows[idx](0) , valid_flows[idx](1) , 0); 
                flow /= event_t1;
                flow = K.inverse() * flow;

                A.row(i) = prefix.transpose() * pixel_skew;
                b(i) = -1.0 * prefix.transpose() * flow;
            }

            // Step 2: Solve for angular velocity using least squares
            angular_vec = A.colPivHouseholderQr().solve(b);

            // Step 3: Calculate residuals for all points and count inliers
            int inlier_count = 0;
            std::vector<cv::Point2d> current_inliers;
            for (int i = 0; i < best_inliers.size(); ++i) {
                Eigen::Vector3d pixel(best_inliers[i].x, best_inliers[i].y, 1.0);
                Eigen::Vector3d pixel_cord = K.inverse() * pixel;
                Eigen::Matrix3d pixel_skew = skew(pixel_cord);
                Eigen::Vector3d prefix = pixel_skew * T_re.block(0, 0, 3, 3) * linear_vel;

                Eigen::Vector3d flow(valid_flows[i](0) , valid_flows[i](0) , 0); 
                flow /= event_t1;
                flow = K.inverse() * flow;

                double error = (prefix.transpose() * pixel_skew * angular_vec + prefix.transpose() * flow).norm();
                LOG(INFO) << "error = " << error << std::endl;
                if (error < threshold) {
                    current_inliers.push_back(best_inliers[i]);
                    ++inlier_count;
                }
            }

            // Step 4: Keep the best model (largest number of inliers)
            if (inlier_count > best_inlier_count) {
                best_inlier_count = inlier_count;
                best_angular_vec = angular_vec;
                best_inliers = current_inliers;

            }
        }

        // Step 5: Use the best inliers for weighted least squares
        Eigen::MatrixXd A(best_inliers.size(), 3);
        Eigen::VectorXd b(best_inliers.size());
        Eigen::MatrixXd W = Eigen::MatrixXd::Identity(best_inliers.size(), best_inliers.size()); // Initialize weights

        for (int i = 0; i < best_inliers.size(); ++i) {
            Eigen::Vector3d pixel(best_inliers[i].x, best_inliers[i].y, 1.0);
            Eigen::Vector3d pixel_cord = K.inverse() * pixel;
            Eigen::Matrix3d pixel_skew = skew(pixel_cord);
            Eigen::Vector3d prefix = pixel_skew * T_re.block(0, 0, 3, 3) * linear_vel;

            Eigen::Vector3d flow(valid_flows[i](0) , valid_flows[i](1) , 0); 
            flow /= event_t1;
            flow = K.inverse() * flow;

            A.row(i) = prefix.transpose() * pixel_skew;
            b(i) = -1.0 * prefix.transpose() * flow;

            // Compute residual for weight
            double residual = (prefix.transpose() * pixel_skew * best_angular_vec + prefix.transpose() * flow).norm();
            W(i, i) = 1.0 / (residual + 1e-6); // Prevent division by zero
        }


        // Step 6: Solve weighted least squares
        Eigen::Vector3d weighted_angular_vec = (A.transpose() * W * A).ldlt().solve(A.transpose() * W * b);

     /*{   
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

            // Eigen::Vector3d flow(flow_velocity.x, flow_velocity.y, 0);              // TODO: flow is 0??
            Eigen::Vector3d flow(valid_flows[i](0), valid_flows[i](1), 0); 
            flow /= event_t1;

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

        Eigen::VectorXd residual = A * angular_vec - b;
        LOG(INFO) << "First Angular Velocity Residual = " << residual << std::endl;

        // 使用加权最小二乘法
        Eigen::MatrixXd W = residual.asDiagonal(); // 初始化权重矩阵为单位矩阵

        // 加权求解 A * W * x = b
        angular_vec = (A.transpose() * W * A).colPivHouseholderQr().solve(A.transpose() * W * b);
    }*/
        angular_vec = weighted_angular_vec;
        angular_vec = lpf.filter(angular_vec);

        // 将事件系的角速度投影回雷达系
        angular_vec = T_re.block(0, 0, 3, 3).inverse() * angular_vec;

        radar_vel.twist.twist.angular.x = angular_vec(0);
        radar_vel.twist.twist.angular.y = angular_vec(1);
        radar_vel.twist.twist.angular.z = angular_vec(2);

        // LOG(INFO) << "Calculate Angular Velocity = " << std::endl;
        LOG(INFO) << "A * w = b \n"
                  << A << "\n"
                  << b << "\n" << std::endl; 

        LOG(INFO) << "Angular Velocity = " << std::endl;
        LOG(INFO) << angular_vec.transpose() << std::endl;

        // 计算残差
        // residual = A * angular_vec - b;
        Eigen::VectorXd residual = W.inverse() * (A * angular_vec - b);
        double sigma_r_squared = (residual.squaredNorm()) / (A.rows() - A.cols());
        LOG(INFO) << "Second Angular Velocity Residual = " << residual << std::endl;


        // 计算协方差矩阵
        Eigen::Matrix3d covariance_matrix = sigma_r_squared * (A.transpose() * A).inverse();

        // LOG(INFO) << "Angular Velocity covariance_matrix = " << covariance_matrix << std::endl;

        // 将协方差矩阵填充到 Covariance 中
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                radar_vel.twist.covariance[21 + i * 6 + j] = covariance_matrix(i, j);  // 填充角速度的协方差
            }
        }

        // LOG_Velocity(radar_vel);
    }

    /*
    未做完的工作
    
    bool AugularVelocityEstiInteg(geometry_msgs::TwistWithCovarianceStamped& radar_vel) {
            int points_num = 15;
            const int max_iterations = 50;
            double min_cov_points = std::numeric_limits<double>::max();
            
            Eigen::Vector3d radar_vel_vec;
            radar_vel_vec << radar_vel.twist.twist.linear.x, radar_vel.twist.twist.linear.y, radar_vel.twist.twist.linear.z;
            
            // int iter_count = 0;
            for (int iter = 0; iter < max_iterations; ++iter) {
                std::vector<cv::Point2d> selected_points;
                RandomlySelectPoints(TimeImage1, selected_points, points_num);
                // std::cout << "points_num = " << points_num << std::endl;
                // std::cout << "selected_points.size = " << selected_points.size() 
                //         << " points_num = " << points_num << std::endl;
                if(selected_points.size() < points_num)
                {
                    // std::cout << "skip" << std::endl;
                    LOG(WARNING) << "selected points num is not valid!" << std::endl;
                    // return false;
                    continue;
                }
                // assert(points_num == selected_points.size());

                // Eigen::Matrix<double, Eigen::Dynamic, 2> grad_A;

                // grad_A.resize(points_num, 2);  // 在运行时设置矩阵大小
                // Eigen::Matrix<double, Eigen::Dynamic, 1> grad_b;
                // grad_b.resize(points_num, 1);  // 在运行时设置矩阵大小

                // 定义 grad_A 和 grad_b
                // Eigen::Matrix<double, Eigen::Dynamic, 2> grad_A(points_num, 2); // m x 2 矩阵
                // Eigen::Matrix<double, Eigen::Dynamic, 1> grad_b(points_num, 1); // m x 1 向量

                std::vector<Eigen::Vector2d> flow_temp_vec;
                Eigen::MatrixXd grad_A(points_num, 2);
                Eigen::MatrixXd grad_b(points_num, 1);
                
                // std::cout << "Traversal selected_points: " << selected_points.size() << std::endl;
                int rows = 0;
                int zeros_count = 0;
                for (const auto& pt : selected_points) {
                    int x = static_cast<int>(pt.x);
                    int y = static_cast<int>(pt.y);

                    double grad_x = 0;
                    double grad_y = 0;

                    // 梯度计算
                    // output.open("/home/hao/Desktop/output.txt", std::ios::app | std::ios::out);
                    for (int i = -radius; i <= radius; ++i) {
                        for (int j = -radius; j <= radius; ++j) {
                            // 确保不越界
                            if (x + j > 0 && x + j < TimeImage1.cols - 1 && y + i > 0 && y + i < TimeImage1.rows - 1) {
                                // 输出像素值
                                // std::cout << "Pixel value at (" << x + j << ", " << y + i << ") = " 
                                //         << TimeImage1.at<double>(y + i, x + j) << std::endl;
                                // output << "Pixel value at (" << x + j << ", " << y + i << ") = " 
                                //         << TimeImage1.at<double>(y + i, x + j) << std::endl;
                                // 水平方向梯度
                                grad_x += (TimeImage1.at<double>(y + i, x + j + 1) - TimeImage1.at<double>(y + i, x + j - 1)) / 2;
                                // grad_x += (TimeImage1.at<double>(y + i, x + j) - TimeImage1.at<double>(y + i, x + j - 1));
                                // output << "delta_x = " << (TimeImage1.at<double>(y + i, x + j + 1) - TimeImage1.at<double>(y + i, x + j)) << std::endl;
                                // 垂直方向梯度
                                grad_y += (TimeImage1.at<double>(y + i + 1, x + j) - TimeImage1.at<double>(y + i - 1, x + j)) / 2;
                                // grad_y += (TimeImage1.at<double>(y + i, x + j) - TimeImage1.at<double>(y + i - 1, x + j));
                                // output << "delta_y = " << (TimeImage1.at<double>(y + i + 1, x + j) - TimeImage1.at<double>(y + i - 1, x + j)) << std::endl;
                            }
                        }
                        // std::cout << std::endl;  // 分隔每一行像素输出
                    }

                    Eigen::Vector2d grad;
                    grad << grad_x, grad_y;

                    Eigen::Vector3d pixel(pt.x, pt.y, 1.0);
                    Eigen::Vector3d pixel_cord = K.inverse() * pixel;
                    Eigen::Matrix3d pixel_skew = skew(pixel_cord);

                    Eigen::Vector3d flow(valid_flows[i](0) , valid_flows[i](1) , 0); 
                    Eigen::Matrix3d R_re = T_re.block(0, 0, 3, 3);
                    Eigen::Vector3d pre_trans = (pixel_skew * R_re * radar_vel_vec);    // when use, please transpose()
                    Eigen::Vector3d a = pre_trans * pixel_skew;
                    Eigen::Vector3d b = pre_trans * flow;
                    

        for (int i = 0; i < best_inliers.size(); ++i) {
            
            
            
            Eigen::Vector3d prefix = pixel_skew *  * linear_vel;

            
            flow /= event_t1;
            flow = K.inverse() * flow;

            A.row(i) = prefix.transpose() * pixel_skew;
            b(i) = -1.0 * prefix.transpose() * flow;

            // Compute residual for weight
            double residual = (prefix.transpose() * pixel_skew * best_angular_vec + prefix.transpose() * flow).norm();
            W(i, i) = 1.0 / (residual + 1e-6); // Prevent division by zero
        }



                }
            }
        }
    }
    */  

    // 计算角速度
    bool AugularVelocityEsti(geometry_msgs::TwistWithCovarianceStamped& radar_vel) {
        // assert(best_inliers.size() > 2);

        Eigen::MatrixXd A(best_inliers.size(), 3);
        Eigen::VectorXd b(best_inliers.size());
        Eigen::Vector3d linear_vel(radar_vel.twist.twist.linear.x, radar_vel.twist.twist.linear.y, radar_vel.twist.twist.linear.z);

        if(linear_vel.norm() < 1e-6)
        {
            LOG(WARNING) << "radar ego velocity is valid!" << std::endl;
            return false;
        }

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

            Eigen::Vector3d flow(flow_velocity.x, flow_velocity.y, 0);              // TODO: flow is 0??

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

        angular_vec = lpf.filter(angular_vec);

        // 将事件系的角速度投影回雷达系
        angular_vec = T_re.block(0, 0, 3, 3).inverse() * angular_vec;

        radar_vel.twist.twist.angular.x = angular_vec(0);
        radar_vel.twist.twist.angular.y = angular_vec(1);
        radar_vel.twist.twist.angular.z = angular_vec(2);

        // LOG(INFO) << "Calculate Angular Velocity = " << std::endl;
        // LOG(INFO) << "A * w = b \n"
        //           << A << "\n"
        //           << b << "\n" << std::endl; 

        LOG(INFO) << "Angular Velocity = " << std::endl;
        LOG(INFO) << angular_vec.transpose() << std::endl;

        // 计算残差
        Eigen::VectorXd residual = A * angular_vec - b;
        double sigma_r_squared = (residual.squaredNorm()) / (A.rows() - A.cols());
        // LOG(INFO) << "Angular Velocity Residual = " << residual << std::endl;


        // 计算协方差矩阵
        Eigen::Matrix3d covariance_matrix = sigma_r_squared * (A.transpose() * A).inverse();

        // LOG(INFO) << "Angular Velocity covariance_matrix = " << covariance_matrix << std::endl;

        // 将协方差矩阵填充到 Covariance 中
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                radar_vel.twist.covariance[21 + i * 6 + j] = covariance_matrix(i, j);  // 填充角速度的协方差
            }
        }

        // LOG_Velocity(radar_vel);

        return true;
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

            LOG(INFO) << "A(i) = " << A.row(i).transpose() << std::endl;
            LOG(INFO) << "b(i) = " << b(i) << std::endl;
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

        std::fstream optical_flow_file("/media/hao/hao2/228/test/lab/doc/optical_flow.txt", std::ios::out | std::ios::app);
        optical_flow_file << "calculate w: A * w = b " << std::endl;
        optical_flow_file << "inliners: " << best_inliers.size() << std::endl;
        optical_flow_file << "A = " << A << std::endl;
        optical_flow_file << "b = " << b << std::endl;
        optical_flow_file << "res = " << residual.transpose() << std::endl;
        optical_flow_file.close();

        // dji seq outut
        // LOG_Velocity(radar_vel, "/media/hao/hao2/228/test/lab/detector.tum");

        // dvs seq output
        LOG_Velocity(radar_vel, "/home/hao/Desktop/twist_ws/src/TwistEstimator/output/detector.tum");

        return true;
    }


    void ILS_Solver(const std::vector<Eigen::Matrix3d> &A_blocks,
                             const std::vector<Eigen::Vector3d> &b_blocks,
                             Eigen::Vector3d &x, int max_iters = 15, bool verbose = false) {
        assert(A_blocks.size() == b_blocks.size() && "A_blocks.size() != b_blocks.size()");
        
        int n = A_blocks[0].cols();  // 变量维度
        x = Eigen::Vector3d::Zero(n);  // 初始化x

        // if(verbose)
        // {
        //     LOG(ERROR) << " iter\t " <<  "x\t\t\t " <<  "residual\t\t\t "  << "gradient\t\t\t "  << "alpha\t ";
        // }

        double alpha = 0.01;
        Eigen::Matrix3d H;
        H.setIdentity();
        for (int iter = 0; iter < max_iters; ++iter) {
            for (size_t i = 0; i < A_blocks.size(); ++i) {
                Eigen::Matrix3d Ai = A_blocks[i];
                Eigen::Vector3d bi = b_blocks[i];

                // Eigen::Vector3d residual = Ai * x - bi;  // 计算残差
                // Eigen::Vector3d gradient = Ai.transpose() * residual;  // 计算梯度
                // Eigen::Vector3d AiT_grad = Ai.transpose() * gradient;
                // alpha = (gradient.transpose() * gradient).sum() / (AiT_grad.transpose() * AiT_grad).sum();
                // x -= alpha * gradient;  // 迭代更新

                // refer to: https://pdf.sciencedirectassets.com/271700/1-s2.0-S0377221712X00201/1-s2.0-S0377221712006674/main.pdf?X-Amz-Security-Token=IQoJb3JpZ2luX2VjEGIaCXVzLWVhc3QtMSJGMEQCICh3nAvyrjdOH4poLPAp5GAfJh27POHil%2B6WGNyPjANrAiAuJEK%2BqXzYDR1ImpSARlhIHJogu%2BqlFM47aSF6KOUGaiq8BQib%2F%2F%2F%2F%2F%2F%2F%2F%2F%2F8BEAUaDDA1OTAwMzU0Njg2NSIMpxzjbPeGcfwGR%2BCyKpAFI0X4CX5Fay7glUije9puq8iIAQQ3x90JC6XLts%2Fv4Ea62sUl05n9fN2H0k9hHXnJrhDGKn4TogJt21V2M5pUkkz61CYhO6eWqm2gqbsAqW%2FYbyqxPT0811zpP8%2Ft4CGA3yxXcYbO4cFI0AeA%2FqTo8d7CnW8Vm0imyzLTLx9fD%2FrqzLELLherweOTSd49Ti4mnMrRzkEtUhqvnCe5zuGk0KjVSBYyx2AJH6JQdVNsN2L7rdQNpQp8fky9cMXJPdzoaaT3mKsHpAkLaAJD8LqfTDIr1TRxnSdQa%2F4V4ch4NbKt7GmVsRXcCdRDNvOURNpDJ3meI1QzB2nkntzrFMsPNIMrdhrly3l3dWjcjGdl0ILpHsJpes1f4r6eyp14MnLkHWuRjABslmi6XCYd0ucwFQ2pTOSAHh0aAJI2EJ1KSk2RbI1BDJRHY6BZvseSC4%2BWi0ZBYsTrQVEy5P8lSSbo%2B7mau670Ri5%2F1X4htbcOamvyCIR%2FlrCIAueXtcbFPotBsDTngb565gPnUVS6YbuPnoxT5oVsVr1vV8zRbxedq8WkDO%2FvpoL1BAE0Dz9uM15RrP2dhcmktuoh%2Fsslkmg%2Bvx8sfUtomd0rbr10%2BU19A47EmZwekj8r8XvIQ7Q0pJkHHyHy2cEpGgU3vtGt7VOFEMEctOzRQ%2BlAQcodKKj2AoEs2uQmsFuuzlxFTLT%2B40zJScdtIwbWjz4poDSl38fkgI2FvgOyHIE19Mrk80xiaW7ma2dVIMxYrtcqupvywZ157V%2BoB%2FuxsW68AEc3mpJsya3uB2ndadAzMfR7rdRHoN7dxOMA6jYhxUIQYcnT8GNLLQ%2FYN22lcK%2Bffzf9x1LLK07z0%2BpifaDbyXXJcK13wx0wks2JvgY6sgHjWmVpunkaeVKmVnqikXnnfKpqUfq1Iz%2FuX5OiUffIN2HE11AQ%2FYJkHP5ogwdT1%2BUBhPjss3GKQ%2Bs61coIRsaAcXhIOq%2B9F2D9j567coGwwwWXSlUgBJPfJFGJWxXTqLO%2BlHrj1MseR7GAnapDlDDxBSrEQdCGbGe3m1oUaM3Jsrb2Efj%2FQW41DjHxonmaMENOambPce%2BXHl8Cg3pneaThmxzSXcjDsZzvHprn%2BvhHuXlz&X-Amz-Algorithm=AWS4-HMAC-SHA256&X-Amz-Date=20250301T030344Z&X-Amz-SignedHeaders=host&X-Amz-Expires=300&X-Amz-Credential=ASIAQ3PHCVTY664YXLIU%2F20250301%2Fus-east-1%2Fs3%2Faws4_request&X-Amz-Signature=093eaa456f9a36d568f28a0e4189efa68c96117b9a87b599c1fde45f83160a43&hash=38b076485dbc18983d908513e70f0cad16a6eef65ec2d56b55ce39c0d1fae23a&host=68042c943591013ac2b2430a89b270f6af2c76d8dfd086a07176afe7c76c2c61&pii=S0377221712006674&tid=spdf-1b62f755-f44d-41a2-b103-0474d03b901d&sid=53fb5a1747b57542468b4be10d041ea2b4a0gxrqa&type=client&tsoh=d3d3LnNjaWVuY2VkaXJlY3QuY29t&rh=d3d3LnNjaWVuY2VkaXJlY3QuY29t&ua=101e5c525856530c5152&rr=91956c4499d55eb4&cc=jp
                {
                    Eigen::Vector3d residual = bi - Ai * x;
                    Eigen::Matrix3d S = H * Ai.transpose();
                    H = H - (S * S.transpose()) * (Eigen::Matrix3d::Identity() + Ai * S).inverse();
                    x += ((H * Ai.transpose()) * residual);
                    // H = H - (S * S.transpose()) / (Eigen::Matrix3d::Identity() + Ai * S);
                    

                    // H = H - (S * S.transpose()).array() * (Eigen::Vector3d::Ones() + Ai * S).array().cwiseInverse().matrix();
                    LOG(ERROR) << "residual = " << residual.transpose() << std::endl;
                    LOG(ERROR) << "H = " << H << std::endl;
                    LOG(ERROR) << "(H * Ai) = " << (H * Ai) << std::endl;
                    LOG(ERROR) << "x = " << x.transpose() << std::endl;
                }
                // if(verbose)
                // {
                //     LOG(ERROR) << " " << iter << "\t " <<  "\t " << x <<  "\t " 
                //                 << residual.transpose() << "\t " << gradient.transpose() << "\t " << alpha;
                // }
            }
        }
    }

    // 计算角速度
    bool ModLSQAugularVelocityEsti(geometry_msgs::TwistWithCovarianceStamped& radar_vel) {
        // assert(best_inliers.size() > 2);

        // Eigen::MatrixXd A(best_inliers.size(), 3);
        // Eigen::VectorXd b(best_inliers.size());
        Eigen::MatrixXd A(3 * best_inliers.size(), 3);
        Eigen::VectorXd b(3 * best_inliers.size());

        Eigen::Vector3d linear_vel(radar_vel.twist.twist.linear.x, radar_vel.twist.twist.linear.y, radar_vel.twist.twist.linear.z);

        if(linear_vel.norm() < 1e-6)
        {
            LOG(WARNING) << "radar ego velocity is valid!" << std::endl;
            return false;
        }

        LOG(ERROR) << "Mid Debug for flow: " << best_inliers.size() << std::endl;

        LOG(ERROR) << "K.inverse() = " << K.inverse() << std::endl;

        // 3-1 修改
        std::vector<Eigen::Matrix3d> A_blocks;
        std::vector<Eigen::Vector3d> b_blocks;

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
        
            
            // Eigen::Vector3d flow(flow_pre_points[i].x, flow_pre_points[i].y, 0);              // TODO: flow is 0??
            double normal_flow = normal_flow_pre_points[i];
            Eigen::Vector2d grad = normal_pre_points[i];
            Eigen::Vector3d normal_vec(grad(0), grad(1), 0);
            
            LOG(ERROR) << "normal_flow = " << normal_flow << std::endl;
            LOG(ERROR) << "normal_vec = " << normal_vec << std::endl;

            // LOG(INFO) << "flow = " << flow.transpose() << std::endl;

            // TODO: 投影到归一化场景                 
            // flow(0) = flow(0) / K(0,0);
            // flow(1) = flow(1) / K(1,1);            
            // flow = K.inverse() * flow;
            // 这个的原因是线性映射
            normal_vec = K.inverse() * (normal_flow * normal_vec); // 恢复法向光流,并转到归一化平面
            // 然后再次归一化
            normal_flow = normal_vec.norm();
            normal_vec = normal_vec / normal_flow;

            LOG(ERROR) << "normal_flow(coordinate) = " << normal_vec << std::endl;
            LOG(ERROR) << "normal_vec(coordinate) = " << normal_vec << std::endl;

            // LOG(INFO) << "flow cordinate = " << flow.transpose() << std::endl;

            // A.row(i) = prefix * normal_vec.transpose() * pixel_skew;
            // b.row(i) = -1.0 * prefix * normal_flow;    //TODO: modify in paper

            LOG(ERROR) << "Ai = " << prefix * normal_vec.transpose() * pixel_skew << std::endl;
            LOG(ERROR) << "bi = " << -1.0 * prefix * normal_flow << std::endl;

            A_blocks.push_back(prefix * normal_vec.transpose() * pixel_skew);
            b_blocks.push_back(-1.0 * prefix * normal_flow);

            A.block(3 * i, 0, 3, 3) = prefix * normal_vec.transpose() * pixel_skew;
            b.block(3 * i, 0, 3, 1) = -1.0 * prefix * normal_flow;

            LOG(INFO) << "A(i) = " << A.block(3 * i, 0, 3, 3).transpose() << std::endl;
            LOG(INFO) << "b(i) = " << b.block(3 * i, 0, 3, 1).transpose() << std::endl;
        }

        // Eigen::Vector3d angular_vec = A.colPivHouseholderQr().solve(b);


        Eigen::Vector3d angular_vec;
        angular_vec.setZero();
        // 求解角速度
        ILS_Solver(A_blocks, b_blocks, angular_vec, 15, true);

        // angular_vec = lpf.filter(angular_vec);

        LOG(ERROR) << "First Angular Velocity = " << std::endl;
        LOG(ERROR) << angular_vec.transpose() << std::endl;


        LOG(ERROR) << "CHECK: A = " << A;
        LOG(ERROR) << "CHECK: b = " << b;
        /*{
        {
            LOG(ERROR) << "CHECK:" << std::endl;

            assert(A_blocks.size() == b_blocks.size());
            Eigen::MatrixXd A_test(3 * A_blocks.size(), 3);
            Eigen::VectorXd b_test(3 * b_blocks.size());
            for(size_t in = 0; in < A_blocks.size(); in++)
            {
                A_test.block(3 * in, 0, 3, 3) = A_blocks[in];
                b_test.block(3 * in, 0, 3, 1) = b_blocks[in];
            }
            LOG(ERROR) << "CHECK: A_test = " << A_test;
            LOG(ERROR) << "CHECK: b_test = " << b_test;
            
            LOG(ERROR) << "CHECK: A = " << A;
            LOG(ERROR) << "CHECK: b = " << b;
            int iteration = 25;

            // 加权最小二乘法

            angular_vec = (A.transpose() * A).colPivHouseholderQr().solve(A.transpose() * b);
            for(int i = 0; i < iteration; i++)
            {
                Eigen::VectorXd residual = A * angular_vec - b;
                Eigen::VectorXd weights = residual.array().inverse();
                Eigen::DiagonalMatrix<double, Eigen::Dynamic> diag_residual(weights.size());
                diag_residual.diagonal() = weights;
                angular_vec = (A.transpose() * diag_residual * A).colPivHouseholderQr().solve(A.transpose() * diag_residual * b);
            
                LOG(ERROR) << "residual = " << residual << std::endl;
                LOG(ERROR) << "angular_vec = " << angular_vec << std::endl;
            }
            
            LOG(ERROR) << "Concentrate First Angular Velocity = " << std::endl;
            LOG(ERROR) << angular_vec.transpose() << std::endl;
        }
        }*/


        {
            int iteration = 25;
            int select_point = 10;
            int min_cov = 10000;
            std::vector<int> best_flow_inlier_idxs;

            LOG(ERROR) << "A_blocks.size() = " << A_blocks.size() << std::endl;

            for(int iter = 0; iter < iteration; iter++)
            {
                std::random_device rd;
                std::mt19937 gen(rd());
                std::uniform_int_distribution<int> dis(0, A_blocks.size() - 1);

                // 1. 随机选择 `select_point` 个数据点
                std::vector<int> sample_idx;
                while (sample_idx.size() < select_point)
                {
                    int idx = dis(gen);
                    if (std::find(sample_idx.begin(), sample_idx.end(), idx) == sample_idx.end())
                        sample_idx.push_back(idx);
                }

                // 2. 构造 A*x = b
                Eigen::MatrixXd A_temp(select_point * 3, 3);
                Eigen::VectorXd b_temp(select_point * 3);
                for (int j = 0; j < select_point; j++)
                {
                    A_temp.block<3, 3>(j * 3, 0) = A_blocks[sample_idx[j]];
                    b_temp.segment<3>(j * 3) = b_blocks[sample_idx[j]];
                }

                LOG(ERROR) << "A_temp = " << A_temp << ", " << b_temp << std::endl;

                // 3. 计算临时估计
                Eigen::Vector3d temp_model = A_temp.colPivHouseholderQr().solve(b_temp);
                LOG(ERROR) << "temp_model = " << temp_model.transpose() << std::endl;

                // 4. 计算所有数据点的误差
                int inlier_count = 0;
                double res_inliers = 0;
                std::vector<int> inlier_idx;
                for (size_t i = 0; i < A_blocks.size(); i++)
                {
                    Eigen::Vector3d residual = A_blocks[i] * temp_model - b_blocks[i];
                    res_inliers += residual.norm();
                    // if (residual.norm() < 1.0)
                    if ((residual.array().abs() < 1.0).all())
                    {
                        inlier_count++;
                        inlier_idx.push_back(i);
                    }
                }

                if(min_cov > res_inliers)
                {
                    min_cov = res_inliers;
                    best_flow_inlier_idxs = inlier_idx;
                }
            }

            // 
            int flow_inliers_size = best_flow_inlier_idxs.size();
            Eigen::MatrixXd A_temp(flow_inliers_size * 3, 3);
            Eigen::VectorXd b_temp(flow_inliers_size * 3);
            for (int j = 0; j < flow_inliers_size; j++)
            {
                A_temp.block<3, 3>(j * 3, 0) = A_blocks[best_flow_inlier_idxs[j]];
                b_temp.segment<3>(j * 3) = b_blocks[best_flow_inlier_idxs[j]];
            }    
            Eigen::Vector3d temp_model = A_temp.colPivHouseholderQr().solve(b_temp);
            Eigen::VectorXd residual = A_temp * temp_model - b_temp;
            // Eigen::VectorXd weights = residual.array().inverse();
            Eigen::VectorXd weights = (residual.array().abs() > 1.0).select(1.0 / residual.array().abs(), 1.0);
            Eigen::DiagonalMatrix<double, Eigen::Dynamic> diag_residual(weights.size());
            diag_residual.diagonal() = weights;
            // angular_vec = (diag_residual * A_temp).colPivHouseholderQr().solve(diag_residual * b_temp);
            angular_vec = (A_temp.transpose() * diag_residual * A_temp)
              .colPivHouseholderQr()
              .solve(A_temp.transpose() * diag_residual * b_temp);
        }
        LOG(ERROR) << "Second angular_vec = " << angular_vec.transpose() << std::endl;
            
        /*{
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
        }*/

        // 将事件系的角速度投影回雷达系
        angular_vec = T_re.block(0, 0, 3, 3).transpose() * angular_vec;

        radar_vel.twist.twist.angular.x = angular_vec(0);
        radar_vel.twist.twist.angular.y = angular_vec(1);
        radar_vel.twist.twist.angular.z = angular_vec(2);

        // 计算协方差矩阵
        /*{
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
        }*/
        // 将协方差矩阵填充到 Covariance 中
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                if (i == j)
                    radar_vel.twist.covariance[21 + i * 6 + j] = 1.0;  // 填充角速度的协方差
            }
        }
        
        /*{
        std::fstream optical_flow_file("/media/hao/hao2/228/test/lab/doc/optical_flow.txt", std::ios::out | std::ios::app);
        optical_flow_file << "calculate w: A * w = b " << std::endl;
        optical_flow_file << "inliners: " << best_inliers.size() << std::endl;
        optical_flow_file << "A = " << A << std::endl;
        optical_flow_file << "b = " << b << std::endl;
        optical_flow_file << "res = " << residual.transpose() << std::endl;
        optical_flow_file.close();
        }*/

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


    void LOG_Velocity(geometry_msgs::TwistWithCovarianceStamped& radar_vel)
    {
        // 创建一个字符串流
        std::stringstream ss;

        // 记录线速度
        ss << "Linear Velocity: "
        << "x = " << radar_vel.twist.twist.linear.x << ", "
        << "y = " << radar_vel.twist.twist.linear.y << ", "
        << "z = " << radar_vel.twist.twist.linear.z << "\n";

        // 记录角速度
        ss << "Angular Velocity: "
        << "x = " << radar_vel.twist.twist.angular.x << ", "
        << "y = " << radar_vel.twist.twist.angular.y << ", "
        << "z = " << radar_vel.twist.twist.angular.z << "\n";

        // 记录协方差矩阵
        ss << "Covariance Matrix:\n";
        for (int i = 0; i < 6; ++i) {
            for (int j = 0; j < 6; ++j) {
                ss << radar_vel.twist.covariance[i * 6 + j] << " ";
            }
            ss << "\n";
        }

        // 使用 glog 记录信息
        LOG(INFO) << ss.str();
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

            // 基于固定时间的
            // CompressTimeImage(process_time_sec);
            // bool accumu_done = true;

            // 基于累积帧的计算
            bool accumu_done = AccumulateTimeImage(process_time_sec);

            if(!accumu_done)
                return false;
            
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
            // if(!(have_flow && LSQAugularVelocityEsti(twist_)))

            if(!(have_flow && ModLSQAugularVelocityEsti(twist_)))
            {
                twist_.twist.twist.angular.x = 0.0f;
                twist_.twist.twist.angular.y = 0.0f;
                twist_.twist.twist.angular.z = 0.0f;
                LOG(ERROR) << "Angular Estimation Failed!" << std::endl;
                // 角速度不可用,尽保留线速度
                // return false;
            } 

            // PublishTimeImages(TimeImage1);
            PublishTimeImages2(TimeImage1);

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
            twist_result2_.push_back(TwistData2(twist_, cur_inliers, best_inliers, flow_pre_points));

            // 清空本次数据
            best_inliers.clear();
            flow_pre_points.clear();

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

    std::deque<dvs_msgs::EventArray> event_stream;
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
    cv::Mat distort_cv;
    Eigen::Matrix3d K;      // intrincs matrix
    Eigen::Matrix3d distort;      // distort parameters
    int sensor_width;
    int sensor_height;
    Eigen::Matrix4d T_re;

    cv::Mat TimeImage1;
    cv::Mat TimeImage2;

    cv::Mat TimeImage1_color;

    cv::Mat TimeImage1_on_event;
    cv::Mat TimeImage1_off_event;
    
    event_flow_velocity flow_velocity;
    // std::vector<double*> flow_velocity;
    std::vector<event_flow_velocity> flow_pre_points;
    int radius;

    // 2-28 修改
    std::vector<Eigen::Vector2d> normal_pre_points;
    std::vector<double> normal_flow_pre_points;

    std::vector<cv::Point2d> best_inliers;

    long int t1_image_count;
    long int t2_image_count;

    bool show_events_ = false;

    geometry_msgs::TwistWithCovarianceStamped twist_;

    LowPassFilter lpf;

    LowPassFilter lpf_v;

    int filter_num;

    int median_radius;
    bool use_gauss;
    double ratio_inliers;

    bool ignore_polarity;

    ros::NodeHandle nh_this_;
    cv_bridge::CvImage cv_image_;           // transport TimeImage
    ros::Publisher image_pub_;

    ros::Time process_time; 

    ros::Publisher pub_event_image_;

    // 存储所有有效光流
    std::vector<Eigen::Vector2d> valid_flows;  

    sensor_msgs::PointCloud2 cur_inliers;

    // void RandomlySelectPoints(std::vector<cv::Point2d> src_points, std::vector<cv::Point2d>& dst_points, int& points_num) {
    //     std::sample(src_points.begin(), src_points.end(), std::back_inserter(dst_points),
    //             15, std::mt19937{std::random_device{}()});
    // }

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
