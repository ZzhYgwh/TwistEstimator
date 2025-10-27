#pragma once

// #include "../../include/radar-event/EventDetector.hpp"

#include "radar_ego_velocity_estimator/radar_ego_velocity_estimator.h"

// #include "event_flow_detector/event_flow_detector.h"

// #include "event_flow_detector/event_ct_flow.h"

// #include "event_flow_detector/event_ct_flow_LP.h"
#include "event_flow_detector/event_ct_flow_sae.h"

#include "estimator/TwistEstimator.h"

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/conversions.h>  // For toPCL

#include <tf/transform_broadcaster.h>

// Global
std::mutex detector_data_mutex;

// EventFlowDetector* event_detector_;

// EventFlowDetector3* event_detector_;
SAEFlowDetector* event_detector_;

geometry_msgs::TwistWithCovarianceStamped twist;
geometry_msgs::TwistWithCovarianceStamped last_twist;

ros::Publisher pub_twist_2;
ros::Publisher pub_twist_;
ros::Publisher pub_inlier_;
ros::Publisher pub_outlier_;
ros::Publisher twist_traj_marker_pub_;
EventParams event;
RadarParams radar;

std::shared_ptr<tf::TransformBroadcaster> br_ptr_;

ros::Publisher pub_marker_;

ros::Publisher pub_marker_array_;

std::deque<sensor_msgs::PointCloud2::Ptr> radar_buffer;
std::deque<dvs_msgs::EventArray::Ptr> event_buffer;
std::deque<sensor_msgs::Image::Ptr> image_buffer;
std::mutex callback_mutex;

TwistEstimator* estimator_;

/**
 * @brief 处理雷达点云数据的回调函数。
 * 
 * 该函数接收来自雷达传感器的点云数据，处理并进行后续操作，如数据过滤、可视化等。
 * 
 * @param radar_msgs_ptr 指向 `sensor_msgs::PointCloud2` 类型的指针，包含从雷达传感器接收到的点云数据。
 */
void RadarCallback(sensor_msgs::PointCloud2::Ptr radar_msgs_ptr)
{
    if (radar_msgs_ptr == nullptr)
        return;

    // 只在ti_mmwave时修改字段，避免每次都遍历
    if (radar.type == "ti_mmwave")
    {
        // 逆序查找更有效,doppler往往存在末尾
        for (auto it = radar_msgs_ptr->fields.rbegin(); it != radar_msgs_ptr->fields.rend(); ++it)
        {
            if (it->name == "velocity")
            {
                it->name = "doppler";  // 将 velocity 改为 doppler
                break;  // 只修改一次，退出循环
            }
        }
    }

    // 将数据添加到缓冲区
    {
        std::lock_guard<std::mutex> lock(callback_mutex);
        radar_buffer.push_back(radar_msgs_ptr);  // 传递指针，避免拷贝
    }

    // 可选：仅在需要调试时输出日志
    // LOG(ERROR) << "Catch Radar Data" << std::endl;
}

/**
 * @brief 将雷达点云数据保存到文本文件。
 * 
 * 该函数将传入的雷达点云数据 `sensor_msgs::PointCloud2` 转换为文本格式，并保存到指定的文件中。
 * 
 * @param radar_msgs_copy 要保存的雷达点云数据，类型为 `sensor_msgs::PointCloud2`。
 * @param filename 目标文件路径，保存点云数据的文件名（包括扩展名）。
 * 
 * @note 保存的数据格式为每行一个点，包含 `x`, `y`, `z` 坐标和 `intensity`（强度）信息。
 * 
 * @warning 确保提供有效的文件路径和足够的写权限，以避免文件写入错误。
 */
void savePointCloud2ToTxt(const sensor_msgs::PointCloud2& radar_msgs_copy, const std::string& filename) 
{
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

/**
 * @brief 将 ROS PointCloud2 消息转换为 PCL 点云数据结构。
 * 
 * 该函数将传入的 ROS 点云消息（`sensor_msgs::PointCloud2`）中的点坐标和多普勒信息提取出来，并填充到 PCL 点云数据结构（`pcl::PointCloud<pcl::PointXYZI>`）中。
 * 
 * @param ros_msg 输入的 ROS PointCloud2 消息，包含雷达点云数据。该消息包含每个点的 `x`、`y`、`z` 坐标以及 `doppler` 信息。
 * @param cloud 输出的 PCL 点云数据，包含转换后的点云数据。点云中的每个点包含 `x`、`y`、`z` 坐标和 `intensity`（多普勒信息）。
 * 
 * @note 该函数假设 ROS 消息中的 `doppler` 数据已经存储在与 `intensity` 字段相对应的位置，并会将其赋值给 PCL 点云中的 `intensity` 字段。
 */
/*void customFromROSMsg(const sensor_msgs::PointCloud2& ros_msg, pcl::PointCloud<pcl::PointXYZI>& cloud) {
    cloud.points.clear();
    cloud.width = ros_msg.width;
    cloud.height = ros_msg.height;
    cloud.is_dense = ros_msg.is_dense;
    cloud.points.reserve(ros_msg.width * ros_msg.height);

    // 准备 iterators
    sensor_msgs::PointCloud2ConstIterator<float> iter_x(ros_msg, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iter_y(ros_msg, "y");
    sensor_msgs::PointCloud2ConstIterator<float> iter_z(ros_msg, "z");
    sensor_msgs::PointCloud2ConstIterator<float> iter_doppler(ros_msg, "doppler");

    for (size_t i = 0; i < ros_msg.width * ros_msg.height; ++i, ++iter_x, ++iter_y, ++iter_z, ++iter_doppler) {
        pcl::PointXYZI pt;
        pt.x = *iter_x;
        pt.y = *iter_y;
        pt.z = *iter_z;
        pt.intensity = *iter_doppler;  // 将 doppler 映射到 intensity
        cloud.points.push_back(pt);
    }
}*/
void customFromROSMsg(const sensor_msgs::PointCloud2& ros_msg, pcl::PointCloud<pcl::PointXYZI>& cloud) 
{
    // 直接构造新的数据，避免悬挂引用
    pcl::PointCloud<pcl::PointXYZI> temp_cloud;
    temp_cloud.width = ros_msg.width;
    temp_cloud.height = ros_msg.height;
    temp_cloud.is_dense = ros_msg.is_dense;
    temp_cloud.points.reserve(ros_msg.width * ros_msg.height);

    // 使用 iterators 从 ROS msg 中提取字段
    sensor_msgs::PointCloud2ConstIterator<float> iter_x(ros_msg, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iter_y(ros_msg, "y");
    sensor_msgs::PointCloud2ConstIterator<float> iter_z(ros_msg, "z");
    sensor_msgs::PointCloud2ConstIterator<float> iter_doppler(ros_msg, "doppler");

    for (size_t i = 0; i < ros_msg.width * ros_msg.height; ++i, ++iter_x, ++iter_y, ++iter_z, ++iter_doppler) 
    {
        pcl::PointXYZI pt;
        pt.x = *iter_x;
        pt.y = *iter_y;
        pt.z = *iter_z;
        pt.intensity = *iter_doppler;
        temp_cloud.points.push_back(pt);
    }

    // 替换掉原来的 cloud 内容（更安全）
    cloud = std::move(temp_cloud);
}



double visualize_marker_scale;
double visualize_doppler_scale;

// 发布点云和矢量
/**
 * @brief 可视化雷达点云及其多普勒矢量。
 * 
 * 该函数用于可视化给定的雷达点云数据，并根据每个点的多普勒信息计算反向矢量。每个点的矢量方向基于点的坐标（`x`, `y`, `z`），其大小则由多普勒信息（`doppler`）控制。 
 * 在视觉化过程中，函数生成并发布包含点云和矢量的标记信息。
 * 
 * @param inlier_radar_msg 输入的雷达点云数据（`sensor_msgs::PointCloud2` 类型）。包含点云信息，如 `x`、`y`、`z` 坐标和每个点的多普勒信息（`doppler`），用于计算矢量。
 * 
 * @note 该函数假定输入的点云消息包含每个点的 `x`、`y`、`z` 坐标及其 `doppler` 信息，且基于此信息计算并显示每个点的矢量和几何形状（例如三角形和线条）。
 * 
 * @note `doppler` 信息用于调整矢量的大小，并在可视化时显示为从点的位置出发的线条，代表每个点的速度方向。
 */
void visualizePointCloudAndVectors(const sensor_msgs::PointCloud2& inlier_radar_msg) 
{
    // 将 ROS PointCloud2 消息转换为 PCL 点云类型
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>());
    // pcl::fromROSMsg(inlier_radar_msg, *cloud);  // 转换到 PCL 格式

    customFromROSMsg(inlier_radar_msg, *cloud);

    visualization_msgs::MarkerArray marker_array;
    marker_array.markers.reserve(cloud->size() * 2);

    int id_counter = 0;  // 每个 Marker 需要唯一 ID
    // tri_size 是边长
    float tri_size = visualize_marker_scale;  // 例如 0.2
    float r = tri_size / sqrt(3);  // 外接圆半径，保证三角形边长为 tri_size
    LOG(ERROR) << "visualize_marker_scale = " << visualize_marker_scale;
    LOG(ERROR) << "r = " << r;
    float height = sqrt(2.0 / 3.0) * tri_size;
    // double cos_0 = 1.0;
    // double sin_0 = 0.0;
    double cos_120 = cos(2.0 * M_PI / 3.0);
    double sin_120 = sin(2.0 * M_PI / 3.0);
    int scale = 4;
    for (size_t i = 0; i < cloud->points.size(); ++i) 
    {
        const pcl::PointXYZI& point = cloud->points[i];
        float x = scale * point.x;
        float y = scale * point.y;
        float z = scale * point.z;
        float doppler = point.intensity;

        float magnitude = std::sqrt(x * x + y * y + z * z);
        if (magnitude < 1e-6) continue;  // 忽略原点附近点，避免除以 0

        Eigen::Vector3f direction(x / magnitude, y / magnitude, z / magnitude);
        Eigen::Vector3f reverse_vector = visualize_doppler_scale * direction * doppler;

        // ---- 1. 创建反向矢量 LINE_LIST Marker ----
        visualization_msgs::Marker line_marker;
        line_marker.header.frame_id = "radar";
        line_marker.header.stamp = ros::Time::now();
        line_marker.ns = "doppler_vectors";
        line_marker.id = id_counter++;
        line_marker.type = visualization_msgs::Marker::LINE_LIST;
        line_marker.action = visualization_msgs::Marker::ADD;
        line_marker.pose.orientation.w = 1.0;
        line_marker.scale.x = 0.5;
        line_marker.color.r = 1.0;
        line_marker.color.a = 1.0;

        geometry_msgs::Point start_point, end_point;
        start_point.x = x;
        start_point.y = y;
        start_point.z = z;
        end_point.x = x + reverse_vector.x();
        end_point.y = y + reverse_vector.y();
        end_point.z = z + reverse_vector.z();

        line_marker.points.push_back(start_point);
        line_marker.points.push_back(end_point);

        marker_array.markers.push_back(line_marker);

        // ---- 2. 创建三角形（三个 POINTS 构成一个 TRIANGLE_LIST Marker）----
        visualization_msgs::Marker triangle_marker;
        triangle_marker.header = line_marker.header;
        triangle_marker.ns = "triangle_marker";
        triangle_marker.id = id_counter++;
        triangle_marker.type = visualization_msgs::Marker::TRIANGLE_LIST;
        triangle_marker.action = visualization_msgs::Marker::ADD;
        triangle_marker.pose.orientation.w = 1.0;
        triangle_marker.scale.x = 1.0;
        triangle_marker.scale.y = 1.0;
        triangle_marker.scale.z = 1.0;
        triangle_marker.color.g = 1.0;
        triangle_marker.color.a = 1.0;
        
        {
            // 底面三角形（XY平面）
            geometry_msgs::Point p0, p1, p2, p3;

            // 使用极坐标分布三个点（120 度间隔）
            p0.x = x + r;
            p0.y = y;
            p0.z = z;

            p1.x = x + r * cos_120;
            p1.y = y + r * sin_120;
            p1.z = z;

            p2.x = x + r * cos_120;
            p2.y = y - r * sin_120;
            p2.z = z;

            // 顶点 p3 — 沿 z 方向上抬

            p3.x = x;
            p3.y = y;
            p3.z = z + height;

            // 添加底面
            triangle_marker.points.push_back(p0);
            triangle_marker.points.push_back(p1);
            triangle_marker.points.push_back(p2);

            // 添加三条侧面
            triangle_marker.points.push_back(p0);
            triangle_marker.points.push_back(p1);
            triangle_marker.points.push_back(p3);

            triangle_marker.points.push_back(p1);
            triangle_marker.points.push_back(p2);
            triangle_marker.points.push_back(p3);

            triangle_marker.points.push_back(p2);
            triangle_marker.points.push_back(p0);
            triangle_marker.points.push_back(p3);
        }

        marker_array.markers.push_back(triangle_marker);
    }

    // 发布 MarkerArray
    pub_marker_array_.publish(marker_array);
    marker_array.markers.clear();
    // 发布点云
    pub_inlier_.publish(inlier_radar_msg);

    // ROS_INFO("Published point cloud and vectors.");
}

// This part of the code is adapted from: [4DRadarSLAM: ]   
// std::vector<double> egovel_time;
rio::RadarEgoVelocityEstimator estimator;
/**
 * @brief 估算雷达点云中的多普勒信息。
 * 
 * 该函数从传入的雷达点云数据中提取点云信息，并使用这些信息估算每个点的多普勒效应。多普勒效应反映了物体相对雷达的速度，进而可以推算出物体的运动方向和速度。 
 * 函数通过计算点云中的每个点的速度和相对运动来估算多普勒信息。
 * 
 * @param radar_msgs_ptr_copy 传入的雷达点云数据（`sensor_msgs::PointCloud2` 类型的智能指针）。该数据包含雷达扫描得到的每个点的坐标（`x`、`y`、`z`）信息。
 * 
 * @note 该函数假定输入的点云消息已经包含所需的点云数据，并且雷达测量已经经过预处理以确保数据的准确性。 
 * 估算的多普勒信息可以用于进一步的运动分析或用于计算目标的速度。
 * 
 * @todo 需要根据具体的多普勒估算方法实现详细的计算步骤，并考虑如何处理噪声和误差。
 */
bool DopplerEstimation(const sensor_msgs::PointCloud2::Ptr radar_msgs_ptr_copy)
{
    // assert(radar_msgs_ptr_copy->fields[4].name == "doppler" && "missing field name 'doppler'");

    // [step1]: radar ego estimation
    geometry_msgs::TwistWithCovarianceStamped radar_twist;
    Eigen::Vector3d v_r, sigma_v_r;
    sensor_msgs::PointCloud2 inlier_radar_msg, outlier_radar_msg;
    clock_t start_ms = clock();

    // savePointCloud2ToTxt(radar_msgs_copy, "/media/hao/hao2/228/test/radar_doppler_data.txt");

    if (estimator.estimate(*radar_msgs_ptr_copy, v_r, sigma_v_r, inlier_radar_msg, outlier_radar_msg)) 
    {
        clock_t end_ms = clock();
        double time_used = double(end_ms - start_ms) / CLOCKS_PER_SEC;
        // egovel_time.push_back(time_used);
        // ROS_WARN("Radar Estimation Successed!");
        // radar_twist.header.stamp = radar_msgs_copy.header.stamp;

        // radar_twist.header.stamp = radar_msgs_ptr->header.stamp;
        radar_twist.header.stamp = radar_msgs_ptr_copy->header.stamp;
        radar_twist.header.frame_id = "radar";
        radar_twist.twist.twist.linear.x = v_r.x();
        radar_twist.twist.twist.linear.y = v_r.y();
        radar_twist.twist.twist.linear.z = v_r.z();

        radar_twist.twist.covariance.at(0)  = std::pow(sigma_v_r.x(), 2);
        radar_twist.twist.covariance.at(7)  = std::pow(sigma_v_r.y(), 2);
        radar_twist.twist.covariance.at(14) = std::pow(sigma_v_r.z(), 2);

        if (pub_twist_2.getNumSubscribers() > 0)
        {
            pub_twist_2.publish(radar_twist);
        }

        // LOG(ERROR) << "pub_twist_.getNumSubscribers() = " << pub_twist_.getNumSubscribers() << std::endl; 
        // if (pub_twist_.getNumSubscribers() > 0)
        // {
        //     pub_twist_.publish(radar_twist);
        // }

        // if (pub_inlier_.getNumSubscribers() > 0)
        // {
        //     pub_inlier_.publish(inlier_radar_msg);
        // }


        LOG(ERROR) << "test visualizePointCloudAndVectors" << std::endl;
         auto start = std::chrono::high_resolution_clock::now();
        if(inlier_radar_msg.width > 0)
            visualizePointCloudAndVectors(inlier_radar_msg);
        auto end = std::chrono::high_resolution_clock::now();
        double duration_ms = std::chrono::duration<double, std::milli>(end - start).count();
        LOG(ERROR) << "[Time] visualizePointCloudAndVectors took " 
                << duration_ms << " ms" << std::endl;
        // HAO TODO: 测试耗时

        // pub_outlier_.publish(outlier_radar_msg);

        // LOG(ERROR) << "inlier_radar_msg.size = " << inlier_radar_msg.width << std::endl;

        LOG(ERROR) << "Radar Estimation: " << std::setprecision(8) << radar_twist.twist.twist.linear.x << ", " 
                    << radar_twist.twist.twist.linear.y << ", " << radar_twist.twist.twist.linear.z << std::endl;

        // HAO: 数据交换
        {
            // std::lock_guard<std::mutex> lock(detector_data_mutex);
            event_detector_->radar_doppler_velocity.push_back(radar_twist);
            event_detector_->radar_inliers.push_back(inlier_radar_msg);
        } 
        return true;
    }
    LOG(ERROR) << "Doppler Estimation Failed" << std::endl;
    // HAO TODO: missing wrong data
    return false;
}

std::fstream event_file;
/**
 * @brief 处理来自事件传感器的事件数据。
 * 
 * 该回调函数处理传入的事件数据（如动态视觉传感器（DVS）事件流），并执行必要的事件处理操作。通常，这包括从事件流中提取有用的视觉信息， 
 * 并将其用于后续的图像处理或视觉任务，如光流计算、物体跟踪等。
 * 
 * @param event_msgs_ptr 传入的事件数据消息（`dvs_msgs::EventArray` 类型的智能指针）。该数据包含时间戳和事件类型（包括每个事件的空间位置和极性）。
 * 
 * @note 本函数假设传入的事件数据已经准备好，并且根据需要对数据进行实时处理。具体实现细节可能会根据任务需求而有所不同。
 * 
 * @todo 进一步优化事件数据的处理速度和精度，考虑如何减少噪声对事件流分析的影响。
 */
void EventCallback(dvs_msgs::EventArray::Ptr event_msgs_ptr)
{
    // std::lock_guard<std::mutex> lock(event_callback_mutex);

    if(event_msgs_ptr == nullptr)
        return ;

    // HAO TODO:不需要检查时序了
    /*
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
    }*/


    // LOG(ERROR) << "event_msgs_ptr.size = " << event_msgs_ptr->events.size() << std::endl;
    // LOG(ERROR) << "event_msgs output = " << event_msgs_ptr->events.size() / 
    //                         (event_msgs_ptr->events.back().ts.toSec() - event_msgs_ptr->events.front().ts.toSec()) << " p/s" << std::endl;

    event_file.open("/home/hao/Desktop/radar-event-new/src/TwistEstimator/output/events.csv", std::ios::out | std::ios::app);
    event_file << event_msgs_ptr->events.size() / (event_msgs_ptr->events.back().ts.toSec() - event_msgs_ptr->events.front().ts.toSec()) << std::endl;
    event_file.close();

    // HAO: 数据交换
    {
        // std::lock_guard<std::mutex> lock(detector_data_mutex);
        // event_detector_->event_stream.push_back(*event_msgs_ptr);

        std::lock_guard<std::mutex> lock(callback_mutex);
        // LOG(ERROR) << "Catch Event Data" << std::endl;
        event_buffer.push_back(event_msgs_ptr);
    }

}

/**
 * @brief 处理图像数据的回调函数。
 * 
 * 该回调函数处理传入的图像数据，通常用于对图像进行进一步处理，如图像转换、图像分析或特征提取等任务。
 * 在图像处理中，该函数可能会被用于从传感器接收图像并进行一些实时的操作，如图像滤波、特征提取或显示。
 * 
 * @param image_msg_ptr 传入的图像消息（`sensor_msgs::Image` 类型的智能指针）。该数据包含图像的原始信息，如图像的宽度、高度、编码格式、数据等。
 * 
 * @note 本函数假定传入的图像数据已经准备好，并且根据需要对数据进行实时处理。具体的图像处理任务将依据实际需求进行调整。
 * 
 * @todo 优化图像处理算法，减少延迟，提高实时性，增加更多图像处理方法，如边缘检测、物体识别等。
 */
void ImageCallback(sensor_msgs::Image::Ptr image_msg_ptr)
{
    if(image_msg_ptr == nullptr)
        return ;
    // HAO: 数据交换
    {
        // std::lock_guard<std::mutex> lock(detector_data_mutex);
        // event_detector_->event_stream.push_back(*event_msgs_ptr);

        std::lock_guard<std::mutex> lock(callback_mutex);
        // LOG(ERROR) << "Catch Event Data" << std::endl;
        image_buffer.push_back(image_msg_ptr);

        // LOG(ERROR) << "image_buffer.size = " << image_buffer.size() << std::endl;
    }
}

void ImuCallback(sensor_msgs::Imu::Ptr imu_msg_ptr)
{
    if(imu_msg_ptr == nullptr)
        return ;
    // HAO: 数据交换
    {
        // std::lock_guard<std::mutex> lock(detector_data_mutex);
        // event_detector_->event_stream.push_back(*event_msgs_ptr);

        std::lock_guard<std::mutex> lock(callback_mutex);
        // LOG(ERROR) << "Catch Event Data" << std::endl;
        estimator_->imu_buffer.push_back(imu_msg_ptr);

        // LOG(ERROR) << "image_buffer.size = " << image_buffer.size() << std::endl;
    }
}

// Function to parse the YAML file and retrieve parameters
/**
 * @brief 解析 YAML 配置文件并将内容存储到相应的参数对象中。
 * 
 * 该函数从指定的 YAML 配置文件中加载参数，并将其解析到不同的参数对象中，以便在程序的其他部分中使用。这些参数包括事件处理、雷达配置、事件相关配置以及图像处理等相关设置。
 * 
 * @param filename 配置文件的路径和文件名（例如 "config.yaml"）。该文件应该包含所需的参数和设置。
 * @param event 事件处理相关的参数，将从 YAML 文件中加载到该对象中。
 * @param radar 雷达配置参数，将从 YAML 文件中加载到该对象中。
 * @param radar_event 雷达事件相关的参数，将从 YAML 文件中加载到该对象中。
 * @param show_events 一个布尔值，指示是否显示事件（从 YAML 文件加载）。
 * @param smooth 平滑系数，用于图像或数据处理的平滑操作（从 YAML 文件加载）。
 * @param filter_num 用于事件滤波的参数，表示过滤的数量或阈值（从 YAML 文件加载）。
 * @param median_radius 中值滤波器的半径，用于事件数据滤波（从 YAML 文件加载）。
 * @param ignore_polarity 一个布尔值，指示是否忽略事件的极性（从 YAML 文件加载）。
 * @param use_gauss 一个布尔值，指示是否使用高斯滤波器（从 YAML 文件加载）。
 * @param ratio_inliers 点云或事件数据中内点的比例，用于匹配或滤波（从 YAML 文件加载）。
 * @param cam_topic 相机主题（话题名称），用于从 ROS 订阅的相机图像流中获取数据。
 * 
 * @return 如果 YAML 文件成功加载并解析，返回 `true`；否则返回 `false`。
 * 
 * @note 本函数假定 YAML 文件的格式正确，并且包含必要的参数。若文件格式不正确或缺少某些参数，则可能导致解析失败。
 * 
 * @todo 增强错误处理，添加对 YAML 文件格式不正确或参数缺失的更多检查。
 */
bool ParseYaml(const std::string& filename, EventParams& event, RadarParams& radar, 
                    RadarEventParams& radar_event, bool& show_events, double& smooth, int& filter_num, 
                    int & median_radius, bool & ignore_polarity, bool& use_gauss, double & ratio_inliers,
                    double & grid_size,
                    std::string& cam_topic, std::string& imu_topic)
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
        event.sae_time = config["event"]["sae_time"].as<double>();

        // event.resolution = config["event"]["resolution"].as<int[2]>();
        std::array<int, 2> resolution = config["event"]["resolution"].as<std::array<int, 2>>();
        event.resolution[0] = resolution[0];
        event.resolution[1] = resolution[1];

        event.deltaT = config["event"]["deltaT"].as<double>();
        event.t1 = config["event"]["t1"].as<double>();
        event.mesh_size = config["event"]["mesh_size"].as<int>();
        LOG(ERROR) << "mesh size = " << event.mesh_size << std::endl;
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

        visualize_marker_scale = config["visualize_marker_scale"].as<double>();
        visualize_doppler_scale = config["visualize_doppler_scale"].as<double>();

        ratio_inliers = config["ratio_inliers"].as<double>();
        LOG(ERROR) << "ratio_inliers = " << ratio_inliers << std::endl;

        grid_size = config["grid_size"].as<double>();

        LOG(ERROR) << "radar modify radar_config: " << config["radar_config"].as<std::string>() << std::endl;
        estimator.LOAD(config["radar_config"].as<std::string>());

        cam_topic = config["cam_topic"].as<std::string>();

        LOG(ERROR) << "cam_topic = " << cam_topic << std::endl;

        imu_topic = config["imu_topic"].as<std::string>();

        LOG(ERROR) << "imu_topic = " << imu_topic << std::endl;
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
std::mutex estimate_mutex;
std::deque<TwistData2> estimate_buffer;



visualization_msgs::Marker generateInstantTwistTrajectoryGradient(
    const geometry_msgs::TwistWithCovarianceStamped& twist,
    double dt_total = 0.4,
    double dt_step  = 0.01,
    const std::string& frame_id = "radar",
    double line_width = 6)
{
    visualization_msgs::Marker marker;
    marker.header.frame_id = frame_id;
    marker.header.stamp = ros::Time::now();
    marker.ns = "twist_body_traj";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::LINE_STRIP;
    marker.action = visualization_msgs::Marker::ADD;
    marker.scale.x = line_width;

    tf::Vector3 pos(0,0,0);
    tf::Quaternion q = tf::createIdentityQuaternion();

    size_t steps = static_cast<size_t>(dt_total / dt_step);
    tf::Vector3 w(
        twist.twist.twist.angular.x,
        twist.twist.twist.angular.y,
        twist.twist.twist.angular.z
    );
    tf::Vector3 v_body(
        80 * twist.twist.twist.linear.x,
        80 * twist.twist.twist.linear.y,
        80 * twist.twist.twist.linear.z
    );

    for (size_t i = 0; i <= steps; ++i)
    {
        double t_ratio = static_cast<double>(i) / steps; // 0~1

        // 渐变色：蓝色(0,0,1) -> 红色(1,0,0)
        std_msgs::ColorRGBA c;
        c.r = 0.0;                  // 蓝->绿 没有红色
        c.g = t_ratio;               // 绿色从 0 -> 1
        c.b = 1.0 - t_ratio;         // 蓝色从 1 -> 0
        c.a = 1.0;


        // 积分角速度更新姿态
        double dt = dt_step;
        double angle = w.length() * dt;
        if (angle > 1e-12) {
            tf::Vector3 axis = w.normalized();
            tf::Quaternion dq;
            dq.setRotation(axis, angle);
            q *= dq;
        }

        // 将机体线速度转换到世界坐标系
        q.normalize();
        tf::Matrix3x3 R(q);
        tf::Vector3 v_world = R * v_body;

        // 积分更新位置
        pos += v_world * dt;

        geometry_msgs::Point pt;
        pt.x = pos.x();
        pt.y = pos.y();
        pt.z = pos.z();
        marker.points.push_back(pt);

        marker.colors.push_back(c); // 对应每个点颜色
    }

    return marker;
}

// 独立线程对象
// detector_thread;
/**
 * @brief 执行主程序的功能，启动必要的操作并处理系统的主要流程。
 * 
 * 该函数是程序的入口点，负责启动并运行主流程。具体实现可以包括初始化系统、加载配置、启动多个线程或服务，并协调不同模块之间的交互。它可能会调用其他模块来执行数据处理、传感器数据采集、事件处理等任务。
 * 
 * @note 该函数通常在程序的启动阶段被调用，确保所有必要的资源和依赖都已初始化，并开始程序的主操作。
 * 
 * @todo 根据需要添加异常处理、错误日志记录等功能。
 */
void RUN()
{
    // std::deque<dvs_msgs::EventArray::Ptr> event_msgs_ptr_buffer;
    // std::deque<sensor_msgs::PointCloud2::Ptr> radar_msgs_ptr_buffer;
    // std::deque<sensor_msgs::Image::Ptr> img_msgs_ptr_buffer;
    while (ros::ok())
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));  // 控制循环频率，避免过高的 CPU 占用
        // 与回调交换
        // if(detector_thread != busy)
        {
            sensor_msgs::PointCloud2::Ptr radar_msgs_ptr;
            // radar_msgs_ptr->width = -1;
            dvs_msgs::EventArray::Ptr event_msgs_ptr_copy;
            // event_msgs_copy.width = 0;

            auto run_start = std::chrono::high_resolution_clock::now();
            // 回调交换接口
            // 事件一般触发更快
            {
                std::lock_guard<std::mutex> lock(callback_mutex);
                if (radar_buffer.empty() && event_buffer.empty()) // 都不存在数据集
                {
                    continue;
                }


                if (!event_buffer.empty())
                {
                    // // 数据交换
                    // // {     
                    //     // 提取事件数据
                        event_msgs_ptr_copy = event_buffer.front();
                        event_buffer.pop_front();     
                        // LOG(ERROR) << "Event Buffer = " << event_buffer.size() << std::endl;  
                    // // } 
                }

                if (!radar_buffer.empty())
                {
                    radar_msgs_ptr = radar_buffer.front();
                    radar_buffer.pop_front();
                    LOG(ERROR) << "Doppler Buffer = " << radar_buffer.size() << std::endl;
                }  
                // else
                // {
                //     LOG(ERROR) << "No Event Input" << std::endl;
                // }

                /*{
                    
                    event_msgs_ptr_buffer.swap(event_buffer);
                    radar_msgs_ptr_buffer.swap(radar_buffer);
                    if(!image_buffer.empty())
                        img_msgs_ptr_buffer.swap(image_buffer);
                }*/


                while(!image_buffer.empty())
                {
                    event_detector_->raw_img_buffer.push_back(image_buffer.front());
                    image_buffer.pop_front();
                }
            }
            // LOG(ERROR) << "Data Stream Input" << std::endl;
            

            // 事件数据
            // if(event_msgs_copy.width > 0)
            if(event_msgs_ptr_copy != nullptr)
            {
                // LOG(ERROR) << "event_msgs_copy.width = " << event_msgs_copy.width << std::endl;
                // LOG(ERROR) << "event_msgs_copy.sec = " << event_msgs_copy.header.stamp.toSec() << std::endl;
                event_detector_->event_stream.push_back(event_msgs_ptr_copy); // HAO TODO: 修改 event_msgs_copy -> event_msgs_ptr_copy
            }
            // else
            // {
            //     LOG(ERROR) << "No Event Input" << std::endl;
            //     continue;
            // }  
            auto run_catch = std::chrono::high_resolution_clock::now();

            // 多普勒估计
            // if (radar_msgs.width != -1)
            bool detetor_flag = false;
            if(radar_msgs_ptr != nullptr)
            {
                // LOG(ERROR) << "DopplerEstimation" << std::endl;
                // LOG(ERROR) << "radar_msgs_copy.width = " << radar_msgs_ptr->width << std::endl;
                // auto doppler_start = std::chrono::high_resolution_clock::now();
                detetor_flag = DopplerEstimation(radar_msgs_ptr);    
                if(!detetor_flag)
                {
                    LOG(ERROR) << "DopplerEstimation Failed" << std::endl;
                }
                /*else
                {
                    auto doppler_end = std::chrono::high_resolution_clock::now();
                    std::chrono::duration<double, std::milli> duration_ms = doppler_end - doppler_start;
                    std::fstream doppler_time("/home/hao/Desktop/radar-event-new/src/TwistEstimator/output/doppler_T.csv", std::ios::out | std::ios::app);
                    doppler_time << duration_ms.count() << std::endl;
                    doppler_time.close();
                }  */  
            }
            // else
            // {
            //     LOG(ERROR) << "No Doppler Input" << std::endl;
            //     continue;
            // }   
           auto run_doppler = std::chrono::high_resolution_clock::now();

            // 传感器数据传入Detector
            /*
            LOG(ERROR) << "import sensor data into detector" << std::endl;
            {
                event_detector_->event_stream.insert(
                    event_detector_->event_stream.end(),
                    std::make_move_iterator(event_msgs_ptr_buffer.begin()),
                    std::make_move_iterator(event_msgs_ptr_buffer.end())
                );
                event_msgs_ptr_buffer.clear();

                while(!radar_msgs_ptr_buffer.empty())
                {
                   DopplerEstimation(radar_msgs_ptr_buffer.front());
                   radar_msgs_ptr_buffer.pop_front();
                }

                event_detector_->raw_img_buffer.insert(
                    event_detector_->raw_img_buffer.end(),
                    std::make_move_iterator(img_msgs_ptr_buffer.begin()),
                    std::make_move_iterator(img_msgs_ptr_buffer.end())
                );

                event_msgs_ptr_buffer.clear();
                img_msgs_ptr_buffer.clear();
            }*/

            // std::lock_guard<std::mutex> lock(detector_mutex);
            // LOG(ERROR) << "Detector" << std::endl;
            TwistData2 twist2;
            // if (detetor_flag && event_detector_->Detector())
            std::chrono::time_point<std::chrono::high_resolution_clock> run_detector;
            if (event_detector_->Detector()) 
            {
                run_detector = std::chrono::high_resolution_clock::now();

                twist = event_detector_->GetTwist();
                // LOG(ERROR) << std::setprecision(18) << "twist time diff = " << (twist.header.stamp - last_twist.header.stamp).toSec() << std::endl;
                last_twist = twist;
                if (pub_twist_.getNumSubscribers() > 0)
                {
                    pub_twist_.publish(twist);
                }
                // 填充 twist_msg（瞬时机体速度和角速度）

                visualization_msgs::Marker traj_marker =
                    generateInstantTwistTrajectoryGradient(twist);

                twist_traj_marker_pub_.publish(traj_marker);

                // tf::Transform transform;
                // transform.setOrigin(tf::Vector3(0.0, 0.0, 0.0));
                // // 设置旋转：绕 X 轴旋转 -45 度（pitch）
                // tf::Quaternion q;
                // double pitch_rad = -45.0 * M_PI / 180.0;
                // q.setRPY(pitch_rad, 0.0, 0.0); // RPY顺序: roll, pitch, yaw
                // q.normalize();  // 保证单位四元数
                // transform.setRotation(q);
                // br_ptr_->sendTransform(tf::StampedTransform(transform, ros::Time::now(), "body", "radar"));

                // 发布一个tf关于body 转 radar pitch -45 度

                // pub_twist_.publish(twist);

                // std::ofstream detector_vel_file("/home/hao/Desktop/radar-event-new/src/TwistEstimator/output/detector2.tum",
                //                                 std::ios::out | std::ios::app);
                // detector_vel_file << std::setprecision(20) << twist.header.stamp.toSec() << " "
                //                 << twist.twist.twist.linear.x << " " << twist.twist.twist.linear.y << " " << twist.twist.twist.linear.z << " "
                //                 << twist.twist.twist.angular.x << " " << twist.twist.twist.angular.y << " " << twist.twist.twist.angular.z << std::endl;
                // detector_vel_file.close();

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
                assert(twist2.timestamp == twist.header.stamp.toSec() && "Detector: Not Same");
                // 3-18 修改
                {
                    std::lock_guard<std::mutex> lock(estimate_mutex);
                    estimate_buffer.push_back(twist2);
                }
                // LOG(ERROR) << "Push_Data" << std::endl;
            }
            auto run_end = std::chrono::high_resolution_clock::now();

            //
            {
                std::chrono::duration<double, std::milli> duration_ms = run_end - run_start;
                LOG(ERROR) << "run total: " << duration_ms.count() << " ms" << std::endl;
                duration_ms = run_doppler - run_catch;
                LOG(ERROR) << "run catch: " << duration_ms.count() << " ms" << std::endl;
                duration_ms = run_detector - run_doppler;
                LOG(ERROR) << "run doppler: " << duration_ms.count() << " ms" << std::endl;
                duration_ms = run_end - run_detector;
                LOG(ERROR) << "run detector: " << duration_ms.count() << " ms" << std::endl;
                // run_catch run_doppler run_detector
            }

            // else        // Doppler 估计失败 或 Event Flow估计失败
            // {
            //     twist2 = TwistData2(event_detector_->GetProcessTime());
            //     LOG(ERROR) << "Event Angular Failed" << std::endl;
            // }

            // // 3-18 修改
            // {
            //     std::lock_guard<std::mutex> lock(estimate_mutex);
            //     estimate_buffer.push_back(twist2);
            //     LOG(ERROR) << "Push_Data" << std::endl;
            // }
        }      
    }
}


// 后端程序
/**
 * @brief 执行估计任务的主函数，启动估计流程并处理相关数据。
 * 
 * 该函数负责执行估计过程的主要任务。通常，它会初始化相关参数、加载必要的数据、启动估计算法，并协调不同模块之间的交互。函数可能会进行数据处理、模型训练、算法优化等操作。
 * 
 * @note 该函数应在系统的估计模块初始化后调用，用于启动和管理整个估计过程。
 * 
 * @todo 根据需要优化算法的计算效率，添加错误处理和日志记录功能。
 */
void Esti_RUN()
{
    std::deque<TwistData2> estimate_buffer_temp;
    // std::vector<TwistData2> estimate_buffer_temp;
    while (ros::ok())
    {
        auto esti_start = std::chrono::high_resolution_clock::now();
        
        if(estimate_buffer_temp.empty())
        {
            // 交换窗口
            estimate_mutex.lock();
            if(estimate_buffer.empty())     // 前端没有数据
            {
                estimate_mutex.unlock();
                // std::this_thread::sleep_for(std::chrono::milliseconds(2));  // 控制循环频率，避免过高的 CPU 占用
                // LOG(ERROR) << "Estimate: No Valid Data" << std::endl;
                // continue;
            }
            else                            // 前端有数据
            {
                estimate_buffer_temp.swap(estimate_buffer);          // 一次全部交换，降低交换频率
                // estimate_buffer_temp.insert(estimate_buffer_temp.end(), 
                //                 estimate_buffer.begin(), estimate_buffer.end());
                // estimate_buffer.clear();
                estimate_mutex.unlock();
                // continue; // 下一次进行
            }

            // LOG(ERROR) << "estimate_buffer_temp.size = " << estimate_buffer_temp.size() << std::endl;
            continue;
        }
        else
        {
            // LOG(ERROR) << "estimate_buffer_temp.size = " << estimate_buffer_temp.size() << std::endl;
            
            TwistData2 twist = estimate_buffer_temp.front();
            /*{
            LOG(ERROR) << "Get Front End Data " << std::endl;
            LOG(ERROR) << "Timestamp = " << std::setprecision(20) << twist.timestamp << std::endl;
            LOG(ERROR) << "linear_vel_vec_ = " << twist.linear_vel_vec_.transpose() << std::endl;
            LOG(ERROR) << "angular_vel_vec_ = " << twist.angular_vel_vec_.transpose() << std::endl;
            LOG(ERROR) << "best_inliers.size = " << twist.best_inliers.size() << std::endl;
            LOG(ERROR) << "best_flow.size = " << twist.best_flow.size() << std::endl;
            LOG(ERROR) << "point_cloud.size = " << twist.point_cloud.width << std::endl;
            // estimator_->twist2_vec_.push_back(twist);
            // estimate_buffer_temp.pop_front();
            }*/

            // LOG(ERROR) << "twist.timestamp = " << std::setprecision(20) << twist.timestamp << std::endl;

            estimator_->PushEstimation(twist);
            estimate_buffer_temp.pop_front();
        }

        // if(!estimator_->twist2_vec_.empty())
        // estimator_->Estimate2();   // 只调试前端

        // estimator_->Estimate3();      // 速度空间的优化
        
        // estimator_->LooseEstimate();     // 松耦合优化
        // estimator_->Local_Estimate2();

        // estimator_->Estimate2(twist);   // 只调试前端
        // estimate_buffer_temp.pop_front();
        // LOG(ERROR) << "Estimate " << std::endl;
        auto esti_end = std::chrono::high_resolution_clock::now();
        //
        {
            std::chrono::duration<double, std::milli> duration_ms = esti_end - esti_start;
            LOG(ERROR) << "esti total: " << duration_ms.count() << " ms" << std::endl;

            // run_catch run_doppler run_detector
        }
    } 
}


int main(int argc,char ** argv)
{
    google::InitGoogleLogging(argv[0]);
    FLAGS_logbufsecs = 0; // 实时写入
    // 指定日志文件的保存路径
    // FLAGS_log_dir = "/home/hao/Desktop/radar-event-new/src/TwistEstimator/../glog_log";
    FLAGS_log_dir = "/home/hao/Desktop/twist_ws/src/log";
    // FLAGS_minloglevel = google::;
    // FLAGS_logtostderr = true;

    ros::init(argc,argv,"test_detector");
    ros::NodeHandle nh;

    std::string cam_topic;
    std::string imu_topic;
    RadarEventParams radar_event;
    radar_event.T_re = Eigen::Matrix4d::Identity();  // Initialize to identity

    // "/home/hao/Desktop/radar-event-new/src/TwistEstimator/config/params.yaml";
    // const std::string filename = "/home/hao/Desktop/radar-event-new/src/TwistEstimator/config/dji.yaml";
    // const std::string filename = "/home/hao/Desktop/twist_ws/src/TwistEstimator/config/dji2.yaml";
    // const std::string filename = "/home/hao/Desktop/radar-event-new/src/TwistEstimator/config/dvs.yaml";
    // const std::string filename = "/home/hao/Desktop/radar-event-new/src/TwistEstimator/config/demo_228.yaml";
    // const std::string filename = "/home/hao/Desktop/twist_ws/src/TwistEstimator/config/dji2.yaml";

    // 2025 - 10 - 16 修改
    // const std::string filename = "/home/hao/Desktop/twist_ws/src/TwistEstimator/config/dji10.yaml";
    // const std::string filename = "/home/hao/Desktop/twist_ws/src/TwistEstimator/config/lab.yaml";
    const std::string filename = "/home/hao/Desktop/twist_ws/src/TwistEstimator/config/road.yaml";
    bool show_events = false;
    double smooth = 0.1;

    int filter_num = 3;
    int median_radius = 5;
    bool ignore_polarity = false;
    bool use_gauss = false;
    double ratio_inliers = 0.1;
    double grid_size = 15;
    ParseYaml(filename, event, radar, radar_event, show_events, smooth, filter_num, median_radius, ignore_polarity, use_gauss, ratio_inliers, grid_size, cam_topic, imu_topic);
    std::cout << "smooth = " << smooth << std::endl;

    LOG(ERROR) << "radar.topic = " << radar.topic << std::endl;
    ros::Subscriber radar_sub_ = nh.subscribe<sensor_msgs::PointCloud2::Ptr>(radar.topic, 100000, RadarCallback);
    LOG(ERROR) << "event.topic = " << event.topic << std::endl;
    ros::Subscriber event_sub_ = nh.subscribe<dvs_msgs::EventArray::Ptr>(event.topic, 100000, EventCallback);
    LOG(ERROR) << "cam_topic = " << cam_topic << std::endl;
    ros::Subscriber image_sub_ = nh.subscribe<sensor_msgs::Image::Ptr>(cam_topic, 100000, ImageCallback);
    LOG(ERROR) << "imu_topic = " << imu_topic << std::endl;
    ros::Subscriber imu_sub_ = nh.subscribe<sensor_msgs::Imu::Ptr>(imu_topic, 100000, ImuCallback);

    // radar_buffer.reserve(5000);

    pub_twist_ = nh.advertise<geometry_msgs::TwistWithCovarianceStamped>("radar/twist", 1000);
    pub_twist_2 = nh.advertise<geometry_msgs::TwistWithCovarianceStamped>("radar/twist_raw", 1000);
    pub_inlier_ = nh.advertise<sensor_msgs::PointCloud2>("radar/inliers", 1000);
    // pub_marker_ = nh.advertise<visualization_msgs::Marker>("radar/inliers_markers", 1000);

    twist_traj_marker_pub_ = nh.advertise<visualization_msgs::Marker>("/twist/predict/traj", 1000);

    pub_marker_array_ = nh.advertise<visualization_msgs::MarkerArray>("radar/inliers_markers", 1000);
    pub_outlier_ = nh.advertise<sensor_msgs::PointCloud2>("radar/outliers", 1000);

    br_ptr_ = std::make_shared<tf::TransformBroadcaster>();

    // event_detector_ = new EventFlowDetector(nh, event, radar_event, show_events, smooth, filter_num, median_radius, ignore_polarity, use_gauss, ratio_inliers);
    // event_detector_ = new EventFlowDetector3(nh, event, radar_event, show_events, smooth, filter_num, ignore_polarity, ratio_inliers);
    event_detector_ = new SAEFlowDetector(nh, event, radar_event, show_events, smooth, filter_num, ignore_polarity, ratio_inliers, grid_size);
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

    // std::thread estimate_thread(Esti_RUN);
    // estimate_thread.detach();

    // 使用AsyncSpinner来处理回调
    ros::AsyncSpinner spinner(3); // 启动2个线程
    spinner.start();
    // 主线程继续做其他事情
    ros::waitForShutdown();

    // ros::spin();

    // detect_thread.join();  // 等待线程安全退出
    google::FlushLogFiles(google::GLOG_INFO);

    google::ShutdownGoogleLogging();  // 在程序结束时关闭 Google Logging

    return 0;
}