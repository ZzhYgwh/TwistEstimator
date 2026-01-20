#pragma once

#include <glog/logging.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <dvs_msgs/Event.h>
#include <dvs_msgs/EventArray.h>
#include <image_geometry/pinhole_camera_model.h>

#include "sae.h"
#include "event_flow_detector.h"

#include <sensor_msgs/point_cloud2_iterator.h>

#include <omp.h>

class SAEFlowDetector
{
public:
    SAEFlowDetector(ros::NodeHandle& nh_this, EventParams& e_params, RadarEventParams r_params, bool show_events, double smooth, int filter_num = 3, bool ignore_polarity = false, double ratio_inliers = 0.1, double grid_size = 15)
        : nh_this_(nh_this), sensor_width(e_params.resolution[0]), ignore_polarity(ignore_polarity), sae_time(e_params.sae_time),
        sensor_height(e_params.resolution[1]), t1_image_count(e_params.t1_count),  T_re(r_params.T_re), radius(e_params.mesh_size),
        grid_size(grid_size)
        // event_dt(e_params.deltaT), event_t1(e_params.t1), lpf(smooth), lpf_v(smooth), filter_num(filter_num), t2_image_count(e_params.t2_count), ratio_inliers(ratio_inliers), show_events_(show_events) 
        {
            K << e_params.fx, 0, e_params.cx,
                0, e_params.fy, e_params.cy,
                0,0,1;

            LOG(ERROR) << "K = " << K;

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

            pub_cloud = nh_this.advertise<sensor_msgs::PointCloud2>("/event/flow/image", 10);

            event_buffer.reserve(100000);
            raw_img_buffer.reserve(100000);
            
            cam_info.height = sensor_height;
            cam_info.width = sensor_width;
            // cam_info.K = K_cv;
            std::copy(K_cv.ptr<double>(), K_cv.ptr<double>() + 9, cam_info.K.begin());

            // 直接赋值
            cam_info.distortion_model = "plumb_bob";
            cam_info.D = {e_params.k1, e_params.k2, e_params.p1, e_params.p2, e_params.k3};

            // LOG(ERROR) << "Distortion Coefficients: ";
            for (size_t i = 0; i < cam_info.D.size(); i++) {
                // LOG(ERROR) << cam_info.D[i] << " ";
            }
            // LOG(ERROR) << std::endl;

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

            // Init SAE
            sae_ptr_ = std::make_shared<SAE>(sensor_width, sensor_height);

            kernel_ = cv::getStructuringElement(cv::MORPH_RECT,
                                                        cv::Size(2 * radius + 1, 2 * radius + 1));
                                                        // cv::Size(4 * radius + 1, 4 * radius + 1));
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

   /* void AccumulateTimeImage()
    {
        for(auto& event : event_buffer)
        {
            sae_ptr_->addEvent(event);  // Note: this is ROS time
        }
    }*/

    // void AccumulateTimeImage()
    // {
    //     auto& sae_on  = sae_ptr_->getSAE(1);
    //     auto& sae_off = sae_ptr_->getSAE(0);

    //     const int width  = sae_on.cols;
    //     const int height = sae_on.rows;

    //     // #pragma omp parallel for schedule(static)
    //     for (int i = 0; i < static_cast<int>(event_buffer.size()); ++i)
    //     {
    //         const auto& e = event_buffer[i];
    //         int x = e.x;
    //         int y = e.y;
    //         if ((unsigned)x >= (unsigned)width || (unsigned)y >= (unsigned)height)
    //             continue;

    //         // 根据事件极性选择对应的 SAE
    //         if (e.polarity)
    //             sae_on.at<double>(y, x) = e.ts.toSec();
    //         else
    //             sae_off.at<double>(y, x) = e.ts.toSec();
    //     }
    // }

    void AccumulateTimeImage()
    {
        auto& sae_on  = sae_ptr_->getSAE(1);
        auto& sae_off = sae_ptr_->getSAE(0);

        const int w = sae_on.cols;
        const int h = sae_on.rows;

        // 必须是 double*
        double* on_ptr  = sae_on.ptr<double>();
        double* off_ptr = sae_off.ptr<double>();

        const int N = static_cast<int>(event_buffer.size());

        for (int i = 0; i < N; ++i)
        {
            const auto& e = event_buffer[i];
            const int x = e.x;
            const int y = e.y;

            if ((unsigned)x >= (unsigned)w || (unsigned)y >= (unsigned)h)
                continue;

            const int idx = y * w + x;
            const double t = e.ts.toSec();

            if (e.polarity)
                on_ptr[idx] = t;
            else
                off_ptr[idx] = t;
        }
    }


    // 构建反对称矩阵
    Eigen::Matrix3d skew(const Eigen::Vector3d& vec) const {
        Eigen::Matrix3d result;
        result << 0, -vec(2), vec(1), vec(2), 0, -vec(0), -vec(1), vec(0), 0;
        return result;
    }

    std::vector<int> RandomSample(int N, int sample_num) {
        std::vector<int> idx(N);
        std::iota(idx.begin(), idx.end(), 0);
        std::random_device rd;
        std::mt19937 g(rd());
        std::shuffle(idx.begin(), idx.end(), g);
        return std::vector<int>(idx.begin(), idx.begin() + sample_num);
    }

    // void RansacPlaneFit(const Eigen::MatrixXd& A, double threshold,
    //                     Eigen::Vector4d& best_normal,
    //                     int max_iterations = 8) {

    //     int N = A.rows();
    //     int max_inlier_count = 0;
    //     std::vector<int> best_inliers_index;
    //     best_inliers_index.clear();
    //     Eigen::MatrixXd A_sample(4,4);

    //     for (int iter = 0; iter < max_iterations; ++iter) {
    //         auto sample_idx = RandomSample(N, 4);
            
    //         for (int i=0; i<4; ++i)
    //             A_sample.row(i) = A.row(sample_idx[i]);

    //         // Eigen::JacobiSVD<Eigen::MatrixXd> svd(A_sample, Eigen::ComputeFullV);
    //         // Eigen::Vector4d normal = svd.matrixV().col(3);
    //         // 10-18 修改
    //         Eigen::Matrix4d ATA = A_sample.transpose() * A_sample;
    //         Eigen::SelfAdjointEigenSolver<Eigen::Matrix4d> eig(ATA);
 
    //         Eigen::Vector4d normal = eig.eigenvectors().col(0); // 最小特征值对应列
    //         double denom = normal.head<3>().norm();

    //         double inv_norm = 1.0 / denom;
    //         /*
    //         std::vector<int> current_inliers;
    //         for (int i=0; i<N; ++i) {
    //             Eigen::Vector4d pt = A.row(i);
    //             double dist = std::abs(normal.dot(pt)) / normal.head<3>().norm();
    //             // 10-18 修改

    //             // LOG(ERROR) << "dist = " << dist << std::endl;
    //             if (dist < threshold)
    //                 current_inliers.push_back(i);
    //         }*/
    //         // 10-18 修改
    //         Eigen::VectorXd dists = (A * normal).cwiseAbs() * inv_norm;

    //         std::vector<int> current_inliers;
    //         current_inliers.reserve(N);
    //         for (int i = 0; i < N; ++i)
    //             if (dists(i) < threshold) current_inliers.push_back(i);

            
    //         if ((int)current_inliers.size() > max_inlier_count) {
    //             max_inlier_count = current_inliers.size();
    //             best_inliers_index = current_inliers;
    //             // best_normal = normal;

    //             // LOG(ERROR) << "A_sample = " << A_sample << std::endl;
    //             // LOG(ERROR) << "normal = " << A_sample << std::endl;
    //             // LOG(ERROR) << "max_inlier_count = " << max_inlier_count << std::endl;
    //         }
    //     }
        
    //     int best_N = best_inliers_index.size();
        
    //     Eigen::MatrixXd A_best(best_N,4);
    //     if(best_N != 0)
    //         for (int i=0; i<best_N; ++i)
    //             A_best.row(i) = A.row(best_inliers_index[i]);
    //     else
    //     {
    //         A_best.resize(A.rows(), A.cols());
    //         A_best = A;
    //     }

    //     // LOG(ERROR) << "A_best = " << A_best << std::endl;
    //     Eigen::JacobiSVD<Eigen::MatrixXd> svd(A_best, Eigen::ComputeFullV);
    //     best_normal = svd.matrixV().col(3);

    //     // LOG(ERROR) << "best_inliers_index.size = " << best_inliers_index.size() << std::endl;
    //     // LOG(ERROR) << "best_normal = " << best_normal.transpose() << std::endl;

    //     // LOG(ERROR) << "fit error = " << A_best * best_normal << std::endl;
    // }

    // 2025-12-15 修改此处
    void RansacPlaneFit(const Eigen::MatrixXd& A,
                    double threshold,
                    Eigen::Vector4d& best_normal,
                    int max_iterations = 8)
    {
        std::chrono::time_point<std::chrono::high_resolution_clock> flow_ransac_start = std::chrono::high_resolution_clock::now();
        const int N = A.rows();
        if (N < 4) {
            best_normal.setZero();
            return;
        }

        int max_inlier_count = 0;
        std::vector<int> best_inliers_index;
        best_inliers_index.reserve(N);

        Eigen::Matrix4d A_sample;                 // 固定大小
        std::vector<int> current_inliers;
        current_inliers.reserve(N);

        for (int iter = 0; iter < max_iterations; ++iter)
        {
            // ---------- 1. 随机采样 ----------
            auto sample_idx = RandomSample(N, 4);
            for (int i = 0; i < 4; ++i)
                A_sample.row(i) = A.row(sample_idx[i]);

            // ---------- 2. 估计平面法向 ----------
            Eigen::Matrix4d ATA = A_sample.transpose() * A_sample;
            Eigen::SelfAdjointEigenSolver<Eigen::Matrix4d> eig(ATA);
            Eigen::Vector4d normal = eig.eigenvectors().col(0); // 最小特征值

            const double denom = normal.head<3>().norm();
            if (denom < 1e-8)     // 数值保护
                continue;

            const double inv_norm = 1.0 / denom;

            // ---------- 3. 计算内点（手写，避免 A*normal） ----------
            current_inliers.clear();

            const double nx = normal(0);
            const double ny = normal(1);
            const double nz = normal(2);
            const double d  = normal(3);

            for (int i = 0; i < N; ++i)
            {
                const Eigen::Vector4d& r = A.row(i);
                const double dist =
                    std::abs(r(0)*nx + r(1)*ny + r(2)*nz + d) * inv_norm;

                if (dist < threshold)
                    current_inliers.push_back(i);
            }

            // ---------- 4. 更新最优模型 ----------
            if ((int)current_inliers.size() > max_inlier_count)
            {
                max_inlier_count = current_inliers.size();
                best_inliers_index = current_inliers;
            }
        }

        // ---------- 5. 使用最优内点重新拟合 ----------
        Eigen::MatrixXd A_best;
        if (!best_inliers_index.empty())
        {
            const int best_N = best_inliers_index.size();
            A_best.resize(best_N, 4);
            for (int i = 0; i < best_N; ++i)
                A_best.row(i) = A.row(best_inliers_index[i]);
        }
        else
        {
            // fallback：全体
            A_best = A;
        }

        // ---------- 6. SVD 精修 ----------
        Eigen::JacobiSVD<Eigen::MatrixXd> svd(A_best, Eigen::ComputeFullV);
        best_normal = svd.matrixV().col(3);

        // ---------- 7. 可选调试 ----------
        // LOG(ERROR) << "best_normal = " << best_normal.transpose();
        // LOG(ERROR) << "inliers = " << max_inlier_count << " / " << N;
        // LOG(ERROR) << "fit residual = \n" << (A_best * best_normal).transpose();

        std::chrono::time_point<std::chrono::high_resolution_clock> flow_ransac_end = std::chrono::high_resolution_clock::now();
        auto dt_ms = std::chrono::duration_cast<
            std::chrono::duration<double, std::milli>>(
            flow_ransac_end - flow_ransac_start).count();

        // LOG(ERROR) << "RansacPlaneFit = " << dt_ms << " ms";

    }




        // SAE光流计算示例
        // 输入:
        //   saeTimeSurface: SAE矩阵，类型CV_64FC1，存储像素的最新事件时间戳，单位秒
        //   validTimeThreshold: 时间阈值，超过此时间视为空（跳过）
        //   windowSize: 拟合窗口大小，奇数，比如3或5
        // 输出:
        //   flowX, flowY: 输出光流X和Y分量，大小同saeTimeSurface，CV_64FC1，空白区域填0
        bool ComputeSAEFlow()
        {
            std::chrono::time_point<std::chrono::high_resolution_clock> flow_start = std::chrono::high_resolution_clock::now();
            // LOG(ERROR) << "radius = " << radius << std::endl;

            // CV_Assert(sae_ptr_->GetPostiveImg().type() == CV_64FC1);
            
            // cv::Mat saeTimeSurface = sae_ptr_->GetPostiveImg(); // return a copy img
            // cv::Mat sae_decay = sae_ptr_->decaySAE(saeTimeSurface, sae_time); // 0.2 0.5
            cv::Mat sae_decay = sae_ptr_->decaySAE(sae_ptr_->GetPostiveImg(), sae_time);

            std::chrono::time_point<std::chrono::high_resolution_clock> flow_sae = std::chrono::high_resolution_clock::now();
            // cv::Mat binaryMask = (saeTimeSurface > 0);  // 只要时间戳 > 0 就认为有效

            // cv::Mat binaryMask = (sae_decay > 0);  // 只要时间戳 > 0 就认为有效
            cv::Mat binaryMask;
            cv::threshold(sae_decay, binaryMask, 0, 255, cv::THRESH_BINARY);

            // 将边界处的像素设置为 0
            // binaryMask.rowRange(0, radius).setTo(0);                              // 顶部
            // binaryMask.rowRange(binaryMask.rows - radius, binaryMask.rows).setTo(0); // 底部
            // binaryMask.colRange(0, radius).setTo(0);                              // 左侧
            // binaryMask.colRange(binaryMask.cols - radius, binaryMask.cols).setTo(0); // 右侧
            // 清除边界
            cv::rectangle(binaryMask, cv::Rect(0, 0, binaryMask.cols, radius), 0, cv::FILLED);
            cv::rectangle(binaryMask, cv::Rect(0, binaryMask.rows - radius, binaryMask.cols, radius), 0, cv::FILLED);
            cv::rectangle(binaryMask, cv::Rect(0, 0, radius, binaryMask.rows), 0, cv::FILLED);
            cv::rectangle(binaryMask, cv::Rect(binaryMask.cols - radius, 0, radius, binaryMask.rows), 0, cv::FILLED);


            binaryMask.convertTo(binaryMask, CV_8U);   // 转成8位图
            // cv::Mat kernel_ = cv::getStructuringElement(cv::MORPH_RECT,
            //                                             cv::Size(2 * radius + 1, 2 * radius + 1));
            //                                             // cv::Size(4 * radius + 1, 4 * radius + 1));    // 转为成员函数
            cv::Mat erodedMask;
            cv::erode(binaryMask, erodedMask, kernel_);
            std::vector<cv::Point> candidates;
            cv::findNonZero(erodedMask, candidates);
            std::chrono::time_point<std::chrono::high_resolution_clock> flow_disnoise = std::chrono::high_resolution_clock::now();
            // cv::Mat sae_decay
            // cv::Mat erodedMask;
            // sae_decay = erodedMask.clone();


            // LOG(ERROR) << "sae_decay type: " << sae_decay.type() << std::endl;

            // static long int img_index = 0;
            // cv::imwrite("/home/hao/Desktop/twist_ws/src/TwistEstimator/output/binary_mask_" 
            //                 + std::to_string(img_index) + ".png", binaryMask);
            // cv::imwrite("/home/hao/Desktop/twist_ws/src/TwistEstimator/output/eroded_mask_" 
            //                 + std::to_string(img_index++) + ".png", erodedMask);

            // 按网格均匀提取一定数量的点,然后在saeTimeSurface有效点局部邻阈中拟合平面参数
            // 设置网格间隔，例如 10 像素
            // int grid_size = std::max(candidates.size() * 0.2, 10.0);
            // int grid_size = 8;
            
            // LOG(ERROR) << "grid_size = " << grid_size << std::endl;
            std::vector<cv::Point> sampled_points;

            // 网格采样
            cv::Mat sample_mask = cv::Mat::zeros(erodedMask.size(), CV_8U);
            /*{
            // 打乱 candidates 顺序（就地乱序）
            std::random_device rd;
            std::mt19937 g(rd());
            std::shuffle(candidates.begin(), candidates.end(), g);
            for (const auto& pt : candidates) {
                int x = pt.x;
                int y = pt.y;
                if (sample_mask.at<uchar>(y / grid_size, x / grid_size) == 0) {
                    sampled_points.emplace_back(pt);
                    sample_mask.at<uchar>(y / grid_size, x / grid_size) = 1;
                }
            }
            // LOG(ERROR) << "total.candidate = " << candidates.size() << std::endl;
            // LOG(ERROR) << "sampled_points.size = " << sampled_points.size() << std::endl;
            if(sampled_points.size() < 6)
                return false;
            }*/
            // --- 1. 初始化 ---
            // 注意 grid_map 尺寸应缩小为 grid_size 的比例
            int grid_rows = erodedMask.rows / grid_size + 1;
            int grid_cols = erodedMask.cols / grid_size + 1;

            if (sample_mask.empty() || sample_mask.rows != grid_rows || sample_mask.cols != grid_cols)
                sample_mask = cv::Mat::zeros(grid_rows, grid_cols, CV_8U);
            else
                sample_mask.setTo(0);

            // --- 2. 打乱候选点 ---
            static thread_local std::mt19937 g(std::random_device{}());
            std::shuffle(candidates.begin(), candidates.end(), g);

            // --- 3. 快速遍历 ---
            sampled_points.clear();
            sampled_points.reserve(candidates.size());

            uchar* data = sample_mask.data;
            int cols = sample_mask.cols;

            for (const auto& pt : candidates) {
                int gx = pt.x / grid_size;
                int gy = pt.y / grid_size;
                uchar* p = data + gy * cols + gx;
                if (*p == 0) {
                    *p = 1;
                    sampled_points.emplace_back(pt);
                    if (sampled_points.size() >= 1000) // 可调
                        break;
                }
            }

            // LOG(INFO) << "Total candidates: " << candidates.size()
            //         << ", sampled: " << sampled_points.size();

            if (sampled_points.size() < 6)
                return false;

            std::chrono::time_point<std::chrono::high_resolution_clock> flow_sample = std::chrono::high_resolution_clock::now();

            flow_pre_points.clear();
            best_inliers.clear();
            plane_params.clear();
            normal_flows.clear();
            normal_norms.clear();
            /*{
                // 拟合平面
                const int mesh_size = (2 * radius + 1);
                for (const auto& pt : sampled_points) {
                    // 构造每个点的 a * x + b * y + c * t + d = 0;
                    Eigen::MatrixXd A(mesh_size * mesh_size, 4);
                    // double center_timestamp = saeTimeSurface.at<double>(pt.y, pt.x);
                    double center_timestamp = sae_decay.at<double>(pt.y, pt.x);
                    // LOG(ERROR) << "center_timestamp = " << center_timestamp << std::endl;
                    
                    int count = 0;
                    for (int dy = -radius; dy <= radius; ++dy) {
                        for (int dx = -radius; dx <= radius; ++dx) {
                            // LOG(ERROR) << "dx = " << dx << ", dy = " << dy << std::endl;
                            // LOG(ERROR) << "x = " << pt.x + dx << ", y = " << pt.y + dy << std::endl;
                            // LOG(ERROR) << "timestamp = " << sae_decay.at<double>(pt.y + dy, pt.x + dx) << std::endl;
                            assert(sae_decay.at<double>(pt.y + dy, pt.x + dx) != 0 && "timstamp fault");
                            // A.row(count++) << dx, dy, sae_decay.at<double>(pt.y + dy, pt.x + dx), 1.0;
                            A.row(count++) << dx, dy, sae_decay.at<double>(pt.y + dy, pt.x + dx) - center_timestamp, 1.0;
                            // A.row(count++) << dx, dy, saeTimeSurface.at<double>(pt.y + dy, pt.x + dx), 1.0;
                                                    // - center_timestamp, 1.0;
                            // LOG(ERROR) << "saeTimeSurface = " << saeTimeSurface.at<double>(pt.y + dy, pt.x + dx) << std::endl;
                            // LOG(ERROR) << "sae_decay = " << sae_decay.at<double>(pt.y + dy, pt.x + dx) << std::endl;
                        }
                    }
                    //  // LOG(ERROR) << "pt = " << pt.x << ", " << pt.y << std::endl;
                    // LOG(ERROR) << "A = " << A << std::endl;

                    {
                        Eigen::Vector4d normal;
                        normal.setZero();
                        RansacPlaneFit(A, 0.05, normal, 8);
                        const double a_ = normal(0);
                        const double b_ = normal(1);
                        const double c_ = normal(2);

                        if (std::abs(a_) > 1e-6 && std::abs(b_) > 1e-6) {
                            event_flow_velocity flow;
                            flow.x = -c_ / a_;
                            flow.y = -c_ / b_;
                            // LOG(ERROR) << "local flow = " << flow.x << ", " << flow.y;
                            // LOG(ERROR) << "fx = " << K(0,0) << ", fy = " << K(1,1);
                            // flow.x /= K(0,0);
                            // flow.y /= K(1,1);
                            double flow_norm = std::sqrt(flow.x * flow.x + flow.y * flow.y);
                            // LOG(ERROR) << "local flow = " << flow.x << ", " << flow.y;
                            // LOG(ERROR) << "flow norm = " << flow_norm;
                            if(flow_norm < 500)
                            {
                                flow_pre_points.push_back(flow);
                                best_inliers.push_back(pt);
                                plane_params.push_back(normal);
                
                                Eigen::Vector3d norm_grad_vec;
                                // double norm_grad = sqrt(plane(0) * plane(0) + plane(1) * plane(1)); //  a* a + b* b
                                norm_grad_vec <<  normal(0) * K(0,0), normal(1) * K(1,1), 0.0;
                                // norm_grad_vec <<  normal(0), normal(1), 0.0;
                                double norm_grad = norm_grad_vec.norm();
                                norm_grad_vec = norm_grad_vec / norm_grad;     // grad / grad.norm();
                                double normal_norm = - normal(2) / norm_grad;
                                Eigen::Vector3d normal_flow = normal_norm * norm_grad_vec;

                                // // 上面的像素光流需要转到相机系下光流
                                // double focal_len_inv = (K(0,0) + K(1,1)) / 2;
                                // // LOG(ERROR) << "focal_len_inv = " << focal_len_inv << std::endl;
                                // normal_norm *= focal_len_inv;
                                // // LOG(ERROR) << "normal_norm = " << normal_norm << std::endl;

                                // LOG(ERROR) << "norm_grad_vec = " << norm_grad_vec.transpose();
                                // LOG(ERROR) << "normal_norm = " << normal_norm;

                                normal_flows.push_back(norm_grad_vec);
                                normal_norms.push_back(normal_norm);
                            }
                        }
                    }
                }
            }*/

            // 并行加速外层循环
            /*
            {
                std::vector<event_flow_velocity> local_flows;
                std::vector<cv::Point> local_inliers;
                std::vector<Eigen::Vector4d> local_planes;
                std::vector<Eigen::Vector3d> local_flow_dirs;
                std::vector<double> local_norms;

                const int mesh_size = (2 * radius + 1);
                const int N_patch = mesh_size * mesh_size;
                #pragma omp for schedule(dynamic)
                
                for (int i = 0; i < sampled_points.size(); ++i) {
                    const auto& pt = sampled_points[i];

                    // Eigen::MatrixXd A(mesh_size * mesh_size, 4);
                    Eigen::MatrixXd A(N_patch, 4);
                    double center_timestamp = sae_decay.at<double>(pt.y, pt.x);

                    int count = 0;
                    // for (int dy = -radius; dy <= radius; ++dy) {
                    //     for (int dx = -radius; dx <= radius; ++dx) {
                    //         double val = sae_decay.at<double>(pt.y + dy, pt.x + dx);
                    //         A.row(count++) << dx, dy, val - center_timestamp, 1.0;
                    //     }
                    // }
                    const double* row_ptr = sae_decay.ptr<double>(pt.y - radius);
                    for (int dy = 0; dy < mesh_size; ++dy) {
                        const double* p = sae_decay.ptr<double>(pt.y - radius + dy);
                        for (int dx = -radius; dx <= radius; ++dx) {
                            double val = p[pt.x + dx];
                            A.row(count++) << dx, dy - radius, val - center_timestamp, 1.0;
                        }
                    }

                    Eigen::Vector4d normal;
                    normal.setZero();
                    RansacPlaneFit(A, 0.05, normal, 8);

                    double a_ = normal(0), b_ = normal(1), c_ = normal(2);
                    if (std::abs(a_) > 1e-6 && std::abs(b_) > 1e-6) {
                        event_flow_velocity flow;
                        flow.x = -c_ / a_;
                        flow.y = -c_ / b_;

                        // double flow_norm = std::sqrt(flow.x * flow.x + flow.y * flow.y);
                        // if (flow_norm < 500) 
                        double flow_norm2 = flow.x * flow.x + flow.y * flow.y;
                        if (flow_norm2 < 250000.0)
                        {
                            Eigen::Vector3d norm_grad_vec(normal(0) * K(0,0), normal(1) * K(1,1), 0.0);
                            double norm_grad = norm_grad_vec.norm();
                            norm_grad_vec /= norm_grad;
                            double normal_norm = -normal(2) / norm_grad;

                            // ✅ 并行中不能直接 push_back，全局容器需要加锁
                            /*#pragma omp critical
                            {
                                flow_pre_points.push_back(flow);
                                best_inliers.push_back(pt);
                                plane_params.push_back(normal);
                                normal_flows.push_back(norm_grad_vec);
                                normal_norms.push_back(normal_norm);
                            } // * /
                            local_flows.push_back(flow);
                            local_inliers.push_back(pt);
                            local_planes.push_back(normal);
                            local_flow_dirs.push_back(norm_grad_vec);
                            local_norms.push_back(normal_norm);
                        }
                    }
                }

                #pragma omp critical
                {
                    flow_pre_points.insert(flow_pre_points.end(), local_flows.begin(), local_flows.end());
                    best_inliers.insert(best_inliers.end(), local_inliers.begin(), local_inliers.end());
                    plane_params.insert(plane_params.end(), local_planes.begin(), local_planes.end());
                    normal_flows.insert(normal_flows.end(), local_flow_dirs.begin(), local_flow_dirs.end());
                    normal_norms.insert(normal_norms.end(), local_norms.begin(), local_norms.end());
                }
            }
            */

            // const int mesh_size = 2 * radius + 1;
            // const int Npath = mesh_size * mesh_size;
            // Eigen::VectorXd coords = Eigen::VectorXd::LinSpaced(mesh_size, -radius, radius);
            // flow_pre_points.clear();
            // best_inliers.clear();
            // plane_params.clear();
            // normal_flows.clear();
            // normal_norms.clear();
            // // 并行处理每个采样点
            // #pragma omp parallel
            // {
            //     std::vector<event_flow_velocity> local_flows;
            //     std::vector<cv::Point> local_inliers;
            //     std::vector<Eigen::Vector4d> local_planes;
            //     std::vector<Eigen::Vector3d> local_flow_dirs;
            //     std::vector<double> local_norms;

            //     #pragma omp for schedule(dynamic)
            //     for (int i = 0; i < (int)sampled_points.size(); ++i) {
            //         const cv::Point& pt = sampled_points[i];

            //         // 防止越界
            //         if (pt.x - radius < 0 || pt.y - radius < 0 ||
            //             pt.x + radius >= sae_decay.cols || pt.y + radius >= sae_decay.rows)
            //             continue;
                    
            //         // 获取 SAE 子区域 ROI
            //         cv::Rect roi(pt.x - radius, pt.y - radius, mesh_size, mesh_size);
            //         cv::Mat patch = sae_decay(roi);
                    
            //         // 映射到 Eigen 连续内存
            //         Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> patch_eigen(
            //             reinterpret_cast<double*>(patch.data), mesh_size, mesh_size);
            //         // LOG(ERROR) << "B";
            //         double center_timestamp = patch_eigen(radius, radius);
                    
            //         // 构建 A 矩阵
            //         Eigen::MatrixXd A(Npath, 4);
            //         int idx = 0;
            //         for (int dy = 0; dy < mesh_size; ++dy) {
            //             for (int dx = 0; dx < mesh_size; ++dx) {
            //                 A(idx, 0) = coords(dx);
            //                 A(idx, 1) = coords(dy);
            //                 A(idx, 2) = patch_eigen(dy, dx) - center_timestamp;
            //                 A(idx, 3) = 1.0;
            //                 ++idx;
            //             }
            //         }
            //         Eigen::Vector4d normal;
            //         normal.setZero();
            //         RansacPlaneFit(A, 0.05, normal, 8);
            //         double a_ = normal(0), b_ = normal(1), c_ = normal(2);
            //         if (std::abs(a_) > 1e-6 && std::abs(b_) > 1e-6) {
            //             event_flow_velocity flow;
            //             flow.x = -c_ / a_;
            //             flow.y = -c_ / b_;
            //             if (flow.x * flow.x + flow.y * flow.y < 250000.0) { // 最大速度约束
            //                 Eigen::Vector3d norm_grad_vec(normal(0) * K(0,0),
            //                                             normal(1) * K(1,1),
            //                                             0.0);
            //                 double norm_grad = norm_grad_vec.norm();
            //                 norm_grad_vec /= norm_grad;
            //                 double normal_norm = -normal(2) / norm_grad;

            //                 // 局部容器收集，避免临界区
            //                 local_flows.push_back(flow);
            //                 local_inliers.push_back(pt);
            //                 local_planes.push_back(normal);
            //                 local_flow_dirs.push_back(norm_grad_vec);
            //                 local_norms.push_back(normal_norm);
            //             }
            //         }
                    
            //     }
                
            //     // 合并局部结果到全局容器
            //     #pragma omp critical
            //     {
            //         flow_pre_points.insert(flow_pre_points.end(), local_flows.begin(), local_flows.end());
            //         best_inliers.insert(best_inliers.end(), local_inliers.begin(), local_inliers.end());
            //         plane_params.insert(plane_params.end(), local_planes.begin(), local_planes.end());
            //         normal_flows.insert(normal_flows.end(), local_flow_dirs.begin(), local_flow_dirs.end());
            //         normal_norms.insert(normal_norms.end(), local_norms.begin(), local_norms.end());
            //     }
            // }
            // 2025-12-15 修改

            // 假设 radius 在编译期或运行期固定较小（<= 6）
            const int mesh_size = 2 * radius + 1;
            const int Npath = mesh_size * mesh_size;

            // 清空全局容器（外部）
            flow_pre_points.clear();
            best_inliers.clear();
            plane_params.clear();
            normal_flows.clear();
            normal_norms.clear();

            // --------- OpenMP 并行区 ---------
            // const int max_threads = omp_get_max_threads();

            // // 每个线程一个局部容器，避免 critical
            // std::vector<std::vector<event_flow_velocity>> flows_tls(max_threads);
            // std::vector<std::vector<cv::Point>>          inliers_tls(max_threads);
            // std::vector<std::vector<Eigen::Vector4d>>    planes_tls(max_threads);
            // std::vector<std::vector<Eigen::Vector3d>>    flow_dirs_tls(max_threads);
            // std::vector<std::vector<double>>             norms_tls(max_threads);

            // #pragma omp parallel
            // {
            //     const int tid = omp_get_thread_num();

            //     auto& local_flows     = flows_tls[tid];
            //     auto& local_inliers   = inliers_tls[tid];
            //     auto& local_planes    = planes_tls[tid];
            //     auto& local_flow_dirs = flow_dirs_tls[tid];
            //     auto& local_norms     = norms_tls[tid];

            //     // 栈上 A，避免 heap
            //     Eigen::Matrix<double, Eigen::Dynamic, 4, Eigen::RowMajor> A;
            //     A.resize(Npath, 4);

            //     #pragma omp for schedule(dynamic)
            //     for (int i = 0; i < (int)sampled_points.size(); ++i)
            //     {
            //         const cv::Point& pt = sampled_points[i];

            //         // ROI 边界检查
            //         if (pt.x - radius < 0 || pt.y - radius < 0 ||
            //             pt.x + radius >= sae_decay.cols ||
            //             pt.y + radius >= sae_decay.rows)
            //             continue;

            //         // ROI
            //         cv::Rect roi(pt.x - radius, pt.y - radius, mesh_size, mesh_size);
            //         const cv::Mat patch = sae_decay(roi);

            //         // 直接指针访问（CV_64F）
            //         const double* patch_ptr = patch.ptr<double>();
            //         const int stride = patch.step1();

            //         const double center_timestamp = patch_ptr[radius * stride + radius];

            //         // 构建 A（无 Eigen::VectorXd coords）
            //         int idx = 0;
            //         for (int dy = 0; dy < mesh_size; ++dy)
            //         {
            //             const int oy = dy - radius;
            //             for (int dx = 0; dx < mesh_size; ++dx)
            //             {
            //                 const int ox = dx - radius;
            //                 const double dt =
            //                     patch_ptr[dy * stride + dx] - center_timestamp;

            //                 A(idx, 0) = ox;
            //                 A(idx, 1) = oy;
            //                 A(idx, 2) = dt;
            //                 A(idx, 3) = 1.0;
            //                 ++idx;
            //             }
            //         }

            //         // RANSAC 平面拟合
            //         Eigen::Vector4d normal;
            //         normal.setZero();
            //         RansacPlaneFit(A, 0.05, normal, 8);

            //         const double a = normal(0);
            //         const double b = normal(1);
            //         const double c = normal(2);

            //         if (std::abs(a) < 1e-6 || std::abs(b) < 1e-6)
            //             continue;

            //         event_flow_velocity flow;
            //         flow.x = -c / a;
            //         flow.y = -c / b;

            //         // 速度约束
            //         if (flow.x * flow.x + flow.y * flow.y > 250000.0)
            //             continue;

            //         // 法向梯度
            //         Eigen::Vector3d grad(
            //             normal(0) * K(0, 0),
            //             normal(1) * K(1, 1),
            //             0.0);

            //         const double grad_norm = grad.norm();
            //         if (grad_norm < 1e-8)
            //             continue;

            //         grad /= grad_norm;
            //         const double normal_norm = -normal(2) / grad_norm;

            //         // 线程本地收集
            //         local_flows.push_back(flow);
            //         local_inliers.push_back(pt);
            //         local_planes.push_back(normal);
            //         local_flow_dirs.push_back(grad);
            //         local_norms.push_back(normal_norm);
            //     }
            // }
            // 2025-12-15 修改
            // --------- OpenMP 并行区 ---------
            const int max_threads = omp_get_max_threads();

            // TLS containers
            std::vector<std::vector<event_flow_velocity>> flows_tls(max_threads);
            std::vector<std::vector<cv::Point>>          inliers_tls(max_threads);
            std::vector<std::vector<Eigen::Vector4d>>    planes_tls(max_threads);
            std::vector<std::vector<Eigen::Vector3d>>    flow_dirs_tls(max_threads);
            std::vector<std::vector<double>>             norms_tls(max_threads);

            #pragma omp parallel
            {
                const int tid = omp_get_thread_num();

                auto& local_flows     = flows_tls[tid];
                auto& local_inliers   = inliers_tls[tid];
                auto& local_planes    = planes_tls[tid];
                auto& local_flow_dirs = flow_dirs_tls[tid];
                auto& local_norms     = norms_tls[tid];

                // 预分配，避免 realloc
                const int approx = sampled_points.size() / max_threads + 8;
                local_flows.reserve(approx);
                local_inliers.reserve(approx);
                local_planes.reserve(approx);
                local_flow_dirs.reserve(approx);
                local_norms.reserve(approx);

                // 栈上 A（一次 resize）
                Eigen::Matrix<double, Eigen::Dynamic, 4, Eigen::RowMajor> A(Npath, 4);

                #pragma omp for schedule(guided,4)
                for (int i = 0; i < (int)sampled_points.size(); ++i)
                {
                    const cv::Point& pt = sampled_points[i];

                    if (pt.x - radius < 0 || pt.y - radius < 0 ||
                        pt.x + radius >= sae_decay.cols ||
                        pt.y + radius >= sae_decay.rows)
                        continue;

                    // ROI
                    const cv::Rect roi(pt.x - radius, pt.y - radius, mesh_size, mesh_size);
                    const cv::Mat patch = sae_decay(roi);

                    const double* patch_ptr = patch.ptr<double>();
                    const int stride = patch.step1();

                    const double center_ts = patch_ptr[radius * stride + radius];

                    // -------- 构建 A（优化版）--------
                    int idx = 0;
                    for (int dy = 0; dy < mesh_size; ++dy)
                    {
                        const int oy = dy - radius;
                        const double* row = patch_ptr + dy * stride;

                        for (int dx = 0; dx < mesh_size; ++dx, ++idx)
                        {
                            A(idx, 0) = dx - radius;
                            A(idx, 1) = oy;
                            A(idx, 2) = row[dx] - center_ts;
                            A(idx, 3) = 1.0;
                        }
                    }

                    // -------- RANSAC（保留）--------
                    Eigen::Vector4d normal;
                    RansacPlaneFit(A, 0.05, normal, 8);

                    const double a = normal(0);
                    const double b = normal(1);
                    const double c = normal(2);

                    if (a * a < 1e-12 || b * b < 1e-12)
                        continue;

                    const double vx = -c / a;
                    const double vy = -c / b;
                    if (vx * vx + vy * vy > 250000.0)
                        continue;

                    Eigen::Vector3d grad(
                        a * K(0, 0),
                        b * K(1, 1),
                        0.0);

                    const double grad_norm = grad.norm();
                    if (grad_norm < 1e-8)
                        continue;

                    grad /= grad_norm;
                    const double normal_norm = -c / grad_norm;

                    // -------- TLS 收集 --------
                    local_flows.push_back({vx, vy});
                    local_inliers.push_back(pt);
                    local_planes.push_back(normal);
                    local_flow_dirs.push_back(grad);
                    local_norms.push_back(normal_norm);
                }
            }
            std::chrono::time_point<std::chrono::high_resolution_clock> flow_plane_2 = std::chrono::high_resolution_clock::now();
            // --------- 串行合并（无锁） ---------
            for (int t = 0; t < max_threads; ++t)
            {
                flow_pre_points.insert(flow_pre_points.end(),
                    flows_tls[t].begin(), flows_tls[t].end());

                best_inliers.insert(best_inliers.end(),
                    inliers_tls[t].begin(), inliers_tls[t].end());

                plane_params.insert(plane_params.end(),
                    planes_tls[t].begin(), planes_tls[t].end());

                normal_flows.insert(normal_flows.end(),
                    flow_dirs_tls[t].begin(), flow_dirs_tls[t].end());

                normal_norms.insert(normal_norms.end(),
                    norms_tls[t].begin(), norms_tls[t].end());
            }
            std::chrono::time_point<std::chrono::high_resolution_clock> flow_plane = std::chrono::high_resolution_clock::now();


            // LOG(ERROR) << "best_inliers.size() = " << best_inliers.size() << std::endl;

            if(best_inliers.size() < 3)
            {
                // LOG(ERROR) << "not enough flow_collect, only: " << best_inliers.size() << std::endl;
                return false;
            }


            // 1. 归一化 saeTimeSurface 到 [0, 255]
            // cv::Mat sae_normalized;
            // cv::normalize(saeTimeSurface, sae_normalized, 0, 255, cv::NORM_MINMAX, CV_8UC1);
            // cv::normalize(sae_decay, sae_normalized, 0, 255, cv::NORM_MINMAX, CV_8UC1);
            // sae_decay.setTo(0, (erodedMask == 0));
            cv::Mat nonZeroMask = (sae_decay != 0);
            double minVal, maxVal;
            cv::minMaxLoc(sae_decay, &minVal, &maxVal, 0, 0, nonZeroMask);
            // LOG(ERROR) << "minVal = " << minVal << ", maxVal = " << maxVal;
            cv::Mat sae_normalized = cv::Mat::zeros(sae_decay.size(), CV_8UC1);
            if (minVal != maxVal) {
                double alpha = (255.0 - 128.0) / (maxVal - minVal);
                double beta  = 128.0 - alpha * minVal;
                cv::Mat tmp;
                sae_decay.convertTo(tmp, CV_8U, alpha, beta);
                tmp.copyTo(sae_normalized, nonZeroMask);
            } else {
                // 所有非零像素相等，映射为中间值
                sae_normalized.setTo(191, nonZeroMask);
            }

            // sae_normalized.copyTo(sae_normalized);

            // sae_normalized = sae_ptr_->decaySAE(saeTimeSurface);

            // 2. 复制为彩色图以便绘制箭头
            cv::Mat sae_color;
            cv::cvtColor(sae_normalized, sae_color, cv::COLOR_GRAY2BGR);

            /* // 3. 遍历 best_inliers 和 flow_pre_points，绘制光流箭头
            for (size_t i = 0; i < best_inliers.size(); ++i) {
                // const cv::Point2d& pt = best_inliers[i];
                /*
                const event_flow_velocity& flow = flow_pre_points[i];

                Eigen::Vector3d normal_flow;
                const auto plane = plane_params[i];
                normal_flow <<  plane(0), plane(1), 0.0;
                double norm_flow_2 = normal_flow.norm() * normal_flow.norm(); 
                normal_flow = (- plane(2) * normal_flow / norm_flow_2);
                // LOG(ERROR) << "normal_flow.norm() = " << normal_flow.norm() << std::endl;
                
                if(normal_flow.norm() > 60)
                    continue;
                    // * /
                

                // Eigen::Vector3d normal_flow;
                // normal_flow << flow.x, flow.y, 0.0;

                // Eigen::Vector3d normal_flow = normal_flows[i];
                // LOG(ERROR) << "visualize flow = " << normal_flow(0) * K(0,0) *  sae_time;
                // LOG(ERROR) << "visualize flow = " << normal_flow(1) * K(1,1) *  sae_time;
                // cv::Point2d normal_end_pt(pt.x + 1.0 * std::max(normal_flow(0) * K(0,0) *  0.5 * sae_time, 1.0),
                //                          pt.y  + 1.0 * std::max(normal_flow(1) * K(1,1) *  0.5 * sae_time, 1.0)); // 放大显示箭头
                
                // cv::Point2d normal_end_pt(pt.x + 1.0 * std::max(normal_flow(0) * 20, 1.0),
                //                          pt.y  + 1.0 * std::max(normal_flow(1) * 20, 1.0)); // 放大显示箭头
                cv::Point2d normal_end_pt(best_inliers[i].x + 1.0 * std::max(normal_flows[i](0) * 20, 1.0),
                        best_inliers[i].y  + 1.0 * std::max(normal_flows[i](1) * 20, 1.0)); // 放大显示箭头
                // cv::arrowedLine(sae_color, pt, normal_end_pt, cv::Scalar(255, 0, 0), 1, cv::LINE_AA, 0, 0.2);
                cv::arrowedLine(sae_color, best_inliers[i], normal_end_pt, cv::Scalar(255, 0, 0), 1, cv::LINE_AA, 0, 0.2);
                // cv::line(sae_color, pt, normal_end_pt, cv::Scalar(255, 0, 0), 1, cv::LINE_AA);

                // cv::Point2d end_pt(pt.x + flow.x, pt.y + flow.y); // 放大显示箭头
                // // cv::arrowedLine(sae_color, pt, end_pt, cv::Scalar(0, 255, 0), 1, cv::LINE_AA, 0, 0.3);
                // cv::line(sae_color, pt, end_pt, cv::Scalar(0, 255, 0), 1, cv::LINE_AA);
            }*/
            // 3. 遍历 best_inliers 和 flow_pre_points，绘制光流箭头
            for (size_t i = 0; i < best_inliers.size(); ++i) {
                cv::Point2d normal_end_pt(best_inliers[i].x + 1.0 * std::max(normal_flows[i](0) * 20, 1.0),
                        best_inliers[i].y  + 1.0 * std::max(normal_flows[i](1) * 20, 1.0)); // 放大显示箭头
                cv::arrowedLine(sae_color, best_inliers[i], normal_end_pt, cv::Scalar(255, 255, 0), 1, cv::LINE_AA, 0, 0.25);
            }

            // 4. 发布图像
            std_msgs::Header header;
            header.stamp = process_time;
            header.frame_id = "event";

            sensor_msgs::ImagePtr msg = cv_bridge::CvImage(header, "bgr8", sae_color).toImageMsg();
            pub_event_image_.publish(msg);

            publishEventFlowCloud(header, sae_color, best_inliers, normal_flows);

            std::chrono::time_point<std::chrono::high_resolution_clock> flow_visualize = std::chrono::high_resolution_clock::now();
            // static long int img_index = 0;
            // cv::imwrite("/home/hao/Desktop/twist_ws/src/TwistEstimator/output/flow_" 
            //         + std::to_string(img_index++) + ".png", sae_color);

            /*{
                std::chrono::duration<double, std::milli> elapsed;
                elapsed = flow_visualize - flow_start;
                // LOG(ERROR) << "FLow Total Time: " << std::setprecision(18) << elapsed.count() << std::endl;
                elapsed = flow_sae - flow_start;
                // LOG(ERROR) << "FLow Sae: " << std::setprecision(18) << elapsed.count() << std::endl;
                elapsed = flow_disnoise - flow_sae;
                // LOG(ERROR) << "FLow Disnoise: " << std::setprecision(18) << elapsed.count() << std::endl;
                elapsed = flow_sample - flow_disnoise;
                // LOG(ERROR) << "FLow Sample: " << std::setprecision(18) << elapsed.count() << std::endl;
                elapsed = flow_plane_2 - flow_sample;
                // LOG(ERROR) << "FLow Plane I: " << std::setprecision(18) << elapsed.count() << std::endl;
                elapsed = flow_plane - flow_plane_2;
                // LOG(ERROR) << "FLow Plane II: " << std::setprecision(18) << elapsed.count() << std::endl;
                elapsed = flow_plane - flow_sample;
                // LOG(ERROR) << "FLow Plane Total: " << std::setprecision(18) << elapsed.count() << std::endl;
                elapsed = flow_visualize - flow_plane;
                // LOG(ERROR) << "FLow Visualize: " << std::setprecision(18) << elapsed.count() << std::endl;
            }*/

            return true;
        }

        /*void publishEventFlowCloud(
            std_msgs::Header header,
            const cv::Mat& sae_color,
            const std::vector<cv::Point2d>& best_inliers,
            const std::vector<Eigen::Vector3d>& normal_flows)
        {
            if (sae_color.empty()) {
                    ROS_WARN("sae_color empty, skip publish");
                    return;
                }

                // --- 相机中心与缩放因子 ---
                // const float cx = 320.0f;
                // const float cy = 256.0f;
                const float cx = 160.0f;
                const float cy = 128.0f;
                const float scale = 1.0f; // 每个像素转为 0.002 m，可根据实际调整

                // --- 生成掩码 mask，找到非零像素 ---
                cv::Mat mask;
                if (sae_color.channels() == 3) {
                    cv::inRange(sae_color, cv::Scalar(1,1,1), cv::Scalar(255,255,255), mask);
                } else {
                    cv::threshold(sae_color, mask, 0, 255, cv::THRESH_BINARY);
                }

                std::vector<cv::Point> nonzero_pts;
                cv::findNonZero(mask, nonzero_pts);

                size_t total_points = nonzero_pts.size() + 2 * best_inliers.size();

                sensor_msgs::PointCloud2 cloud_msg;
                cloud_msg.header = header;
                cloud_msg.header.frame_id = "radar";
                cloud_msg.height = 1;
                cloud_msg.is_dense = false;

                sensor_msgs::PointCloud2Modifier modifier(cloud_msg);
                modifier.setPointCloud2FieldsByString(2, "xyz", "rgb");
                modifier.resize(total_points);

                sensor_msgs::PointCloud2Iterator<float> iter_x(cloud_msg, "x");
                sensor_msgs::PointCloud2Iterator<float> iter_y(cloud_msg, "y");
                sensor_msgs::PointCloud2Iterator<float> iter_z(cloud_msg, "z");
                sensor_msgs::PointCloud2Iterator<uint8_t> iter_r(cloud_msg, "r");
                sensor_msgs::PointCloud2Iterator<uint8_t> iter_g(cloud_msg, "g");
                sensor_msgs::PointCloud2Iterator<uint8_t> iter_b(cloud_msg, "b");

                // --- 1️⃣ 普通非零像素点 ---
                for (const auto& p : nonzero_pts) {
                    int u = p.x;
                    int v = p.y;

                    // 坐标去中心化并投影到 Y-Z 平面
                    float X = 50.0f;
                    float Y = -(u - cx) * scale;
                    float Z = -(v - cy) * scale;

                    *iter_x = X;
                    *iter_y = Y;
                    *iter_z = Z;

                    // LOG(ERROR) << "pixel = [" << u << ", " << v << "]";
                    // LOG(ERROR) << "coordinate = [" << X << ", " << Y << "]";

                    if (sae_color.channels() == 3) {
                        cv::Vec3b col = sae_color.at<cv::Vec3b>(v, u);
                        *iter_r = col[2];
                        *iter_g = col[1];
                        *iter_b = col[0];
                    } else {
                        uint8_t gray = sae_color.at<uint8_t>(v, u);
                        *iter_r = gray;
                        *iter_g = gray;
                        *iter_b = gray;
                    }

                    ++iter_x; ++iter_y; ++iter_z;
                    ++iter_r; ++iter_g; ++iter_b;
                }
                // LOG(ERROR) << "Project Pixel Sucess!";

                float scale_flow = 50.0f; // 缩放光流向量长度
                // LOG(ERROR) << "best_inliers.size = " << best_inliers.size();
                // LOG(ERROR) << "normal_flows.size = " << normal_flows.size();
                for (size_t i = 0; i < best_inliers.size(); ++i) {
                    const cv::Point2d& pt = best_inliers[i];
                    int u = static_cast<int>(pt.x);
                    int v = static_cast<int>(pt.y);
                    if (u < 0 || u >= sae_color.cols || v < 0 || v >= sae_color.rows) continue;

                    float dx = normal_flows[i](0);
                    float dy = normal_flows[i](1) ;

                    // 起点（红色）
                    float X0 = 50.0f;
                    float Y0 = (pt.x - cx) * scale;
                    float Z0 = -(pt.y - cy) * scale;

                    *iter_x = X0;
                    *iter_y = Y0;
                    *iter_z = Z0;
                    *iter_r = 255;
                    *iter_g = 0;
                    *iter_b = 0;
                    ++iter_x; ++iter_y; ++iter_z;
                    ++iter_r; ++iter_g; ++iter_b;

                    // 终点（蓝色）
                    float X1 = 0.0f;
                    float Y1 = Y0 + dx * scale_flow;
                    float Z1 = Z0 - dy * scale_flow; // 注意 Z 轴方向与像素坐标一致

                    *iter_x = X1;
                    *iter_y = Y1;
                    *iter_z = Z1;
                    *iter_r = 0;
                    *iter_g = 0;
                    *iter_b = 255;
                    ++iter_x; ++iter_y; ++iter_z;
                    ++iter_r; ++iter_g; ++iter_b;
                }

                pub_cloud.publish(cloud_msg);
        }*/

        void publishEventFlowCloud(
            const std_msgs::Header& header,
            const cv::Mat& sae_color,
            const std::vector<cv::Point2d>& best_inliers,
            const std::vector<Eigen::Vector3d>& normal_flows)
        {
            if (sae_color.empty()) {
                ROS_WARN("sae_color empty, skip publish");
                return;
            }

            // --- 参数 ---
            float R = 100.0f;                // 球面半径
            float scale_flow = 20.0f;        // 光流箭头长度缩放
            float max_theta = M_PI / 4.0f;  // 水平方向角度范围 (rad)
            float max_phi   = M_PI / 4.0f;  // 垂直方向角度范围 (rad)

            float cx = sae_color.cols * 0.5f;
            float cy = sae_color.rows * 0.5f;

            // --- 生成掩码 mask，找到非零像素 ---
            cv::Mat mask;
            if (sae_color.channels() == 3) {
                cv::inRange(sae_color, cv::Scalar(1,1,1), cv::Scalar(255,255,255), mask);
            } else {
                cv::threshold(sae_color, mask, 0, 255, cv::THRESH_BINARY);
            }

            std::vector<cv::Point> nonzero_pts;
            cv::findNonZero(mask, nonzero_pts);
            int n_line_points = 10; // 线段上点的数量
            size_t total_points = nonzero_pts.size() + best_inliers.size() * (2 + n_line_points); // 光流起点 + 终点

            // --- 初始化 PointCloud2 ---
            sensor_msgs::PointCloud2 cloud_msg;
            cloud_msg.header = header;
            cloud_msg.header.frame_id = "radar";
            cloud_msg.height = 1;
            cloud_msg.is_dense = false;

            sensor_msgs::PointCloud2Modifier modifier(cloud_msg);
            modifier.setPointCloud2FieldsByString(2, "xyz", "rgb");
            modifier.resize(total_points);

            sensor_msgs::PointCloud2Iterator<float> iter_x(cloud_msg, "x");
            sensor_msgs::PointCloud2Iterator<float> iter_y(cloud_msg, "y");
            sensor_msgs::PointCloud2Iterator<float> iter_z(cloud_msg, "z");
            sensor_msgs::PointCloud2Iterator<uint8_t> iter_r(cloud_msg, "r");
            sensor_msgs::PointCloud2Iterator<uint8_t> iter_g(cloud_msg, "g");
            sensor_msgs::PointCloud2Iterator<uint8_t> iter_b(cloud_msg, "b");

            // --- 1️⃣ 非零像素点映射到球面 ---
            for (const auto& p : nonzero_pts) {
                int u = p.x;
                int v = p.y;

                float theta = -1.0 *(u - cx) / cx * max_theta; // 水平角
                float phi   = -1.0 *(v - cy) / cy * max_phi;   // 垂直角

                // 球面坐标
                float X = R * cos(phi) * cos(theta);
                float Y = R * cos(phi) * sin(theta);
                float Z = R * sin(phi);

                *iter_x = X;
                *iter_y = Y;
                *iter_z = Z;

                if (sae_color.channels() == 3) {
                    cv::Vec3b col = sae_color.at<cv::Vec3b>(v, u);
                    *iter_r = col[2];
                    *iter_g = col[1];
                    *iter_b = col[0];
                } else {
                    uint8_t gray = sae_color.at<uint8_t>(v, u);
                    *iter_r = gray;
                    *iter_g = gray;
                    *iter_b = gray;
                }

                ++iter_x; ++iter_y; ++iter_z;
                ++iter_r; ++iter_g; ++iter_b;
            }

            // --- 2️⃣ 光流 inliers 映射到球面并画箭头 ---
            /*
            size_t N = std::min(best_inliers.size(), normal_flows.size());
            for (size_t i = 0; i < N; ++i) {
                const cv::Point2d& pt = best_inliers[i];
                int u = static_cast<int>(pt.x);
                int v = static_cast<int>(pt.y);
                if (u < 0 || u >= sae_color.cols || v < 0 || v >= sae_color.rows) continue;

                float theta = -1.0 * (u - cx) / cx * max_theta;
                float phi   = -1.0 *(v - cy) / cy * max_phi;

                float dx = normal_flows[i](0) ;
                float dy = normal_flows[i](1) ;

                // 起点（红色）
                Eigen::Vector3f p0;
                p0[0] = R * cos(phi) * cos(theta);
                p0[1] = R * cos(phi) * sin(theta);
                p0[2] = R * sin(phi);

                // 终点（蓝色）
                Eigen::Vector3f p1 = p0;
                p1[1] += dx * scale_flow;  // Y方向偏移
                p1[2] -= dy * scale_flow;  // Z方向偏移

                // 线段插值
                
                for (int j = 0; j <= n_line_points; ++j) {
                    float alpha = float(j) / n_line_points;
                    Eigen::Vector3f pi = p0 * (1.0f - alpha) + p1 * alpha;

                    *iter_x = pi[0];
                    *iter_y = pi[1];
                    *iter_z = pi[2];

                    // 渐变颜色：起点红色 -> 终点蓝色
                    *iter_r = static_cast<uint8_t>(255 * (1.0f - alpha));
                    *iter_g = 0;
                    *iter_b = static_cast<uint8_t>(255 * alpha);

                    ++iter_x; ++iter_y; ++iter_z;
                    ++iter_r; ++iter_g; ++iter_b;
                }
            }
            */

            pub_cloud.publish(cloud_msg);
        }

        bool NormalAngularVelocityEsti(
            geometry_msgs::TwistWithCovarianceStamped& radar_vel  // 输出，角速度与协方差
        ) 
        { 
            Eigen::Matrix3d K_inv = K.inverse();
            Eigen::Matrix3d R_re = T_re.block(0, 0, 3, 3);

            const int ransac_iter = 15;
            const int minimum_flow = 3;
            int N = flow_pre_points.size();
            // LOG(ERROR) << "N = " << N  << std::endl;
            if (N < minimum_flow) {
                std::cerr << "Error: Not enough points for estimation." << std::endl;
                return false;
            }

            linear_vel << radar_vel.twist.twist.linear.x, radar_vel.twist.twist.linear.y, radar_vel.twist.twist.linear.z;
            if(linear_vel.norm() < 1e-6)
            {
                // LOG(ERROR) << "radar ego velocity is valid!" << std::endl;
                return false;
            }

            // 先构造全集 A 和 b
            Eigen::MatrixXd A_all(3 * N, 3);
            Eigen::VectorXd b_all(3 * N);

            for (int i = 0; i < N; ++i) {
                Eigen::Vector3d pixel_coord = K_inv * Eigen::Vector3d(best_inliers[i].x, best_inliers[i].y, 1.0);
                // Eigen::Vector3d pixel_coord = Eigen::Vector3d(best_inliers[i].x, best_inliers[i].y, 1.0);
                Eigen::Matrix3d pixel_skew = skew(pixel_coord);
                Eigen::Vector3d prefix = pixel_skew * R_re * linear_vel;

                // TODO: 这里 flow_pre_points[i] 需要替换为对应的光流向量，假设用x,y临时代替
                // Eigen::Vector3d flow(flow_pre_points[i].x, flow_pre_points[i].y, 0);

                // double& normal_norm = norm_pre_points[i];

                // norm_pre_points.push_back(grad_norm);
                // flow_pre_points.push_back(flow);          

                // Eigen::Vector3d grad;
                // grad << -1.0 / flow(0), -1.0 / flow(1), 0.0;
                // double normal_norm = 1.0 / grad.norm();
                // Eigen::Vector3d normal_flow = grad * normal_norm;

                // - (c * [a, b, 0]) / (a^2 + b^2)
                /*
                    [-c * a , -c * b, 0] / (a^2 + b^2)

                    a * x + b * y + c * z + d = 0
                    u = - c / a
                    v = - c / b

                    -c  / (1 + (b/a)^2 ) 
                    -c / (1 + (a/b)^2)

                    a * u = -c 
                    -c * a = a * u * a

                    -c * b = b * v * b

                    a^2 * u , b^2 * v  / (a^2 + b^2)
                    u / (1 + (b/a)^2), v / (1 + (a/b)^2)

                    u / (1 + (u / v)^2), v / (1 + (v / u)^2)

                    u * v^2 / (u^2 + v^2), v * u^2 / (u^2 + v^2)

                    [u, v] * [v^2 , u^2] / (u^2 + v^2)
                    
                    b / a = u / v
                    a / b = v / u
                */
                /*{
                    const auto plane = plane_params[i];
                    Eigen::Vector3d norm_grad_vec;
                    // double norm_grad = sqrt(plane(0) * plane(0) + plane(1) * plane(1)); //  a* a + b* b
                    norm_grad_vec <<  plane(0), plane(1), 0.0;
                    double norm_grad = norm_grad_vec.norm();
                    norm_grad_vec = norm_grad_vec / norm_grad;     // grad / grad.norm();
                    double normal_norm = - plane(2) / norm_grad;
                    Eigen::Vector3d flow = normal_norm * norm_grad_vec;

                    // 上面的像素光流需要转到相机系下光流
                    double focal_len_inv = (K_inv(0,0) + K_inv(1,1)) / 2;
                    // LOG(ERROR) << "focal_len_inv = " << focal_len_inv << std::endl;
                    normal_norm *= focal_len_inv;
                    // LOG(ERROR) << "normal_norm = " << normal_norm << std::endl;
                    // 由于内参仅仅是缩放尺寸，因此梯度方向并不改变 norm_grad_vec
                }*/

                /*
                Eigen::Vector3d norm_grad_vec;
                double normal_norm;
                
                norm_grad_vec = flow.cwiseAbs2() / flow.squaredNorm();
                normal_norm = norm_grad_vec.norm();
                norm_grad_vec = norm_grad_vec / normal_norm;
                */

                Eigen::Vector3d norm_grad_vec = normal_flows[i];
                double normal_norm = normal_norms[i];

                {
                    // LOG(ERROR) << "flow = " << flow << std::endl;
                    // LOG(ERROR) << "grad = " << grad << std::endl;

                    // LOG(ERROR) << "normal_flow = " << norm_grad_vec.transpose() << std::endl;
                    // LOG(ERROR) << "pixel_skew = " << pixel_skew << std::endl;
                    // LOG(ERROR) << "normal_norm = " << normal_norm << std::endl;

                    // LOG(ERROR) << "prefix = " << prefix << std::endl;
                }

                // if(normal_norm > 60)
                //     continue;

                // double normal_norm = norm_pre_points[i]; // flow.norm();
                // Eigen::Vector3d normal_flow = flow.normalized();
                // flow.normalized();

                A_all.block<3,3>(3*i, 0) = prefix * norm_grad_vec.transpose() * pixel_skew;
                b_all.segment<3>(3*i) = -1.0 * prefix * normal_norm;

                // A_all.block<3,3>(3*i, 0) = prefix * normal_flow.transpose() * pixel_skew;
                // b_all.segment<3>(3*i) = -1.0 * prefix * normal_norm;
            }

            // LOG(ERROR) << "A_all = " << A_all << std::endl;
            // LOG(ERROR) << "b_all = " << b_all << std::endl;            

            // RANSAC初始化
            std::random_device rd;
            std::mt19937 gen(rd());
            std::uniform_int_distribution<> dis(0, N - 1);

            double best_error = std::numeric_limits<double>::max();
            std::vector<int> best_inliers_index;
            Eigen::Vector3d best_angular_vec = Eigen::Vector3d::Zero();

            Eigen::MatrixXd A_sample(3 * minimum_flow, 3);
            Eigen::VectorXd b_sample(3 * minimum_flow);
            std::vector<int> indices(N);
            /*{
            // RANSAC迭代
            for (int iter = 0; iter < ransac_iter; ++iter) {
                std::vector<int> sample_indices;
                // while ((int)sample_indices.size() < minimum_flow) {
                //     int idx = dis(gen);
                //     if (std::find(sample_indices.begin(), sample_indices.end(), idx) == sample_indices.end()) {
                //         sample_indices.push_back(idx);
                //     }
                // }    
                std::iota(indices.begin(), indices.end(), 0);
                std::shuffle(indices.begin(), indices.end(), gen);
                sample_indices.assign(indices.begin(), indices.begin() + minimum_flow);


                for (int j = 0; j < minimum_flow; ++j) {
                    int idx = sample_indices[j];
                    A_sample.block<3,3>(3*j, 0) = A_all.block<3,3>(3*idx, 0);
                    b_sample.segment<3>(3*j) = b_all.segment<3>(3*idx);
                }

                Eigen::Vector3d angular_vec = A_sample.colPivHouseholderQr().solve(b_sample);

                // 计算内点和误差
                std::vector<int> inliers;
                double error_sum = 0.0;
                for (int i = 0; i < N; ++i) {
                    // Eigen::Vector3d residual = A_all.block<3,3>(3*i, 0) * angular_vec - b_all.segment<3>(3*i);
                    Eigen::VectorXd residuals = (A_all * angular_vec - b_all).reshaped(3, N).colwise().norm();

                    double error = residuals.norm();
                    // LOG(ERROR) << "error = " << error << std::endl;
                    if (error < 1.5) {  // 阈值可调 0.5
                        inliers.push_back(i);
                        error_sum += error;
                    }
                }

                if ((int)inliers.size() >= best_inliers_index.size()){ // && error_sum < best_error) {
                    best_error = error_sum;
                    best_inliers_index = inliers;
                    best_angular_vec = angular_vec;
                    // LOG(ERROR) << "best_inliers_index = " << best_inliers_index << std::endl;
                }
            }
            }*/

            // RANSAC迭代
            #pragma omp parallel
            {
                std::mt19937 gen(omp_get_thread_num() + 12345);
                std::vector<int> indices(N);
                Eigen::MatrixXd A_sample(3 * minimum_flow, 3);
                Eigen::VectorXd b_sample(3 * minimum_flow);
                std::vector<int> local_best_inliers;
                Eigen::Vector3d local_best_vec = Eigen::Vector3d::Zero();
                double local_best_error = std::numeric_limits<double>::max();

                #pragma omp for schedule(dynamic)
                for (int iter = 0; iter < ransac_iter; ++iter) {
                    // --- 随机采样 ---
                    std::iota(indices.begin(), indices.end(), 0);
                    std::shuffle(indices.begin(), indices.end(), gen);
                    std::vector<int> sample_indices(indices.begin(), indices.begin() + minimum_flow);

                    for (int j = 0; j < minimum_flow; ++j) {
                        int idx = sample_indices[j];
                        A_sample.block<3,3>(3*j, 0) = A_all.block<3,3>(3*idx, 0);
                        b_sample.segment<3>(3*j) = b_all.segment<3>(3*idx);
                    }

                    // --- 拟合 ---
                    Eigen::Vector3d angular_vec = A_sample.colPivHouseholderQr().solve(b_sample);

                    // --- 内点计算 ---
                    Eigen::VectorXd residuals = (A_all * angular_vec - b_all).reshaped(3, N).colwise().norm();

                    std::vector<int> inliers;
                    inliers.reserve(N);
                    double error_sum = 0.0;
                    for (int i = 0; i < N; ++i) {
                        double error = residuals(i);
                        if (error < 1.5) {
                            inliers.push_back(i);
                            error_sum += error;
                        }
                    }

                    // --- 局部最优 ---
                    if (inliers.size() > local_best_inliers.size() ||
                        (inliers.size() == local_best_inliers.size() && error_sum < local_best_error)) {
                        local_best_inliers = inliers;
                        local_best_vec = angular_vec;
                        local_best_error = error_sum;
                    }
                }

                // --- 全局最优更新 ---
                #pragma omp critical
                {
                    if (local_best_inliers.size() > best_inliers_index.size() ||
                        (local_best_inliers.size() == best_inliers_index.size() && local_best_error < best_error)) {
                        best_inliers_index = local_best_inliers;
                        best_angular_vec = local_best_vec;
                        best_error = local_best_error;
                    }
                }
            } // omp parallel

            // LOG(ERROR) << "best_error = " << best_error << std::endl;
            // LOG(ERROR) << "best_inliers.size = " << best_inliers_index.size() << std::endl;

            if (best_inliers_index.empty()) {
                // LOG(ERROR) << "RANSAC failed to find a good angular velocity." << std::endl;
                best_inliers.clear();
                flow_pre_points.clear();
                normal_flows.clear();
                normal_norms.clear();
                return false;
            }

            // 用内点精细求解
            /*
            Eigen::MatrixXd A_inliers(3 * best_inliers_index.size(), 3);
            Eigen::VectorXd b_inliers(3 * best_inliers_index.size());
            for (int i = 0; i < (int)best_inliers_index.size(); ++i) {
                int idx = best_inliers_index[i];
                A_inliers.block<3,3>(3*i, 0) = A_all.block<3,3>(3*idx, 0);
                b_inliers.segment<3>(3*i) = b_all.segment<3>(3*idx);
            }
            // LOG(ERROR) << "A_inliers = " << A_inliers << std::endl;
            // LOG(ERROR) << "b_inliers = " << b_inliers << std::endl;
            Eigen::Vector3d refined_angular_vec = A_inliers.colPivHouseholderQr().solve(b_inliers);
            // LOG(ERROR) << "refined_angular_vec = " << refined_angular_vec.transpose() << std::endl;
            */

            int M = 3 * best_inliers_index.size();
            Eigen::MatrixXd A_inliers(M, 3);
            Eigen::VectorXd b_inliers(M);
            for (int i = 0; i < best_inliers_index.size(); ++i) {
                int idx = best_inliers_index[i];
                A_inliers.middleRows<3>(3*i) = A_all.middleRows<3>(3*idx);
                b_inliers.segment<3>(3*i) = b_all.segment<3>(3*idx);
            }
            // LOG(ERROR) << "A_inliers = " << A_inliers << std::endl;
            // LOG(ERROR) << "b_inliers = " << b_inliers << std::endl;
            Eigen::Vector3d refined_angular_vec = A_inliers.colPivHouseholderQr().solve(b_inliers);
            // LOG(ERROR) << "refined_angular_vec = " << refined_angular_vec.transpose() << std::endl;


            // 计算残差和协方差
            Eigen::VectorXd residual = A_inliers * refined_angular_vec - b_inliers;
            double sigma_r_squared = residual.squaredNorm() / (A_inliers.rows() - A_inliers.cols());
            Eigen::Matrix3d covariance_matrix = sigma_r_squared * (A_inliers.transpose() * A_inliers).inverse();
            // Eigen::Matrix3d covariance_matrix = sigma_r_squared * (A_inliers.transpose() * A_inliers).ldlt().solve(
            //                                                         Eigen::Matrix3d::Identity());

            // 填充输出消息
            radar_vel.twist.twist.angular.x = refined_angular_vec(0);
            radar_vel.twist.twist.angular.y = refined_angular_vec(1);
            radar_vel.twist.twist.angular.z = refined_angular_vec(2);

            // 清空角速度协方差（36维数组，角速度对应下标21开始的3x3块）
            // for (int i = 0; i < 36; ++i) radar_vel.twist.covariance[i] = 0.0;
            std::fill(std::begin(radar_vel.twist.covariance), std::end(radar_vel.twist.covariance), 0.0);
            for (int i = 0; i < 3; ++i) {
                for (int j = 0; j < 3; ++j) {
                    radar_vel.twist.covariance[21 + i*6 + j] = covariance_matrix(i,j);
                }
            }

            // 填充LSQ后的 best_liners
            // std::vector<event_flow_velocity> flow_pre_points_lsq(best_inliers_index.size());
            // std::vector<cv::Point2d> best_inliers_lsq(best_inliers_index.size());
            /*
            std::vector<event_flow_velocity> flow_pre_points_lsq;
            std::vector<cv::Point2d> best_inliers_lsq;
            std::vector<Eigen::Vector3d> best_normal_flows_lsq;
            std::vector<double> best_normal_norms_lsq;
            for(auto& idx: best_inliers_index)
            {   
                best_inliers_lsq.push_back(best_inliers[idx]);
                flow_pre_points_lsq.push_back(flow_pre_points[idx]);
                best_normal_flows_lsq.push_back(normal_flows[idx]);
                best_normal_norms_lsq.push_back(normal_norms[idx]);

                // LOG(ERROR) << "input flow_pre_points_lsq = " << flow_pre_points[idx].x 
                //                 << ", " << flow_pre_points[idx].y << std::endl;
            }
            best_inliers.clear();
            flow_pre_points.clear();
            normal_flows.clear();
            normal_norms.clear();
            best_inliers = best_inliers_lsq;
            flow_pre_points = flow_pre_points_lsq;
            normal_flows = best_normal_flows_lsq;
            normal_norms = best_normal_norms_lsq;
            */


            auto select_inliers = [&](auto& vec){
                std::vector<typename std::decay<decltype(vec)>::type::value_type> tmp;
                tmp.reserve(best_inliers_index.size());
                for (auto idx : best_inliers_index) tmp.push_back(vec[idx]);
                vec.swap(tmp);
            };

            select_inliers(best_inliers);
            select_inliers(flow_pre_points);
            select_inliers(normal_flows);
            select_inliers(normal_norms);

            return true;
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
                    // LOG(ERROR) << "radar data is not new than event" << std::endl;
                    radar_doppler_velocity.pop_front();
                    radar_inliers.pop_front();
                    return false;
                }

                // 筛选区间内的IMU数据
                
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
            LOG(ERROR) << "have data";
            static long int radar_count = 0;
            radar_count ++;
            // LOG(ERROR) << "radar_count = " << radar_count;

            radar_doppler_velocity.pop_front(); 
            radar_inliers.pop_front();  

            {
                std::chrono::time_point<std::chrono::high_resolution_clock> start_time = std::chrono::high_resolution_clock::now();
                double process_time_sec = process_time.toSec();

                EventArray2EventVec();
                std::chrono::time_point<std::chrono::high_resolution_clock> time1 = std::chrono::high_resolution_clock::now();
                // bool accumu_done = AccumulateTimeImage(); // process_time_sec
                AccumulateTimeImage();
                std::chrono::time_point<std::chrono::high_resolution_clock> time2 = std::chrono::high_resolution_clock::now();
                bool have_flow = false;
                // if(accumu_done)
                    have_flow = ComputeSAEFlow();
                // else
                // {
                    // LOG(ERROR) << "Not enough event data!" << std::endl;
                    // return false;
                // }
                std::chrono::time_point<std::chrono::high_resolution_clock> time3 = std::chrono::high_resolution_clock::now();   
                
                std::chrono::time_point<std::chrono::high_resolution_clock> time4;    
                std::chrono::time_point<std::chrono::high_resolution_clock> time5; 
                // if(!(have_flow && LSQAugularVelocityEsti(twist_)))
                if(!(have_flow))
                {
                    LOG(ERROR) << "No Flow" << std::endl;
                }


                // 法向光流
                if(!(have_flow && NormalAngularVelocityEsti(twist_)))
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
                    normal_flows.clear();
                    normal_norms.clear();
                    return false;
                } 
                else
                {
                    // LOG(ERROR) << "PublishTimeImages" << std::endl;
                    time4 = std::chrono::high_resolution_clock::now();
                    
                    // PublishTimeImages(TimeImage1, TimeImage1_time);
                    time5 = std::chrono::high_resolution_clock::now();
                }
                // std::chrono::time_point<std::chrono::high_resolution_clock> time4 = std::chrono::high_resolution_clock::now();
                
                // std::chrono::time_point<std::chrono::high_resolution_clock> time5 = std::chrono::high_resolution_clock::now();

                assert(twist_.header.stamp == cur_inliers.header.stamp 
                    && "radar_inliers not valid Or Time is not correct!");

                // INFO_Velocity(twist_);
                // LOG_Velocity(twist_, "/home/hao/Desktop/twist_ws/src/TwistEstimator/output/detector.tum");


                // LOG(ERROR) << "Final Debug for flow: " << best_inliers.size() << std::endl;
                // LOG(ERROR) << "Debug Flow: " << flow_pre_points.front().x  << ", " << flow_pre_points.front().y << std::endl;
                for(auto& f : flow_pre_points)
                {
                    // LOG(ERROR) << "f = " << f.x << ", " << f.y;
                }
                
                twist_result2_.push_back(TwistData2(twist_, cur_inliers, best_inliers, flow_pre_points, normal_flows, normal_norms));
                std::chrono::time_point<std::chrono::high_resolution_clock> end_time = std::chrono::high_resolution_clock::now();

                {
                    std::chrono::duration<double, std::milli> elapsed;
                    elapsed = end_time - start_time;
                    // LOG(ERROR) << "Detector Total Time: " << std::setprecision(18) << elapsed.count() << std::endl;
                    elapsed = time1 - start_time;
                    // LOG(ERROR) << "EventArray2EventVec: " << std::setprecision(18) << elapsed.count() << std::endl;
                    elapsed = time2 - time1;
                    // LOG(ERROR) << "AccumulateTimeImage: " << std::setprecision(18) << elapsed.count() << std::endl;
                    elapsed = time3 - time2;
                    // LOG(ERROR) << "CalculateOpFlowPrepointSingleFit: " << std::setprecision(18) << elapsed.count() << std::endl;
                    elapsed = time4 - time3;
                    // LOG(ERROR) << "LSQAugularVelocityEsti: " << std::setprecision(18) << elapsed.count() << std::endl;
                    elapsed = time5 - time4;
                    // LOG(ERROR) << "PublishTimeImages: " << std::setprecision(18) << elapsed.count() << std::endl;
                }

            }
            return true;
        }


        // TODO: 数据拷贝可以使用指针来进行
        // 获取光流速度
        geometry_msgs::TwistWithCovarianceStamped GetTwist() const {
            return twist_;
        }

        TwistData2 GetTwistData2() {
            TwistData2 twist_result_temp_ = twist_result2_.front();
            twist_result2_.pop_front();
            return twist_result_temp_;          
        }

public:
    std::vector<sensor_msgs::Image::Ptr> raw_img_buffer;
    std::deque<dvs_msgs::EventArray::Ptr> event_stream;
    std::deque<geometry_msgs::TwistWithCovarianceStamped> radar_doppler_velocity;
    std::deque<sensor_msgs::PointCloud2> radar_inliers;
    std::deque<TwistData> twist_result_;

    std::deque<TwistData2> twist_result2_;


private:
    Eigen::Matrix3d K;
    cv::Mat K_cv;
    ros::NodeHandle nh_this_;
    cv_bridge::CvImage cv_image_;           // transport TimeImage

    ros::Publisher image_pub_;
    ros::Publisher pub_event_image_;
    ros::Publisher pub_raw_image_;
    ros::Publisher pub_cloud;

    int sensor_width;
    int sensor_height;
    long int t1_image_count;
    bool ignore_polarity;
    Eigen::Matrix4d T_re;
    int radius;
    ros::Time process_time;

    // Input:
    std::vector<dvs_msgs::Event> event_buffer;
    geometry_msgs::TwistWithCovarianceStamped twist_;
    Eigen::Vector3d linear_vel;

    // Output:
    sensor_msgs::CameraInfo cam_info;
    image_geometry::PinholeCameraModel camera_model;
    // std::vector<Eigen::Vector2d> valid_flows;
    std::vector<event_flow_velocity> flow_pre_points;
    std::vector<cv::Point2d> best_inliers;
    std::vector<double> norm_pre_points;
    std::vector<Eigen::Vector4d> plane_params;
    std::vector<Eigen::Vector3d> normal_flows;
    std::vector<double> normal_norms;
    SAE::Ptr sae_ptr_;
    double sae_time;

    cv::Mat kernel_;

    double grid_size;

    cv::Mat TimeImage1;

};