// This file is part of RIO - Radar Inertial Odometry and Radar ego velocity estimation.
// Copyright (C) 2021  Christopher Doer <christopher.doer@kit.edu>
// (Institute of Control Systems, Karlsruhe Institute of Technology)

// RIO is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
//(at your option) any later version.

// RIO is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.

// You should have received a copy of the GNU General Public License
// along with RIO.  If not, see <https://www.gnu.org/licenses/>.

#define PCL_NO_PRECOMPILE

#include <random>
#include <algorithm>
#include <iostream>
#include <sstream>

#include <angles/angles.h>

#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/pcl_macros.h>

#include <pcl_ros/transforms.h>

#include <fstream>

#include <chrono>

#include "radar_ego_velocity_estimator/radar_point_cloud.h"

#include "radar_ego_velocity_estimator/radar_ego_velocity_estimator.h"

using namespace std;
using namespace rio;

static RadarPointCloudType toRadarPointCloudType(const Vector11& item, const RadarEgoVelocityEstimatorIndices& idx)
{
  RadarPointCloudType point;
  point.x             = item[idx.x_r];
  point.y             = item[idx.y_r];
  point.z             = item[idx.z_r];
  point.doppler       = -item[idx.doppler];
  point.intensity     = item[idx.snr_db];
  return point;
}

bool RadarEgoVelocityEstimator::estimate(const sensor_msgs::PointCloud2& radar_scan_msg,
                                         Vector3& v_r,
                                         Vector3& sigma_v_r)
{
  sensor_msgs::PointCloud2 inlier_radar_msg, outlier_radar_msg;
  return estimate(radar_scan_msg, v_r, sigma_v_r, inlier_radar_msg, outlier_radar_msg);
}
/*
bool RadarEgoVelocityEstimator::estimate(const sensor_msgs::PointCloud2& radar_scan_msg,
                                         Vector3& v_r,
                                         Vector3& sigma_v_r,
                                         sensor_msgs::PointCloud2& inlier_radar_msg,
                                         sensor_msgs::PointCloud2& outlier_radar_msg)
{
  auto radar_scan(new pcl::PointCloud<RadarPointCloudType>);
  auto radar_scan_inlier(new pcl::PointCloud<RadarPointCloudType>); // static objects
  auto radar_scan_outlier(new pcl::PointCloud<RadarPointCloudType>);

  std::chrono::time_point<std::chrono::high_resolution_clock> doppler_start_time = std::chrono::high_resolution_clock::now();

  // file.open("/home/hao/Desktop/doppler_debug.txt", std::ios::out | std::ios::app);
  // std::fstream file("/home/hao/Desktop/radar-event-new/src/TwistEstimator/output/doppler_debug.txt", std::ios::out | std::ios::app);
  std::fstream file("/home/hao/Desktop/radar-event-new/src/TwistEstimator/output/doppler_debug.txt", std::ios::out | std::ios::app);

  // if(config_.radar_type == "ti_mmwave")
  //   for (auto& field : radar_scan_msg.fields) {
  //       if (field.name == "velocity") {
  //           field.name = "doppler";  // 将 velocity 改为 doppler
  //       }
  //   }


  bool success = false;
  pcl::fromROSMsg (radar_scan_msg, *radar_scan);
  file << "radar_scan.size = " << radar_scan->size() << std::endl;
  // std::chrono::time_point<std::chrono::high_resolution_clock> doppler_start_time = std::chrono::high_resolution_clock::now();
  std::chrono::time_point<std::chrono::high_resolution_clock> doppler_init_time = std::chrono::high_resolution_clock::now();
  std::chrono::time_point<std::chrono::high_resolution_clock> doppler_ransac_start_time;
  std::chrono::time_point<std::chrono::high_resolution_clock> doppler_ransac_end_time;
  if (1) // pcl2msgToPcl(radar_scan_msg, *radar_scan)
  {
    std::vector<Vector11> valid_targets;
    for (uint i = 0; i < radar_scan->size(); ++i)
    {
      const auto target = radar_scan->at(i);
      const double r      = Vector3(target.x, target.y, target.z).norm();

      Real azimuth   = std::atan2(target.y, target.x);
      Real elevation = std::atan2(std::sqrt(target.x * target.x + target.y * target.y), target.z)- M_PI_2;

      // ROS_INFO("radar config = [");
      // ROS_INFO("config_.dist = [ %f, %f], actual = %f", config_.min_dist, config_.max_dist, r);
      // ROS_INFO("config_.db = %f, actual = %f", config_.min_db, target.intensity);
      // ROS_INFO("config_.azimuth_thresh_deg = %f, actual = %f", angles::from_degrees(config_.azimuth_thresh_deg), std::fabs(azimuth));
      // ROS_INFO("config_.elevation_thresh_deg = %f, actual = %f", angles::from_degrees(config_.elevation_thresh_deg), std::fabs(elevation));
      // ROS_INFO("]");

      
      // file << "points " << i << ": " << std::endl;
      // file << "config_.dist = [ " << config_.min_dist << "," << config_.max_dist << ", actual =  " << r << std::endl;
      // file << "config_.db = " << config_.min_db << ", actual = " << target.intensity << std::endl; 
      // file << "config_.azimuth_thresh_deg = " << angles::from_degrees(config_.azimuth_thresh_deg) 
      //       << ", actual = " << std::fabs(azimuth) << std::endl;                
      // file << "config_.elevation_thresh_deg =  " << angles::from_degrees(config_.elevation_thresh_deg) 
      //       << ", actual = " << std::fabs(elevation) << std::endl;


      if (r > config_.min_dist && r < config_.max_dist && target.intensity > config_.min_db &&
          std::fabs(azimuth) < angles::from_degrees(config_.azimuth_thresh_deg) &&
          std::fabs(elevation) < angles::from_degrees(config_.elevation_thresh_deg))
      {
        Vector11 v_pt; // features of a point
        v_pt << target.x, target.y, target.z, target.intensity, -target.doppler * config_.doppler_velocity_correction_factor,
                r, azimuth, elevation, target.x / r, target.y / r, target.z / r;
        valid_targets.emplace_back(v_pt);
        // file << "distance = " << r << std::endl;
      }
      // HAO Debug
      // else{
      //   file << "Failure condition check" << (r > config_.min_dist) << ", " << (r < config_.max_dist) << ", " 
      //       << (target.intensity > config_.min_db) << ", " << (std::fabs(azimuth) < angles::from_degrees(config_.azimuth_thresh_deg)) << ", "
      //       << (std::fabs(elevation) < angles::from_degrees(config_.elevation_thresh_deg)) // << ", distance = " << r  
      //       << " " << angles::to_degrees(azimuth) << " " << angles::to_degrees(elevation) << endl;
      //   ;
      // }
    }

    

    // ROS_INFO("valid_targets = %d", valid_targets.size());

    if (valid_targets.size() > 2)
    {
      // check for zero velocity
      std::vector<double> v_dopplers;
      for (const auto& v_pt : valid_targets) v_dopplers.emplace_back(std::fabs(v_pt[idx_.doppler]));
      const size_t n = v_dopplers.size() * (1.0 - config_.allowed_outlier_percentage);
      std::nth_element(v_dopplers.begin(), v_dopplers.begin() + n, v_dopplers.end());
      const auto median = v_dopplers[n];

      // ROS_INFO("checkout zero velocity = %f, actually = %f", config_.thresh_zero_velocity, median);  
      file << "checkout zero velocity = " << config_.thresh_zero_velocity << ", actually = " << median << std::endl;
      if (median < config_.thresh_zero_velocity)
      {
        // ROS_INFO_STREAM_THROTTLE(0.5, kPrefix << "Zero velocity detected!");
        v_r = Vector3(0, 0, 0);
        sigma_v_r =
            Vector3(config_.sigma_zero_velocity_x, config_.sigma_zero_velocity_y, config_.sigma_zero_velocity_z);

        int index = 0;
        for (auto& item : valid_targets)
        {
          if(index ++ > 20)
            break;
          if (std::fabs(item[idx_.doppler]) < config_.thresh_zero_velocity)
          {
            item[idx_.doppler] = 0;
            radar_scan_inlier->push_back(toRadarPointCloudType(item, idx_));
          }
          else
            radar_scan_outlier->push_back(toRadarPointCloudType(item, idx_));
        }
        success = true;

        // 讨论是否需要0速度
        // success = true;

      }
      else
      {
        file << "False: velocity median < config_.thresh_zero_velocity " << "[" << median << "," << config_.thresh_zero_velocity << "]" << std::endl;
        file << "valid_targets.size =  " << valid_targets.size() << std::endl;

        std::chrono::time_point<std::chrono::high_resolution_clock> doppler_filter_time = std::chrono::high_resolution_clock::now();

        // LSQ velocity estimation
        Matrix radar_data(valid_targets.size(), 4);  // rx, ry, rz, v
        uint idx = 0, idx_o = 0;
        for (const auto& v_pt : valid_targets)
          radar_data.row(idx++) = Vector4(v_pt[idx_.normalized_x], v_pt[idx_.normalized_y], v_pt[idx_.normalized_z], v_pt[idx_.doppler]);

        file << "check 1 : radar_data = " << radar_data << std::endl;



        // 筛选 2 * sigma 的数值 [0.6827 0.9545 0.9973]
        Matrix radar_filtered;
        std::vector<Vector11> valid_targets_filted;
        {
            // **计算第四列 (速度 v) 的均值和标准差**
            Eigen::VectorXd velocity = radar_data.col(3);  // 提取 v 列
            double mean_v = velocity.mean();
            double std_v = sqrt(((velocity.array() - mean_v).square().sum()) / (velocity.size() - 1));

            // **计算逻辑掩码，判断是否在 2 * sigma 内**
            Eigen::Array<bool, Eigen::Dynamic, 1> mask = (velocity.array() >= mean_v - 2 * std_v) && 
                                          (velocity.array() <= mean_v + 2 * std_v);

            // **筛选有效行**
            vector<int> valid_rows;
            for (int i = 0; i < radar_data.rows(); ++i) {
                if (mask(i)) {
                    valid_rows.push_back(i);
                }
            }

            // **构建筛选后的矩阵**
            radar_filtered = Eigen::MatrixXd(valid_rows.size(), 4);
            for (size_t i = 0; i < valid_rows.size(); ++i) {
                radar_filtered.row(i) = radar_data.row(valid_rows[i]);

                valid_targets_filted.push_back(valid_targets.at(valid_rows[i]));
            }
        }
        radar_data = radar_filtered;
        valid_targets = valid_targets_filted;

        file << "check 2 : radar_data = " << radar_data.rows() 
             << ", valid_targets = " << valid_targets.size() << std::endl;

        // 3-17 修改
        /*{
        // LSQ velocity estimation
        Matrix radar_data(valid_targets.size(), 4);  // rx, ry, rz, v
        uint idx = 0, idx_o = 0;
        for (const auto& v_pt : valid_targets)
          radar_data.row(idx++) = Vector4(v_pt[idx_.normalized_x], v_pt[idx_.normalized_y], v_pt[idx_.normalized_z], v_pt[idx_.doppler]);

        file << "check 1 : radar_data = " << radar_data << std::endl;
        }
        doppler_ransac_start_time = std::chrono::high_resolution_clock::now();

        if (config_.use_ransac)
        {
          std::vector<uint> inlier_idx_best;
          std::vector<uint> outlier_idx_best;
          success = solve3DFullRansac(radar_data, v_r, sigma_v_r, inlier_idx_best, outlier_idx_best);
          file << "check 2 : radar_data = " << radar_data << std::endl;
          file << "solve 3D Ransac " << ((success)? "True":"False") << std::endl;
          file << "solve v_r =  " << v_r(0) << ", " << v_r(1) << ", " << v_r(2) << std::endl;

          file << "check inlier_idx_best size = " << inlier_idx_best.size() << std::endl;

          // Insert inlier points to the cloud
          for (const auto& idx : inlier_idx_best)
            radar_scan_inlier->push_back(toRadarPointCloudType(valid_targets.at(idx), idx_));
          for (const auto& idx : outlier_idx_best)
            radar_scan_outlier->push_back(toRadarPointCloudType(valid_targets.at(idx), idx_));
        
        }
        else
        {
          for (const auto& item : valid_targets) radar_scan_inlier->push_back(toRadarPointCloudType(item, idx_));
          success = solve3DFull(radar_data, v_r, sigma_v_r);
          file << "check 2 : radar_data = " << radar_data << std::endl;

          file << "solve 3D Full " << ((success)? "True":"False") << std::endl;
        }

        doppler_ransac_end_time = std::chrono::high_resolution_clock::now();
      }
    }
    // else ROS_INFO("To small valid_targets (< 2) in radar_scan (%ld)", radar_scan->size());

    radar_scan_inlier->height = 1;
    radar_scan_inlier->width  = radar_scan_inlier->size();



    // std::cout << "RadarEstimator: radar_scan_inlier->width = " << radar_scan_inlier->width << std::endl;

    pcl::PCLPointCloud2 tmp;
    pcl::toPCLPointCloud2<RadarPointCloudType>(*radar_scan_inlier, tmp);
    pcl_conversions::fromPCL(tmp, inlier_radar_msg);
    inlier_radar_msg.header = radar_scan_msg.header;

    radar_scan_outlier->height = 1;
    radar_scan_outlier->width  = radar_scan_outlier->size();


    file << "inliers = " << inlier_radar_msg.width << ", outlier = " << radar_scan_outlier->width << std::endl;

    pcl::PCLPointCloud2 tmp_o;
    pcl::toPCLPointCloud2<RadarPointCloudType>(*radar_scan_outlier, tmp_o);
    pcl_conversions::fromPCL(tmp_o, outlier_radar_msg);
    outlier_radar_msg.header = radar_scan_msg.header;
  }
  // file << "inliers = " << inlier_radar_msg->width << std::endl;

  // if(success)
  //   ROS_INFO("Ego Velocity estimation Successful");
  // else
  //   ROS_INFO("Ego Velocity estimation Failed");

  // file.close();
  // file << "sigma_v_r = [" << sigma_v_r.transpose() << "]" << std::endl;


  file << "inliers = " << inlier_radar_msg.width << std::endl;


  std::chrono::time_point<std::chrono::high_resolution_clock> end_time = std::chrono::high_resolution_clock::now();

  {
      std::chrono::duration<double, std::milli> elapsed;
      elapsed = end_time - doppler_start_time;
      file <<  "Total Time: " << elapsed.count() << std::endl;
      elapsed = doppler_init_time - doppler_start_time;
      file <<  "Init: " << elapsed.count() << std::endl;
      elapsed = doppler_ransac_start_time - doppler_init_time;
      file <<  "Filter: " << elapsed.count() << std::endl;
      elapsed = doppler_ransac_end_time - doppler_ransac_start_time;
      file <<  "Ransac: " << elapsed.count() << std::endl;
      elapsed = end_time - doppler_ransac_end_time;
      file <<  "Publish: " << elapsed.count() << std::endl;
  }
  file.close();
  return success;
}
*/


bool RadarEgoVelocityEstimator::estimate(const sensor_msgs::PointCloud2& radar_scan_msg,
                                         Vector3& v_r,
                                         Vector3& sigma_v_r,
                                         sensor_msgs::PointCloud2& inlier_radar_msg,
                                         sensor_msgs::PointCloud2& outlier_radar_msg)
{
  auto radar_scan(new pcl::PointCloud<RadarPointCloudType>);
  auto radar_scan_inlier(new pcl::PointCloud<RadarPointCloudType>); // static objects
  // pcl::PointCloud<RadarPointCloudType>::Ptr radar_scan_inlier(new pcl::PointCloud<RadarPointCloudType>());

  auto radar_scan_outlier(new pcl::PointCloud<RadarPointCloudType>);

  // auto radar_scan = boost::make_shared<pcl::PointCloud<RadarPointCloudType>>();
  // auto radar_scan_inlier = boost::make_shared<pcl::PointCloud<RadarPointCloudType>>();
  // auto radar_scan_outlier = boost::make_shared<pcl::PointCloud<RadarPointCloudType>>();

  std::chrono::time_point<std::chrono::high_resolution_clock> doppler_start_time = std::chrono::high_resolution_clock::now();

  // file.open("/home/hao/Desktop/doppler_debug.txt", std::ios::out | std::ios::app);
  // std::fstream file("/home/hao/Desktop/radar-event-new/src/TwistEstimator/output/doppler_debug.txt", std::ios::out | std::ios::app);
  std::fstream file("/home/hao/Desktop/radar-event-new/src/TwistEstimator/output/doppler_debug.txt", std::ios::out | std::ios::app);

  // if(config_.radar_type == "ti_mmwave")
  //   for (auto& field : radar_scan_msg.fields) {
  //       if (field.name == "velocity") {
  //           field.name = "doppler";  // 将 velocity 改为 doppler
  //       }
  //   }


  bool success = false;
  pcl::fromROSMsg (radar_scan_msg, *radar_scan);
  file << "radar_scan.size = " << radar_scan->size() << std::endl;
  // std::chrono::time_point<std::chrono::high_resolution_clock> doppler_start_time = std::chrono::high_resolution_clock::now();
  std::chrono::time_point<std::chrono::high_resolution_clock> doppler_init_time = std::chrono::high_resolution_clock::now();
  std::chrono::time_point<std::chrono::high_resolution_clock> doppler_ransac_start_time;
  std::chrono::time_point<std::chrono::high_resolution_clock> doppler_ransac_end_time;
  const Real azimuth_thresh_rad   = angles::from_degrees(config_.azimuth_thresh_deg);
  const Real elevation_thresh_rad = angles::from_degrees(config_.elevation_thresh_deg);
  if (1) // pcl2msgToPcl(radar_scan_msg, *radar_scan)
  {
    std::vector<Vector11> valid_targets;
    // std::vector<Vector11, Eigen::aligned_allocator<Vector11>> valid_targets;
    valid_targets.reserve(radar_scan->size());
    for (uint i = 0; i < radar_scan->size(); ++i)
    {
      const auto target = radar_scan->at(i);      
      const double xy_norm = std::sqrt(target.x * target.x + target.y * target.y);
      const double r = std::sqrt(xy_norm * xy_norm + target.z * target.z);

      Real azimuth   = std::atan2(target.y, target.x);
      Real elevation = std::atan2(xy_norm, target.z)- M_PI_2;

      // ROS_INFO("radar config = [");
      // ROS_INFO("config_.dist = [ %f, %f], actual = %f", config_.min_dist, config_.max_dist, r);
      // ROS_INFO("config_.db = %f, actual = %f", config_.min_db, target.intensity);
      // ROS_INFO("config_.azimuth_thresh_deg = %f, actual = %f", angles::from_degrees(config_.azimuth_thresh_deg), std::fabs(azimuth));
      // ROS_INFO("config_.elevation_thresh_deg = %f, actual = %f", angles::from_degrees(config_.elevation_thresh_deg), std::fabs(elevation));
      // ROS_INFO("]");

      
      // file << "points " << i << ": " << std::endl;
      // file << "config_.dist = [ " << config_.min_dist << "," << config_.max_dist << ", actual =  " << r << std::endl;
      // file << "config_.db = " << config_.min_db << ", actual = " << target.intensity << std::endl; 
      // file << "config_.azimuth_thresh_deg = " << angles::from_degrees(config_.azimuth_thresh_deg) 
      //       << ", actual = " << std::fabs(azimuth) << std::endl;                
      // file << "config_.elevation_thresh_deg =  " << angles::from_degrees(config_.elevation_thresh_deg) 
      //       << ", actual = " << std::fabs(elevation) << std::endl;


      if (r > config_.min_dist && r < config_.max_dist && target.intensity > config_.min_db &&
          std::fabs(azimuth) < azimuth_thresh_rad &&
          std::fabs(elevation) < elevation_thresh_rad)
      {
        Vector11 v_pt; // features of a point
        v_pt << target.x, target.y, target.z, target.intensity, -target.doppler * config_.doppler_velocity_correction_factor,
                r, azimuth, elevation, target.x / r, target.y / r, target.z / r;
        valid_targets.emplace_back(v_pt);
        // file << "distance = " << r << std::endl;
      }
      // HAO Debug
      // else{
      //   file << "Failure condition check" << (r > config_.min_dist) << ", " << (r < config_.max_dist) << ", " 
      //       << (target.intensity > config_.min_db) << ", " << (std::fabs(azimuth) < angles::from_degrees(config_.azimuth_thresh_deg)) << ", "
      //       << (std::fabs(elevation) < angles::from_degrees(config_.elevation_thresh_deg)) // << ", distance = " << r  
      //       << " " << angles::to_degrees(azimuth) << " " << angles::to_degrees(elevation) << endl;
      //   ;
      // }
    }

    

    // ROS_INFO("valid_targets = %d", valid_targets.size());

    if (valid_targets.size() > 2)
    {
      // check for zero velocity
      std::vector<double> v_dopplers;
      v_dopplers.reserve(valid_targets.size());
      std::transform(valid_targets.begin(), valid_targets.end(), std::back_inserter(v_dopplers),
                 [&](const Vector11& pt) { return std::fabs(pt[idx_.doppler]); });
      const size_t n = static_cast<size_t>(v_dopplers.size() * (1.0 - config_.allowed_outlier_percentage));
      // for (const auto& v_pt : valid_targets) v_dopplers.emplace_back(std::fabs(v_pt[idx_.doppler]));
      // const size_t n = v_dopplers.size() * (1.0 - config_.allowed_outlier_percentage);
      std::nth_element(v_dopplers.begin(), v_dopplers.begin() + n, v_dopplers.end());
      // const auto median = v_dopplers[n];
      const double median = v_dopplers[n];

      // ROS_INFO("checkout zero velocity = %f, actually = %f", config_.thresh_zero_velocity, median);  
      file << "checkout zero velocity = " << config_.thresh_zero_velocity << ", actually = " << median << std::endl;
      if (median < config_.thresh_zero_velocity)
      {
        // ROS_INFO_STREAM_THROTTLE(0.5, kPrefix << "Zero velocity detected!");
        v_r = Vector3(0, 0, 0);
        sigma_v_r =
            Vector3(config_.sigma_zero_velocity_x,
                    config_.sigma_zero_velocity_y, 
                    config_.sigma_zero_velocity_z);

        int index = 0;
        for (auto& item : valid_targets)
        {
          if(index ++ > 20)
            break;
          if (std::fabs(item[idx_.doppler]) < config_.thresh_zero_velocity)
          {
            item[idx_.doppler] = 0;
            radar_scan_inlier->push_back(toRadarPointCloudType(item, idx_));
          }
          else
            radar_scan_outlier->push_back(toRadarPointCloudType(item, idx_));
        }
        success = true;

        // 讨论是否需要0速度
        // success = true;

      }
      else
      {
        file << "False: velocity median < config_.thresh_zero_velocity " << "[" << median << "," << config_.thresh_zero_velocity << "]" << std::endl;
        file << "valid_targets.size =  " << valid_targets.size() << std::endl;

        std::chrono::time_point<std::chrono::high_resolution_clock> doppler_filter_time = std::chrono::high_resolution_clock::now();

        // LSQ velocity estimation
        if (valid_targets.size() < 3)
            return false;

        Matrix radar_data(valid_targets.size(), 4);  // rx, ry, rz, v
        uint idx = 0;
        for (const auto& v_pt : valid_targets)
            radar_data.row(idx++) = Vector4(v_pt[idx_.normalized_x], v_pt[idx_.normalized_y], v_pt[idx_.normalized_z], v_pt[idx_.doppler]);

        file << "check 1 : radar_data = " << radar_data << std::endl;

        // 筛选 2 * sigma 的数值 [0.6827 0.9545 0.9973]
        Matrix radar_filtered;  // 初始化矩阵
        std::vector<Vector11> valid_targets_filted;
        // std::vector<Vector11, Eigen::aligned_allocator<Vector11>> valid_targets_filted;

        {
            // **计算第四列 (速度 v) 的均值和标准差**
            Vector velocity = radar_data.col(3);  // 提取 v 列
            double mean_v = velocity.mean();
            double std_v = sqrt(((velocity.array() - mean_v).square().sum()) / (velocity.size() - 1));

            std::fstream radar_doppler_std_file("/home/hao/Desktop/radar-event-new/src/TwistEstimator/output/doppler_std.csv", std::ios::out | std::ios::app);
            radar_doppler_std_file << std_v << std::endl;
            radar_doppler_std_file.close();

            // **计算逻辑掩码，判断是否在 2 * sigma 内**
            Eigen::Array<bool, Eigen::Dynamic, 1> mask = (velocity.array() >= mean_v - 2 * std_v) && 
                                                      (velocity.array() <= mean_v + 2 * std_v);

            // **筛选有效行**
            std::vector<int> valid_rows;
            for (int i = 0; i < radar_data.rows(); ++i) {
                if (mask(i)) {
                    valid_rows.push_back(i);
                }
            }

            file << "check 5 : valid_rows.size() = " << valid_rows.size() << std::endl;
            if (valid_rows.size() < 3)
                return false;

            // **直接筛选并构建新的矩阵和目标数据**
            radar_filtered.resize(valid_rows.size(), 4); // 预分配内存
            valid_targets_filted.reserve(valid_rows.size()); // 预分配内存
            for (size_t i = 0; i < valid_rows.size(); ++i) {
                radar_filtered.row(i) = radar_data.row(valid_rows[i]);
                valid_targets_filted.push_back(valid_targets[valid_rows[i]]);
            }
        }

        radar_data = radar_filtered;  // 将筛选后的数据赋回 radar_data
        valid_targets = valid_targets_filted;  // 更新有效目标数据

        file << "check 2 : radar_data = " << radar_data.rows() 
            << ", valid_targets = " << valid_targets.size() << std::endl;

        // 3-17 修改
        /*{
        // LSQ velocity estimation
        Matrix radar_data(valid_targets.size(), 4);  // rx, ry, rz, v
        uint idx = 0, idx_o = 0;
        for (const auto& v_pt : valid_targets)
          radar_data.row(idx++) = Vector4(v_pt[idx_.normalized_x], v_pt[idx_.normalized_y], v_pt[idx_.normalized_z], v_pt[idx_.doppler]);

        file << "check 1 : radar_data = " << radar_data << std::endl;
        } */ 
        doppler_ransac_start_time = std::chrono::high_resolution_clock::now();

        if (config_.use_ransac)
        {
          std::vector<uint> inlier_idx_best;
          std::vector<uint> outlier_idx_best;
          success = solve3DFullRansac(radar_data, v_r, sigma_v_r, inlier_idx_best, outlier_idx_best);
          file << "check 2 : radar_data = " << radar_data << std::endl;
          file << "solve 3D Ransac " << ((success)? "True":"False") << std::endl;
          file << "solve v_r =  " << v_r(0) << ", " << v_r(1) << ", " << v_r(2) << std::endl;

          file << "check inlier_idx_best size = " << inlier_idx_best.size() << std::endl;

          // Insert inlier points to the cloud
          for (const auto& idx : inlier_idx_best)
            radar_scan_inlier->push_back(toRadarPointCloudType(valid_targets.at(idx), idx_));
          for (const auto& idx : outlier_idx_best)
            radar_scan_outlier->push_back(toRadarPointCloudType(valid_targets.at(idx), idx_));
        
        }
        else
        {
          // LOG(ERROR) << "valid_targets.size = " << valid_targets.size() << std::endl;
          std::cout << "valid_targets.size = " << valid_targets.size() << std::endl;
          for (const auto& item : valid_targets) 
            radar_scan_inlier->push_back(toRadarPointCloudType(item, idx_));
            // radar_scan_inlier->points.emplace_back(toRadarPointCloudType(item, idx_));
          success = solve3DFull(radar_data, v_r, sigma_v_r);
          file << "check 2 : radar_data = " << radar_data << std::endl;

          file << "solve 3D Full " << ((success)? "True":"False") << std::endl;
        }

        doppler_ransac_end_time = std::chrono::high_resolution_clock::now();
      }
    }
    // else ROS_INFO("To small valid_targets (< 2) in radar_scan (%ld)", radar_scan->size());

    radar_scan_inlier->height = 1;
    radar_scan_inlier->width  = radar_scan_inlier->size();



    // std::cout << "RadarEstimator: radar_scan_inlier->width = " << radar_scan_inlier->width << std::endl;

    pcl::PCLPointCloud2 tmp;
    pcl::toPCLPointCloud2<RadarPointCloudType>(*radar_scan_inlier, tmp);
    pcl_conversions::fromPCL(tmp, inlier_radar_msg);
    inlier_radar_msg.header = radar_scan_msg.header;

    radar_scan_outlier->height = 1;
    radar_scan_outlier->width  = radar_scan_outlier->size();


    file << "inliers = " << inlier_radar_msg.width << ", outlier = " << radar_scan_outlier->width << std::endl;

    pcl::PCLPointCloud2 tmp_o;
    pcl::toPCLPointCloud2<RadarPointCloudType>(*radar_scan_outlier, tmp_o);
    pcl_conversions::fromPCL(tmp_o, outlier_radar_msg);
    outlier_radar_msg.header = radar_scan_msg.header;
  }
  // file << "inliers = " << inlier_radar_msg->width << std::endl;

  // if(success)
  //   ROS_INFO("Ego Velocity estimation Successful");
  // else
  //   ROS_INFO("Ego Velocity estimation Failed");

  // file.close();
  // file << "sigma_v_r = [" << sigma_v_r.transpose() << "]" << std::endl;


  file << "inliers = " << inlier_radar_msg.width << std::endl;


  std::chrono::time_point<std::chrono::high_resolution_clock> end_time = std::chrono::high_resolution_clock::now();

  {
      std::fstream time_total("/home/hao/Desktop/radar-event-new/src/TwistEstimator/output/time.csv", std::ios::out | std::ios::app);
      std::chrono::duration<double, std::milli> elapsed;
      elapsed = end_time - doppler_start_time;
      file <<  "Total Time: " << elapsed.count() << std::endl;
      time_total << elapsed.count() << ", ";
      elapsed = doppler_init_time - doppler_start_time;
      file <<  "Init: " << elapsed.count() << std::endl;
      time_total << elapsed.count() << ", ";
      elapsed = doppler_ransac_start_time - doppler_init_time;
      file <<  "Filter: " << elapsed.count() << std::endl;
      time_total << elapsed.count() << ", ";
      elapsed = doppler_ransac_end_time - doppler_ransac_start_time;
      file <<  "Ransac: " << elapsed.count() << std::endl;
      time_total << elapsed.count() << ", ";
      elapsed = end_time - doppler_ransac_end_time;
      file <<  "Publish: " << elapsed.count() << std::endl;
      time_total << elapsed.count() << std::endl;
  }
  file.close();
  return success;
}


/*
bool RadarEgoVelocityEstimator::solve3DFullRansac(const Matrix& radar_data,
                                                  Vector3& v_r,
                                                  Vector3& sigma_v_r,
                                                  std::vector<uint>& inlier_idx_best,
                                                  std::vector<uint>& outlier_idx_best)
{
  Matrix H_all(radar_data.rows(), 3);
  H_all.col(0)       = radar_data.col(0);
  H_all.col(1)       = radar_data.col(1);
  H_all.col(2)       = radar_data.col(2);
  const Vector y_all = radar_data.col(3);

  std::vector<uint> idx(radar_data.rows());
  for (uint k = 0; k < radar_data.rows(); ++k) idx[k] = k;

  std::random_device rd;
  std::mt19937 g(rd());

  // HAO TODO: 
  std::fstream radar_inlier("/home/hao/Desktop/radar-event-new/src/TwistEstimator/output/radar_inliers.debug", std::ios::out | std::ios::app);
  // radar_inlier.clear();

  if (radar_data.rows() >= config_.N_ransac_points)
  {
    for (uint k = 0; k < ransac_iter_; ++k)
    {
      std::shuffle(idx.begin(), idx.end(), g);
      Matrix radar_data_iter;
      radar_data_iter.resize(config_.N_ransac_points, 4);

      for (uint i = 0; i < config_.N_ransac_points; ++i) radar_data_iter.row(i) = radar_data.row(idx.at(i));
      bool rtn = solve3DFull(radar_data_iter, v_r, sigma_v_r, false);
      radar_inlier << "check 3 : radar_data = " << radar_data << std::endl;
      if (rtn == false) {ROS_INFO("Failure at solve3DFullRansac() 1");
      radar_inlier << "Failure at solve3DFullRansac() 1" << std::endl;}
      if (rtn)
      {
        // radar_inlier << "y_all = " << y_all << std::endl;
        // radar_inlier << "H_all = " << H_all << std::endl;
        radar_inlier << "v_r = " << v_r << std::endl;

        const Vector err = (y_all - H_all * v_r).array().abs();
        std::vector<uint> inlier_idx;
        std::vector<uint> outlier_idx;
        /*for (uint j = 0; j < err.rows(); ++j)
        {
          // ROS_INFO("Error: %f",err(j));
          radar_inlier << "err(j) = " << err(j) << " required = " << config_.inlier_thresh << std::endl;
          if (err(j) < config_.inlier_thresh)     
            // find inlier points
            inlier_idx.emplace_back(j);
          else
            outlier_idx.emplace_back(j);
        }// 

        {
          // 修改这个为误差最小的25个点
          // 假设 err 是一个向量，包含每个点的误差，索引是点的编号。
          std::vector<std::pair<double, uint>> error_with_index;
          // 将误差值与对应的索引一起存储
          for (uint j = 0; j < err.rows(); ++j)
          {
            error_with_index.emplace_back(err(j), j);
          }
          // 按照误差值升序排序
          std::sort(error_with_index.begin(), error_with_index.end(), [](const std::pair<double, uint>& a, const std::pair<double, uint>& b) {
            return a.first < b.first;
          });

          for(auto& err: error_with_index)
          {
            radar_inlier << "err = " << err.first << std::endl;
          }

          // 选择误差最小的前 25 个点
          for (uint j = 0; j < 50 && j < error_with_index.size(); ++j)  // 45
          {
            if(error_with_index[j].first < config_.inlier_thresh)
              inlier_idx.emplace_back(error_with_index[j].second);  // 将前 45 个点的索引添加到 inlier_idx 中  
            else
              break;
          }
          // 如果需要，也可以将剩下的点视为外点
          for (uint j = 25; j < error_with_index.size(); ++j)
          {
            outlier_idx.emplace_back(error_with_index[j].second);  // 将剩余的点视为外点
          }
        }

        radar_inlier << "inlier_idx.size = " << inlier_idx.size() << " outlier_idx.size = " << outlier_idx.size() << std::endl;

        // radar_inlier << "inlier_idx.size = " << inlier_idx.size() << std::endl;
        // if too small number of inlier detected, the error is too high, so regard outlier as inlier
        // HAO TODO: Discard this

        // radar_inlier << "rate = " << float(outlier_idx.size())/(inlier_idx.size()+outlier_idx.size()) << std::endl;
        // if ( float(outlier_idx.size())/(inlier_idx.size()+outlier_idx.size()) > 0.05 )
        // {
        //   inlier_idx.insert(inlier_idx.end(), outlier_idx.begin(), outlier_idx.end());
        //   outlier_idx.clear();
        //   // outlier_idx.swap(std::vector<uint>());
        // }

        // radar_inlier << "total pcl = " << (inlier_idx.size()+outlier_idx.size()) << std::endl;

        // ROS_INFO("Inlier number: %ld, Outlier number: %ld, outlier Ratio: %f", 
        //           inlier_idx.size(), outlier_idx.size(), float(outlier_idx.size())/(inlier_idx.size()+outlier_idx.size()));

        if (inlier_idx.size() > inlier_idx_best.size())
        {
          inlier_idx_best = inlier_idx;
        }
        if (outlier_idx.size() > outlier_idx_best.size())
        {
          outlier_idx_best = outlier_idx;
        }

        radar_inlier << "best inlier = " << inlier_idx_best.size() << "  best outlier = " << outlier_idx_best.size() << std::endl;
      }
    }
  }
  else{ROS_INFO("Warning: radar_data.rows() < config_.N_ransac_points");}
  // radar_inlier.close();
  if (!inlier_idx_best.empty())
  {
    Matrix radar_data_inlier(inlier_idx_best.size(), 4);
    for (uint i = 0; i < inlier_idx_best.size(); ++i) radar_data_inlier.row(i) = radar_data.row(inlier_idx_best.at(i));
    radar_inlier << "Failure at solve3DFullRansac() 2: size = " << inlier_idx_best.size() << std::endl;
    bool rtn = solve3DFull(radar_data_inlier, v_r, sigma_v_r, true);
    radar_inlier << "check 4 : radar_data = " << radar_data_inlier << std::endl;
    radar_inlier << "final vel  = " << v_r << std::endl;
    if (rtn == false) {ROS_INFO("Failure at solve3DFullRansac() 2");radar_inlier << "Failure at solve3DFullRansac() 2" << std::endl;}
    radar_inlier.close();

    return rtn;
  }
  ROS_INFO("Failure at solve3DFullRansac() 3");
  radar_inlier << "Failure at solve3DFullRansac() 3" << std::endl;
  radar_inlier.close();
  return false;
}
*/

bool RadarEgoVelocityEstimator::solve3DFullRansac(const Matrix& radar_data,
                                                  Vector3& v_r,
                                                  Vector3& sigma_v_r,
                                                  std::vector<uint>& inlier_idx_best,
                                                  std::vector<uint>& outlier_idx_best)
{
  Matrix H_all(radar_data.rows(), 3);
  H_all.col(0)       = radar_data.col(0);
  H_all.col(1)       = radar_data.col(1);
  H_all.col(2)       = radar_data.col(2);
  const Vector y_all = radar_data.col(3);

  std::vector<uint> idx(radar_data.rows());
  for (uint k = 0; k < radar_data.rows(); ++k) idx[k] = k;

  std::random_device rd;
  std::mt19937 g(rd());

  std::fstream radar_inlier("/home/hao/Desktop/radar-event-new/src/TwistEstimator/output/radar_inliers.debug", std::ios::out | std::ios::app);

  double inlier_error ;
  double best_inlier_error = 100;
  int ransac_iter_own = 12;
  if (radar_data.rows() >= config_.N_ransac_points)
  {
    for (uint k = 0; k < ransac_iter_own; ++k)
    {
      inlier_error  = 0;
      std::shuffle(idx.begin(), idx.end(), g);
      Matrix radar_data_iter;
      radar_data_iter.resize(config_.N_ransac_points, 4);

      for (uint i = 0; i < config_.N_ransac_points; ++i) radar_data_iter.row(i) = radar_data.row(idx.at(i));
      bool rtn = solve3DFull(radar_data_iter, v_r, sigma_v_r, false);
      if (rtn == false) ROS_INFO("Failure at solve3DFullRansac() 1");
      if (rtn)
      {
        const Vector err = (y_all - H_all * v_r).array().abs();
        radar_inlier << "err = " << err.transpose() << std::endl;
        std::vector<uint> inlier_idx;
        std::vector<uint> outlier_idx;
        for (uint j = 0; j < err.rows(); ++j)
        {
          // ROS_INFO("Error: %f",err(j));
          if (err(j) < config_.inlier_thresh)
          {
            // find inlier points
            inlier_idx.emplace_back(j);
            inlier_error += err(j);
          }
          else
            outlier_idx.emplace_back(j);
        }
        // if too small number of inlier detected, the error is too high, so regard outlier as inlier
        // if ( float(outlier_idx.size())/(inlier_idx.size()+outlier_idx.size()) > 0.05 )
        // {
        //   inlier_idx.insert(inlier_idx.end(), outlier_idx.begin(), outlier_idx.end());
        //   outlier_idx.clear();
        //   // outlier_idx.swap(std::vector<uint>());
        // }

        // ROS_INFO("Inlier number: %ld, Outlier number: %ld, outlier Ratio: %f", 
        //           inlier_idx.size(), outlier_idx.size(), float(outlier_idx.size())/(inlier_idx.size()+outlier_idx.size()));

        radar_inlier << "Radar Inlier Update K = " << k << std::endl;
        if (inlier_idx.size() > inlier_idx_best.size() || (inlier_idx.size() == inlier_idx_best.size() && inlier_error < best_inlier_error))
        {
          radar_inlier << "Radar Inlier Update iter = " << k
                       << " inlier_idx.size = " << inlier_idx.size()
                       << " origin inlier_idx.size = " << inlier_idx_best.size() 
                       << " inlier_error = " << inlier_error 
                       << " best_inlier_error = " << best_inlier_error 
                       << std::endl;
          inlier_idx_best = inlier_idx;
          best_inlier_error = inlier_error;
        }
        if (outlier_idx.size() > outlier_idx_best.size())
        {
          outlier_idx_best = outlier_idx;
        }
      }
    }
  }
  else{ROS_INFO("Warning: radar_data.rows() < config_.N_ransac_points");}

  if (!inlier_idx_best.empty())
  {
    radar_inlier << "inlier_idx_best.size = " << inlier_idx_best.size() << std::endl;
    Matrix radar_data_inlier(inlier_idx_best.size(), 4);
    for (uint i = 0; i < inlier_idx_best.size(); ++i) radar_data_inlier.row(i) = radar_data.row(inlier_idx_best.at(i));
    
    bool rtn = solve3DFull(radar_data_inlier, v_r, sigma_v_r, true);
    if (rtn == false) ROS_INFO("Failure at solve3DFullRansac() 2");
    radar_inlier.close();
    return rtn;
  }
  ROS_INFO("Failure at solve3DFullRansac() 3");
  radar_inlier.close();
  return false;
}

bool RadarEgoVelocityEstimator::solve3DFull(const Matrix& radar_data,
                                            Vector3& v_r,
                                            Vector3& sigma_v_r,
                                            bool estimate_sigma)
{
  std::fstream debug_opti_file("/home/hao/Desktop/radar-event-new/src/TwistEstimator/output/debug_radar_opti.tum", std::ios::out | std::ios::app);

  Matrix H(radar_data.rows(), 3);
  H.col(0)         = radar_data.col(0);
  H.col(1)         = radar_data.col(1);
  H.col(2)         = radar_data.col(2);
  // const 
  Matrix HTH = H.transpose() * H;

  // debug_opti_file << "H = " << H << std::endl;
  // debug_opti_file << "HTH = " << HTH << std::endl;

  // const 
  Vector y = radar_data.col(3);

  // debug_opti_file << "y = " << y << std::endl;

  Eigen::JacobiSVD<Matrix> svd(HTH);
  Real cond = svd.singularValues()(0) / svd.singularValues()(svd.singularValues().size() - 1);
  // std::cout << "std::fabs(cond) = " << std::fabs(cond) << " max cond is " << config_.max_r_cond << std::endl;
  // debug_opti_file << "cond = " << std::fabs(cond) << std::endl;

  // cond > 1000, error occurs
  if (1)//std::fabs(cond) < 1.0e3
  {

    // discard this 2025-4-1
    if (config_.use_cholesky_instead_of_bdcsvd)
    {
      v_r = (HTH).ldlt().solve(H.transpose() * y);
    }
    else
      v_r = H.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(y);

    // 2-16 修改
    // v_r = HTH.inverse() * (H.transpose() * y);
    v_r = H.colPivHouseholderQr().solve(y);

    double delta = 0.035; // 0.1;  // 控制阈值
    Eigen::VectorXd residuals = H * v_r - y;
    debug_opti_file << "radar size = " << y.size() << std::endl;
    debug_opti_file << "residuals = " << residuals.size() << std::endl;

    debug_opti_file << "residuals I = " << residuals.transpose() << std::endl;  
    for (int i = 0; i < residuals.size(); ++i) {
        double r = residuals(i);
        if (std::abs(r) > delta) {
            residuals(i) = delta * std::copysign(1.0, r);  // 线性部分
        }
    }
    debug_opti_file << "residuals with huber lost = " << residuals.transpose() << std::endl;  
    v_r = H.colPivHouseholderQr().solve(y + residuals);
    residuals = H * v_r - y;
    debug_opti_file << "residuals II = " << residuals.transpose() << std::endl;  

    debug_opti_file << "H = " << H << ", y = " << y << std::endl;  
    debug_opti_file << "v_r = " << v_r << std::endl;
    // debug_opti_file << "estimate_sigma = " << ((estimate_sigma)? "True": "False") << std::endl;
    if (estimate_sigma)
    {

      debug_opti_file << "A1" << std::endl;

      // std::cout << "H dimensions: " << H.rows() << "x" << H.cols() << std::endl;
      // std::cout << "v_r dimensions: " << v_r.rows() << "x" << v_r.cols() << std::endl;
      // std::cout << "y dimensions: " << y.rows() << "x" << y.cols() << std::endl;

      // debug_opti_file << "e = H * v_r - y " << H * v_r - y << std::endl;
      // const 
      Vector e = H * v_r - y;
      debug_opti_file << "before C" << std::endl;
      // debug_opti_file << "(HTH).inverse() = " << (HTH).inverse() << std::endl;
      // const 
      Matrix C = (e.transpose() * e).x() * (HTH).inverse() / (H.rows() - 3);
      debug_opti_file << "after C" << std::endl;
      sigma_v_r      = Vector3(C(0, 0), C(1, 1), C(2, 2));
      sigma_v_r      = sigma_v_r.array();
  
      // debug_opti_file << "linear sigma = [" << sigma_v_r.x() << "," << sigma_v_r.y() << "," << sigma_v_r.z() << "]" << std::endl;
      // debug_opti_file.close();
      debug_opti_file << "B1" << std::endl;
      if (sigma_v_r.x() >= 0.0 && sigma_v_r.y() >= 0.0 && sigma_v_r.z() >= 0.)
      {
        sigma_v_r = sigma_v_r.array().sqrt();
        sigma_v_r += Vector3(config_.sigma_offset_radar_x, config_.sigma_offset_radar_y, config_.sigma_offset_radar_z);

      //   debug_opti_file << "config sigma = [" << config_.sigma_offset_radar_x << ", " 
      //             << config_.sigma_offset_radar_y << ", " 
      //             << config_.sigma_offset_radar_z << "]" << std::endl;
        debug_opti_file << "check sigma = [" << sigma_v_r.x() << ", " 
                  << sigma_v_r.y() << ", " 
                  << sigma_v_r.z() << "]" << std::endl;

        // if (sigma_v_r.x() < config_.max_sigma_x && sigma_v_r.y() < config_.max_sigma_y &&
        //     sigma_v_r.z() < config_.max_sigma_z)
        // {
        //   return false;
        // }

        // if (sigma_v_r.x() < config_.max_sigma_x && sigma_v_r.y() < config_.max_sigma_y &&
        //     sigma_v_r.z() < config_.max_sigma_z)
        // {
          
      //     return true;
      //   // }
      }
      debug_opti_file << "C1" << std::endl;
      debug_opti_file.close();
      return true;
    }
    else
    {
      debug_opti_file.close();
      return true;
    }
  }
  //ROS_INFO("cond too large, cond = %f", cond);
  debug_opti_file.close();
  return false;
  // return true;//return false;
}
