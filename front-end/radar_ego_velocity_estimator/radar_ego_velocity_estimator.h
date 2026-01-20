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

#pragma once
 
#include <sensor_msgs/PointCloud2.h>

#include <yaml-cpp/yaml.h>

#include "radar_ego_velocity_estimator/data_types.h"
#include "radar_ego_velocity_estimator/ros_helper.h"

#include <fstream>

// #include <RadarEgoVelocityEstimatorConfig.h>

namespace rio
{

struct RadarEgoVelocityEstimatorConfig
{
 float min_dist = 1;
 float max_dist = 200; //300;    // RS548 300 m
 float min_db = 10; //10;
 float elevation_thresh_deg = 22.5;
 float azimuth_thresh_deg = 56.5;
 float doppler_velocity_correction_factor = 1;
 
 float thresh_zero_velocity = 0.05; //Below this is recognized as inlier (m/s)
 float allowed_outlier_percentage = 0.30;
 float sigma_zero_velocity_x = 1.0e-03; // default Standard Deviation
 float sigma_zero_velocity_y = 3.2e-03;
 float sigma_zero_velocity_z = 1.0e-02;
 
 float sigma_offset_radar_x = 0;
 float sigma_offset_radar_y = 0;
 float sigma_offset_radar_z = 0;

 float max_sigma_x = 0.2;
 float max_sigma_y = 0.2;
 float max_sigma_z = 0.2;
 float max_r_cond;
 bool use_cholesky_instead_of_bdcsvd = true;

 bool use_ransac = true;
 float outlier_prob = 0.05; // Outlier Probability, to calculate ransac_iter_
 float success_prob = 0.995;
 float N_ransac_points = 5;
 float inlier_thresh = 0.5; // err(j) threshold, 0.1 too small, 1.0 too large
//  std::string radar_type = "ti_mmwave";

 void LOAD(const std::string& file_path) {
    std::cout << "load radar parameters at: " << file_path << std::endl;

    YAML::Node config = YAML::LoadFile(file_path);

    assert(config["min_dist"] && "Missing min_dist"); 
    min_dist = config["min_dist"].as<float>();

    assert(config["max_dist"] && "Missing max_dist");
    max_dist = config["max_dist"].as<float>();

    assert(config["min_db"] && "Missing min_db");
    min_db = config["min_db"].as<float>();

    assert(config["elevation_thresh_deg"] && "Missing elevation_thresh_deg");
    elevation_thresh_deg = config["elevation_thresh_deg"].as<float>();

    assert(config["azimuth_thresh_deg"] && "Missing azimuth_thresh_deg");
    azimuth_thresh_deg = config["azimuth_thresh_deg"].as<float>();

    assert(config["doppler_velocity_correction_factor"] && "Missing doppler_velocity_correction_factor");
    doppler_velocity_correction_factor = config["doppler_velocity_correction_factor"].as<float>();

    assert(config["thresh_zero_velocity"] && "Missing thresh_zero_velocity");
    thresh_zero_velocity = config["thresh_zero_velocity"].as<float>();

    assert(config["allowed_outlier_percentage"] && "Missing allowed_outlier_percentage");
    allowed_outlier_percentage = config["allowed_outlier_percentage"].as<float>();

    assert(config["sigma_zero_velocity_x"] && "Missing sigma_zero_velocity_x");
    sigma_zero_velocity_x = config["sigma_zero_velocity_x"].as<float>();

    assert(config["sigma_zero_velocity_y"] && "Missing sigma_zero_velocity_y");
    sigma_zero_velocity_y = config["sigma_zero_velocity_y"].as<float>();

    assert(config["sigma_zero_velocity_z"] && "Missing sigma_zero_velocity_z");
    sigma_zero_velocity_z = config["sigma_zero_velocity_z"].as<float>();

    assert(config["sigma_offset_radar_x"] && "Missing sigma_offset_radar_x");
    sigma_offset_radar_x = config["sigma_offset_radar_x"].as<float>();

    assert(config["sigma_offset_radar_y"] && "Missing sigma_offset_radar_y");
    sigma_offset_radar_y = config["sigma_offset_radar_y"].as<float>();

    assert(config["sigma_offset_radar_z"] && "Missing sigma_offset_radar_z");
    sigma_offset_radar_z = config["sigma_offset_radar_z"].as<float>();

    assert(config["max_sigma_x"] && "Missing max_sigma_x");
    max_sigma_x = config["max_sigma_x"].as<float>();

    assert(config["max_sigma_y"] && "Missing max_sigma_y");
    max_sigma_y = config["max_sigma_y"].as<float>();

    assert(config["max_sigma_z"] && "Missing max_sigma_z");
    max_sigma_z = config["max_sigma_z"].as<float>();

    assert(config["max_r_cond"] && "Missing max_r_cond");
    max_r_cond = config["max_r_cond"].as<float>();

    assert(config["use_cholesky_instead_of_bdcsvd"] && "Missing use_cholesky_instead_of_bdcsvd");
    use_cholesky_instead_of_bdcsvd = config["use_cholesky_instead_of_bdcsvd"].as<bool>();

    assert(config["use_ransac"] && "Missing use_ransac");
    use_ransac = config["use_ransac"].as<bool>();

    assert(config["outlier_prob"] && "Missing outlier_prob");
    outlier_prob = config["outlier_prob"].as<float>();

    assert(config["success_prob"] && "Missing success_prob");
    success_prob = config["success_prob"].as<float>();

    assert(config["N_ransac_points"] && "Missing N_ransac_points");
    N_ransac_points = config["N_ransac_points"].as<float>();

    assert(config["inlier_thresh"] && "Missing inlier_thresh");
    inlier_thresh = config["inlier_thresh"].as<float>();

    // assert(config["radar_type"] && "Missing radar_type");
    // radar_type = config["radar_type"].as<std::string>();
  }
};

struct RadarEgoVelocityEstimatorIndices
{ 
  
  uint x_r          = 0;
  uint y_r          = 1;
  uint z_r          = 2;
  uint snr_db       = 3;
  uint doppler      = 4;
  uint range        = 5;
  uint azimuth      = 6;
  uint elevation    = 7;
  uint normalized_x = 8;
  uint normalized_y = 9;
  uint normalized_z = 10;
};

class RadarEgoVelocityEstimator
{
public:
  // EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /**
   * @brief RadarEgoVelocityEstimator constructor
   */
  RadarEgoVelocityEstimator() {

    }

  void LOAD(std::string yaml_file)
  {
    config_.LOAD(yaml_file);
    setRansacIter();
  }


  /**
   * @brief Reconfigure callback
   * @param config  has to contain RadarEgoVelocityEstimatorConfig
   * @return
   */
  template <class ConfigContainingRadarEgoVelocityEstimatorConfig>
  bool configure(ConfigContainingRadarEgoVelocityEstimatorConfig& config);

  /**
   * @brief Estimates the radar ego velocity based on a single radar scan
   * @param[in] radar_scan_msg       radar scan
   * @param[out] v_r                 estimated radar ego velocity
   * @param[out] sigma_v_r           estimated sigmas of ego velocity
   * @param[out] inlier_radar_scan   inlier point cloud
   * @returns true if estimation successful
   */
  bool estimate(const sensor_msgs::PointCloud2& radar_scan_msg, Vector3& v_r, Vector3& sigma_v_r);
  bool estimate(const sensor_msgs::PointCloud2& radar_scan_msg,
                Vector3& v_r,
                Vector3& sigma_v_r,
                sensor_msgs::PointCloud2& inlier_radar_msg,
                sensor_msgs::PointCloud2& outlier_radar_msg);

private:
  /**
   * @brief Implementation of the ransac based estimation
   * @param[in] radar_data          matrix of parsed radar scan --> see RadarEgoVelocityEstimatorIndices
   * @param[out] v_r                estimated radar ego velocity
   * @param[out] sigma_v_r          estimated sigmas of ego velocity
   * @param[out] inlier_idx_best    idices of inlier
   * @returns true if estimation successful
   */
  bool
  solve3DFullRansac(const Matrix& radar_data, Vector3& v_r, Vector3& sigma_v_r, std::vector<uint>& inlier_idx_best, std::vector<uint>& outlier_idx_best);

  /**
   * @brief Estimates the radar ego velocity using all mesurements provided in radar_data
   * @param[in] radar_data          matrix of parsed radar scan --> see RadarEgoVelocityEstimatorIndices
   * @param[out] v_r                estimated radar ego velocity
   * @param[out] sigma_v_r          estimated sigmas of ego velocity
   * @param estimate_sigma          if true sigma will be estimated as well
   * @returns true if estimation successful
   */
  bool solve3DFull(const Matrix& radar_data, Vector3& v_r, Vector3& sigma_v_r, bool estimate_sigma = true);

  /**
   * @brief Helper function which estiamtes the number of RANSAC iterations
   */
  void setRansacIter()
  {
    ransac_iter_ = uint((std::log(1.0 - config_.success_prob)) /
                        std::log(1.0 - std::pow(1.0 - config_.outlier_prob, config_.N_ransac_points)));
    ROS_INFO_STREAM(kPrefix << "Number of Ransac iterations: " << ransac_iter_);
    std::fstream radar_file("/home/hao/Desktop/twist_ws/src/TwistEstimator/output/doppler.debug", std::ios::out | std::ios::app);
    radar_file << "ransac_iter_ = " << ransac_iter_ << std::endl;
    radar_file.flush();
    radar_file.close();

  }

  const std::string kPrefix = "[RadarEgoVelocityEstimator]: ";
  const RadarEgoVelocityEstimatorIndices idx_;

  RadarEgoVelocityEstimatorConfig config_;
  uint ransac_iter_ = 0;
};

template <class ConfigContainingRadarEgoVelocityEstimatorConfig>
bool RadarEgoVelocityEstimator::configure(ConfigContainingRadarEgoVelocityEstimatorConfig& config)
{
  config_.min_dist                           = config.min_dist;
  config_.max_dist                           = config.max_dist;
  config_.min_db                             = config.min_db;
  config_.elevation_thresh_deg               = config.elevation_thresh_deg;
  config_.azimuth_thresh_deg                 = config.azimuth_thresh_deg;
  config_.doppler_velocity_correction_factor = config.doppler_velocity_correction_factor;

  config_.thresh_zero_velocity       = config.thresh_zero_velocity;
  config_.allowed_outlier_percentage = config.allowed_outlier_percentage;
  config_.sigma_zero_velocity_x      = config.sigma_zero_velocity_x;
  config_.sigma_zero_velocity_y      = config.sigma_zero_velocity_y;
  config_.sigma_zero_velocity_z      = config.sigma_zero_velocity_z;

  config_.sigma_offset_radar_x = config.sigma_offset_radar_x;
  config_.sigma_offset_radar_y = config.sigma_offset_radar_y;
  config_.sigma_offset_radar_z = config.sigma_offset_radar_z;

  config_.max_sigma_x                    = config.max_sigma_x;
  config_.max_sigma_y                    = config.max_sigma_y;
  config_.max_sigma_z                    = config.max_sigma_z;
  config_.max_r_cond                     = config.max_r_cond;
  config_.use_cholesky_instead_of_bdcsvd = config.use_cholesky_instead_of_bdcsvd;

  config_.use_ransac      = config.use_ransac;
  config_.outlier_prob    = config.outlier_prob;
  config_.success_prob    = config.success_prob;
  config_.N_ransac_points = config.N_ransac_points;
  config_.inlier_thresh   = config.inlier_thresh;

  setRansacIter();

  ROS_INFO_STREAM(kPrefix << "Number of Ransac iterations: " << ransac_iter_);
  return true;
}
}  // namespace rio
