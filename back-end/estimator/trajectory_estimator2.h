/*
 * Continuous-Time Fixed-Lag Smoothing for LiDAR-Inertial-Camera SLAM
 * Copyright (C) 2022 Jiajun Lv
 * Copyright (C) 2022 Xiaolei Lang
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

#pragma once

#include <ceres/ceres.h>
#include <ceres/covariance.h>
#include "factor/ceres_local_param.h"
// #include <inertial/imu_state_estimator.h>
// #include <lidar_odometry/lidar_feature.h>
#include "utils/parameter_struct.h"

#include "estimator/trajectory_estimator_options.h"
// #include "visual_odometry/integration_base.h"

#include "spline/trajectory.h"
#include "spline/rd_spline.h"

#include "spline/twist_trajectory.h"
#include "spline/rd6_spline.h"

#include "factor/marginalization_factor.h"
// #include "../factor/marginalization_factor.h"

// #include <flex_estimator/factor/analytic_diff/marginalization_factor.h>
// #include <flex_estimator/factor/analytic_diff/trajectory_value_factor.h>

// IMUGlobalVelocityFactor
// #include <flex_estimator/factor/auto_diff/trajectory_value_factor.h>

#include "factor/TwistFactor.h"

struct LMCallback : public ceres::IterationCallback {
    ceres::CallbackReturnType operator()(const ceres::IterationSummary& summary) override {
        LOG(ERROR) << "Debug Iter: " << summary.iteration
                  << ", Cost: " << summary.cost
                  << ", ΔCost: " << summary.cost_change
                  << ", Max|∇|: " << summary.gradient_max_norm
                  << ", Step norm: " << summary.step_norm
                  << ", Trust region: " << summary.trust_region_radius
                  << ", Success: " << summary.step_is_successful
                  << ", Lin. iters: " << summary.linear_solver_iterations
                  << std::endl;
        return ceres::SOLVER_CONTINUE;
    }
};

struct ResidualSummary2 {
  std::map<ResidualType, std::vector<double>> err_type_sum;
  std::map<ResidualType, int> err_type_number;

  std::map<ResidualType, time_span_t> err_type_duration;
  std::map<int, std::pair<size_t, size_t>> opt_knot;

  std::string descri_info;

  ResidualSummary2(std::string descri = "") : descri_info(descri) {
    for (auto typ = RType_Pose; typ <= RType_Prior;
         typ = ResidualType(typ + 1)) {
      err_type_sum[typ].push_back(0);
      err_type_number[typ] = 0;
      err_type_duration[typ] = std::make_pair(0, 0);
    }
    opt_knot[0] = std::make_pair(1, 0);  // pos knot
    opt_knot[1] = std::make_pair(1, 0);  // rot knot
  }

  void AddResidualInfo(ResidualType r_type,
                       const ceres::CostFunction* cost_function,
                       const std::vector<double*>& param_vec);

  void AddResidualTimestamp(ResidualType r_type, int64_t time_ns) {
    auto& t_span = err_type_duration[r_type];
    if (t_span.first == 0) {
      t_span.first = time_ns;
      t_span.second = time_ns;
    } else {
      t_span.first = t_span.first < time_ns ? t_span.first : time_ns;
      t_span.second = t_span.second > time_ns ? t_span.second : time_ns;
    }
  }

  void AddKnotIdx(size_t knot, bool is_pos_knot) {
    int k = is_pos_knot ? 0 : 1;
    if (opt_knot[k].first > opt_knot[k].second) {
      opt_knot[k].first = knot;
      opt_knot[k].second = knot;
    } else {
      opt_knot[k].first = opt_knot[k].first < knot ? opt_knot[k].first : knot;
      opt_knot[k].second =
          opt_knot[k].second > knot ? opt_knot[k].second : knot;
    }
  }

  void PrintSummary(int64_t t0_ns, int64_t dt_ns,
                    int fixed_ctrl_idx = -1) const;

  std::string GetTimeString(int64_t knot_min, int64_t knot_max, int64_t t0_ns,
                            int64_t dt_ns) const;

  std::string GetCtrlString(int64_t t_min_ns, int64_t t_max_ns, int64_t t0_ns,
                            int64_t dt_ns) const;
};

class TrajectoryEstimator2 {
  static ceres::Problem::Options DefaultProblemOptions() {
    ceres::Problem::Options options;
    options.loss_function_ownership = ceres::TAKE_OWNERSHIP;
    options.local_parameterization_ownership = ceres::DO_NOT_TAKE_OWNERSHIP;
    return options;
  }

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  typedef std::shared_ptr<TrajectoryEstimator2> Ptr;

  TrajectoryEstimator2(Twist_Trajectory::Ptr trajectory,
                      TrajectoryEstimatorOptions& option,
                      std::string descri = "");
                      
  ~TrajectoryEstimator2() {
    // Ceres will call delete on local_parameterization upon completion.

    if (analytic_local_parameterization_)
      delete analytic_local_parameterization_;

    if (auto_diff_local_parameterization_)
      delete auto_diff_local_parameterization_;

    if (homo_vec_local_parameterization_)
      delete homo_vec_local_parameterization_;

    // delete marginalization_info_;
  }

  /// fixed the ctrl points before that scan
  void SetKeyScanConstant(double max_time);

  /// check if the measurement within the opt window
  bool MeasuredTimeToNs(const SensorType& sensor_type, const double& timestamp,
                        int64_t& time_ns) const;

  /// directly fix the specify ctrl point
  void SetFixedIndex(int idx) { fixed_control_point_index_ = idx; }

  int GetFixedControlIndex() const { return fixed_control_point_index_; }

  void SetTimeoffsetState();

  // [factor] start pose of the trajectory
  void AddStartTimePose(const PoseData& pose);

  // [factor] prior factor
  void AddMarginalizationFactor(
      MarginalizationInfo::Ptr last_marginalization_info,
      std::vector<double*>& last_marginalization_parameter_blocks);

  ceres::Solver::Summary Solve(int max_iterations = 50, bool progress = false,
                               int num_threads = -1);

  void PrepareMarginalizationInfo(ResidualType r_type,
                                  ceres::CostFunction* cost_function,
                                  ceres::LossFunction* loss_function,
                                  std::vector<double*>& parameter_blocks,
                                  std::vector<int>& drop_set);

  void SaveMarginalizationInfo(MarginalizationInfo::Ptr& marg_info_out,
                               std::vector<double*>& marg_param_blocks_out);

  const ResidualSummary2& GetResidualSummary() const {
    return residual_summary_;
  }

  void AddPoseMeasurementAnalytic(PoseData pose_temp, 
            Eigen::Matrix<double, 6, 1> info_vec){};

 private:
  // void AddControlPoints(const SplineMeta<SplineOrder>& spline_meta,
  //                       std::vector<double*>& vec, bool addPosKnot = false);

  void AddControlPoints(
      const SplineMeta<SplineOrder>& spline_meta, std::vector<double*>& vec,
      bool addPosKnot = false, bool use_order_opti = false);

  void PrepareMarginalizationInfo(ResidualType r_type,
                                  const SplineMeta<SplineOrder>& spline_meta,
                                  ceres::CostFunction* cost_function,
                                  ceres::LossFunction* loss_function,
                                  std::vector<double*>& parameter_blocks,
                                  std::vector<int>& drop_set_wo_ctrl_point);

  void PrepareMarginalizationInfo(
      ResidualType r_type, const SplineMeta<SplineOrder>& linear_spline_meta,
      const SplineMeta<SplineOrder>& angular_spline_meta,
      ceres::CostFunction* cost_function, ceres::LossFunction* loss_function,
      std::vector<double*>& parameter_blocks,
      std::vector<int>& drop_set_wo_ctrl_point);

  bool IsParamUpdated(const double* values) const;

public:
  // --------- Factor ---------

  // void AddDopplerMeasurementAnalytic(
  //   double timestamp, const std::vector<Eigen::Vector3d>& point_vec, double* gyro_bias,
  //   const std::vector<double>& doppler_vec, const Eigen::Matrix3d R_e_r,
  //   double weight, bool marg_this_factor);

  void AddDopplerMeasurementAnalytic(
    double timestamp, const Eigen::Vector3d& point, double* gyro_bias,
    const double& doppler, const Eigen::Matrix3d R_e_r,
    double weight, bool marg_this_factor);

  // void AddEventFlowMeasurementAnalytic(
  //   double timestamp,
  //   cv::Point2d pixel_p, event_flow_velocity flow, 
  //   const Eigen::Vector3d doppler_velocity,
  //   Eigen::Quaterniond & q_e_r, Eigen::Vector3d& t_e_r,
  //   Eigen::Vector3d angular_bias, double time_offset,
  //   double weight, bool marg_this_factor);

  void AddEventFlowMeasurementAnalytic(
    double timestamp,
    // cv::Point2d pixel_p, event_flow_velocity flow, 
    Eigen::Vector3d pixel_p, event_flow_velocity flow, 
    const Eigen::Vector3d doppler_velocity,
    Eigen::Quaterniond & q_e_r, Eigen::Vector3d& t_e_r, double* linear_bias,
    double* angular_bias, double* time_offset,
    double weight, bool marg_this_factor);

  void AddBiasFactor(
    double* bias_gyr_i, double* bias_gyr_j, double* bias_acc_i,
    double* bias_acc_j, double dt, const Eigen::Matrix<double, 6, 1>& info_vec,
    bool marg_this_factor = false, bool marg_all_bias = false);

  void AddTwistBiasAnalytic(
    double* bias_w_i, double* bias_w_j, double* bias_v_i,
    double* bias_v_j, double dt, const Eigen::Matrix<double, 6, 1>& info_vec,
    bool marg_this_factor, bool marg_all_bias);

  // void AddBodyLocalVelocityMeasurementAnalytic(
  //     double timestamp, double* linear_bias,
  //     const Eigen::Vector3d local_vel,
  //     // double* time_offset,
  //     double weight, bool marg_this_factor);

  // void AddBodyLocalVelocityMeasurementAnalytic(
  //     double timestamp, double* linear_bias,
  //     const Eigen::Vector3d local_vel,
  //     // double* time_offset,
  //     double weight, double w_weight,
  //     bool marg_this_factor);

void AddBodyLocalVelocityMeasurementAnalytic(
    double timestamp, double* linear_bias,
    const Eigen::Vector3d local_vel,
    double weight, double w_weight, double R_weight,
    bool marg_this_factor);

  // void AddBodyLocalAngularVelocityMeasurementAnalytic(
  //     double timestamp, double* angular_bias,
  //     const Eigen::Vector3d local_angular_vel,
  //     // double* time_offset,
  //     double weight, bool marg_this_factor);

  void AddBodyLocalAngularVelocityMeasurementAnalytic(
      double timestamp, double* angular_bias, 
      const Eigen::Vector3d local_angular_vel,
      double weight, double w_weight_,
      bool marg_this_factor);

void AddEventFlowMeasurementAnalytic2(
    double timestamp,
    Eigen::Vector3d pixel_p, event_flow_velocity flow, 
    const Eigen::Vector3d doppler_velocity,
    Eigen::Quaterniond & q_e_r, Eigen::Vector3d& t_e_r, double* linear_bias,
    double* angular_bias, double* time_offset,
    double weight, double w_weight, bool marg_this_factor);

// void AddEventFlowMeasurementAnalytic3(
//     double timestamp,
//     Eigen::Vector3d pixel_p, event_flow_velocity flow, 
//     const Eigen::Vector3d doppler_velocity,
//     Eigen::Quaterniond & q_e_r, Eigen::Vector3d& t_e_r, double* linear_bias,
//     double* angular_bias, double* time_offset,
//     double weight, double w_weight, bool marg_this_factor);

void AddDopplerMeasurementAnalytic2(
  double timestamp, const Eigen::Vector3d& point, double* linear_bias,
  const double& doppler, const Eigen::Matrix3d R_e_r, bool use_order_opti,
  double weight, double linear_w_weight, bool marg_this_factor);

// void AddEventFlowMeasurementAnalytic3(
//     double timestamp,
//     Eigen::Vector3d pixel_p, event_flow_velocity flow, 
//     const Eigen::Vector3d doppler_velocity,
//     Eigen::Quaterniond & q_e_r, Eigen::Vector3d& t_e_r, double* linear_bias,
//     double* angular_bias, double* time_offset,
//     double weight, double w_weight, bool marg_this_factor);

void AddEventFlowMeasurementAnalytic3(
    double timestamp,
    Eigen::Vector3d pixel_p, event_flow_velocity flow, 
    const Eigen::Vector3d doppler_velocity,
    Eigen::Quaterniond & q_e_r, Eigen::Vector3d& t_e_r, double* linear_bias,
    double* angular_bias, double* time_offset,
    std::shared_ptr<FEJ_STATE> global_fej_state_, bool use_fej, bool use_order_opti,
    double weight, double w_weight, bool marg_this_factor);

  void AddCallback2(
      const std::vector<std::string>& descriptions,
      const std::vector<size_t>& block_size, 
      std::vector<double*>& param_block);

  void SetParameterBlockOrdering()
  {
    // ordering->AddElementToGroup(pose_param_block, 0);
    // ordering->AddElementToGroup(velocity_param_block, 1);
    // ordering->AddElementToGroup(bias_param_block, 2);
    // problem_->SetParameterBlockOrdering(ordering);
    ;
  }

//  public:
  TrajectoryEstimatorOptions options;

 private:
  Twist_Trajectory::Ptr trajectory_;

  std::shared_ptr<ceres::Problem> problem_;
  ceres::LocalParameterization* analytic_local_parameterization_;
  ceres::HomogeneousVectorParameterization* homo_vec_local_parameterization_;

  ceres::LocalParameterization* auto_diff_local_parameterization_;

  std::map<SensorType, double*> t_offset_ns_opt_params_;

  int fixed_control_point_index_;

  // Marginalization
  MarginalizationInfo::Ptr marginalization_info_;

  // for debug
  ResidualSummary2 residual_summary_;

  bool callback_needs_state_;
  std::vector<std::unique_ptr<ceres::IterationCallback>> callbacks_;

  // 分组优化
  std::shared_ptr<ceres::ParameterBlockOrdering> ordering;
};


