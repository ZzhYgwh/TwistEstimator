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

#include <ceres/ceres.h>
#include <ceres/covariance.h>
#include <ceres/dynamic_cost_function.h>

#include "trajectory_estimator2.h"
#include "utils/ceres_callbacks.h"

#include <iostream>
#include <memory>
#include <thread>
#include <variant>

void ResidualSummary2::AddResidualInfo(ResidualType r_type,
                                      const ceres::CostFunction* cost_function,
                                      const std::vector<double*>& param_vec) {
  int num_residuals = cost_function->num_residuals();
  Eigen::MatrixXd residuals;
  residuals.setZero(num_residuals, 1);
  cost_function->Evaluate(param_vec.data(), residuals.data(), nullptr);

  auto& error_sum = err_type_sum[r_type];
  // initial error as 0
  while ((int)error_sum.size() < num_residuals) {
    error_sum.push_back(0);
  }
  for (int i = 0; i < num_residuals; i++) {
    error_sum[i] += std::fabs(residuals(i, 0));
  }
  err_type_number[r_type]++;

  if (RType_PreIntegration == r_type) {
    auto&& log = COMPACT_GOOGLE_LOG_INFO;
    log.stream() << "imu_residuals :";
    for (int i = 0; i < num_residuals; i++) {
      log.stream() << std::fabs(residuals(i, 0)) << ", ";
    }
    log.stream() << "\n";
  }
}

void ResidualSummary2::PrintSummary(int64_t t0_ns, int64_t dt_ns,
                                   int fixed_ctrl_idx) const {
  if (err_type_sum.empty()) return;

  auto&& log = COMPACT_GOOGLE_LOG_INFO;
  log.stream() << "ResidualSummary2 :" << descri_info << "\n";
  // look through every residual info
  for (auto typ = RType_Pose; typ <= RType_Prior; typ = ResidualType(typ + 1)) {
    double num = err_type_number.at(typ);
    if (num > 0) {
      log.stream() << "\t- " << ResidualTypeStr[int(typ)] << ": ";
      log.stream() << "num = " << num << "; err_ave = ";

      auto& error_sum = err_type_sum.at(typ);
      if (RType_Prior == typ) {
        log.stream() << "(dim = " << error_sum.size() << ") ";
      }
      for (int i = 0; i < (int)error_sum.size(); ++i) {
        log.stream() << error_sum[i] / num << ", ";
        if ((i + 1) % 15 == 0) log.stream() << "\n\t\t\t\t";
      }
      log.stream() << std::endl;
    }
  }

  for (auto typ = RType_Pose; typ <= RType_Prior; typ = ResidualType(typ + 1)) {
    double num = err_type_number.at(typ);
    auto const& t_span_ns = err_type_duration.at(typ);
    if (num > 0 && t_span_ns.first > 0) {
      log.stream() << "\t- " << ResidualTypeStr[int(typ)] << ": "
                   << GetCtrlString(t_span_ns.first, t_span_ns.second, t0_ns,
                                    dt_ns)
                   << "\n";
    }
  }

  for (int k = 0; k < 2; k++) {
    if (k == 0)
      log.stream() << "\t- Pos ctrl       : ";
    else
      log.stream() << "\t- Rot ctrl       : ";

    auto const& knot_span = opt_knot.at(k);
    log.stream() << GetTimeString(knot_span.first, knot_span.second, t0_ns,
                                  dt_ns)
                 << "\n";
  }
  if (fixed_ctrl_idx > 0) {
    int64_t time_ns =
        (fixed_ctrl_idx - SplineOrder + 1) * dt_ns + t0_ns + dt_ns;

    log.stream() << "\t- fixed ctrl idx: " << fixed_ctrl_idx << "; opt time ["
                 << time_ns * NS_TO_S << ", ~]\n";
  }
}

std::string ResidualSummary2::GetTimeString(int64_t knot_min, int64_t knot_max,
                                           int64_t t0_ns, int64_t dt_ns) const {
  int64_t t_min_ns = (knot_min - SplineOrder + 1) * dt_ns + t0_ns;
  int64_t t_max_ns = (knot_max - SplineOrder + 1) * dt_ns + t0_ns + (dt_ns);

  std::stringstream ss;
  ss << "[" << t_min_ns * NS_TO_S << ", " << t_max_ns * NS_TO_S << "] in ["
     << knot_min << ", " << knot_max << "]";
  std::string segment_info = ss.str();
  return segment_info;
}

std::string ResidualSummary2::GetCtrlString(int64_t t_min_ns, int64_t t_max_ns,
                                           int64_t t0_ns, int64_t dt_ns) const {
  int64_t knot_min = (t_min_ns - t0_ns) / dt_ns;
  int64_t knot_max = (t_max_ns - t0_ns) / dt_ns + (SplineOrder - 1);
  std::stringstream ss;
  ss << "[" << t_min_ns * NS_TO_S << ", " << t_max_ns * NS_TO_S << "] in ["
     << knot_min << ", " << knot_max << "]";
  std::string segment_info = ss.str();
  return segment_info;
}

TrajectoryEstimator2::TrajectoryEstimator2(Twist_Trajectory::Ptr trajectory,
                                         TrajectoryEstimatorOptions& option,
                                         std::string descri)
    : trajectory_(trajectory), fixed_control_point_index_(-1) {
  this->options = option;
  problem_ = std::make_shared<ceres::Problem>(DefaultProblemOptions());

  residual_summary_.descri_info = descri;

  if (option.use_auto_diff) {
    auto_diff_local_parameterization_ = new LieLocalParameterization<SO3d>();
    analytic_local_parameterization_ = nullptr;
  } else {
    auto_diff_local_parameterization_ = nullptr;
    analytic_local_parameterization_ =
        new LieAnalyticLocalParameterization<SO3d>();
  }

  // For gravity
  homo_vec_local_parameterization_ =
      new ceres::HomogeneousVectorParameterization(3);

  marginalization_info_ = std::make_shared<MarginalizationInfo>();

  // 初始化时间偏置的内参地址
  for (auto& ep : trajectory_->GetSensorEPs()) {
    t_offset_ns_opt_params_[ep.first] = &ep.second.t_offset_ns;
  }

  ordering = std::make_shared<ceres::ParameterBlockOrdering>();


  // // HAO TODO:
  // std::vector<std::string> descriptions = {"Position", "Velocity"};
  // std::vector<size_t> block_sizes = {3, 3};  // 每个参数块的大小
  // std::vector<double*> param_blocks = {position, velocity};  // 对应的参数块指针

  // // 参数块指针，假设你有两个数组（例如 4D 和 3D 向量）来存储这些参数
  // std::vector<std::string> descriptions;
  // std::vector<size_t> block_sizes;
  // std::vector<double*> param_blocks;

  // // 第一个循环：为每个节点的 4D 参数块添加描述
  // for (size_t i = 0; i < knot_num; ++i) {
  //     descriptions.push_back("Control_Node_" + std::to_string(i) + "_quaternion");
  //     block_sizes.push_back(4);
  //     param_blocks.push_back(node_4d_params[i]);  // 指向对应的 4D 参数数组
  // }

  // // 第二个循环：为每个节点的 3D 参数块添加描述
  // for (size_t i = 0; i < knot_num; ++i) {
  //     descriptions.push_back("Control_Node_" + std::to_string(i) + "_position");
  //     block_sizes.push_back(3);
  //     param_blocks.push_back(node_3d_params[i]);  // 指向对应的 3D 参数数组
  // }

  // // 添加偏置的参数块
  // descriptions.push_back("Bias");
  // block_sizes.push_back(3);
  // param_blocks.push_back(linear_bias_params);  // 指向线性偏置的参数数组


  // // 假设 position 和 velocity 已经被定义为 double 数组
  // estimator.AddCallback(descriptions, block_sizes, param_blocks);

}

void TrajectoryEstimator2::SetKeyScanConstant(double max_time) {
  int64_t time_ns;
  if (!MeasuredTimeToNs(RadarSensor, max_time, time_ns)) return;

  std::pair<double, size_t> max_i_s = trajectory_->computeTIndexNs(time_ns);

  int index = 0;
  if (max_i_s.first < 0.5) {
    index = max_i_s.second + SplineOrder - 2;
  } else {
    index = max_i_s.second + SplineOrder - 1;
  }
  if (fixed_control_point_index_ < index) {
    fixed_control_point_index_ = index;
  }

  LOG(INFO) << "fixed_control_point_index: " << index << "/"
            << trajectory_->numKnots() << "; max_time: " << max_time
            << std::endl;
}

bool TrajectoryEstimator2::MeasuredTimeToNs(const SensorType& sensor_type,
                                           const double& timestamp,
                                           int64_t& time_ns) const {
  time_ns = timestamp * S_TO_NS;

  // 已考虑时间偏置
  int64_t t_min_traj_ns = trajectory_->minTime(sensor_type) * S_TO_NS;
  int64_t t_max_traj_ns = trajectory_->maxTime(sensor_type) * S_TO_NS;

  // LOG(ERROR) << " time_ns = " << time_ns << std::endl;
  // LOG(ERROR) << " t_max_traj_ns = " << t_max_traj_ns << ", t_min_traj_ns = " << t_min_traj_ns << std::endl;

  if (!options.lock_EPs.at(sensor_type).lock_t_offset) {
    // |____|______________________________|____|
    //    t_min                          t_max
    if (time_ns - options.t_offset_padding_ns < t_min_traj_ns ||
        time_ns + options.t_offset_padding_ns >= t_max_traj_ns)
      return false;
  } else {
    if (time_ns < t_min_traj_ns || time_ns >= t_max_traj_ns) return false;
  }

  return true;
}

void TrajectoryEstimator2::SetTimeoffsetState() {
  for (auto& sensor_t : t_offset_ns_opt_params_) {
    if (problem_->HasParameterBlock(sensor_t.second)) {
      if (options.lock_EPs.at(sensor_t.first).lock_t_offset)
        problem_->SetParameterBlockConstant(sensor_t.second);
    }
  }
}

void TrajectoryEstimator2::AddStartTimePose(const PoseData& pose) {
  // 若涉及到前 N 个控制点，则固定初始时刻位姿
  PoseData pose_temp = pose;
  pose_temp.timestamp = trajectory_->minTime(RadarSensor);

  double pos_weight = 100;
  double rot_weight = 100;
  Eigen::Matrix<double, 6, 1> info_vec;
  info_vec.head(3) = rot_weight * Eigen::Vector3d::Ones();
  info_vec.tail(3) = pos_weight * Eigen::Vector3d::Ones();
  AddPoseMeasurementAnalytic(pose_temp, info_vec);
}

bool TrajectoryEstimator2::IsParamUpdated(const double* values) const {
  if (problem_->HasParameterBlock(values) &&
      !problem_->IsParameterBlockConstant(const_cast<double*>(values))) {
    return true;
  } else {
    return false;
  }
}

void TrajectoryEstimator2::AddControlPoints(
    const SplineMeta<SplineOrder>& spline_meta, std::vector<double*>& vec,
    bool addPosKnot, bool use_order_opti) {
  for (auto const& seg : spline_meta.segments) {
    size_t start_idx = trajectory_->GetCtrlIndex(seg.t0_ns);
    LOG(ERROR) << "seg.NumParameters() = " << seg.NumParameters() << std::endl;
    for (size_t i = start_idx; i < (start_idx + seg.NumParameters()); ++i) {
      if (addPosKnot) {
        vec.emplace_back(trajectory_->getKnotLinear(i).data());
        problem_->AddParameterBlock(vec.back(), 3);
        if(use_order_opti)
        {
          ordering->AddElementToGroup(vec.back(), 0);
        }
      } else {
        vec.emplace_back(trajectory_->getKnotAngular(i).data());
        if(use_order_opti)
        {
          ordering->AddElementToGroup(vec.back(), 2);     // angular
        }
        if (options.use_auto_diff) {
          // problem_->AddParameterBlock(vec.back(), 4,
          //                             auto_diff_local_parameterization_);
          problem_->AddParameterBlock(vec.back(), 3);
        } else {
          // problem_->AddParameterBlock(vec.back(), 4,
          //                             analytic_local_parameterization_);
          problem_->AddParameterBlock(vec.back(), 3);
        }
      }
      if (options.lock_traj || (int)i <= fixed_control_point_index_) {
        problem_->SetParameterBlockConstant(vec.back());
        LOG(ERROR) << "parameter constant = " << vec.size() - 1 << " has been locked!" << std::endl;
        LOG(ERROR) << "i = " << i << " fixed_control_point_index_ = " << fixed_control_point_index_ << std::endl;
        
      }
      residual_summary_.AddKnotIdx(i, addPosKnot);
    }
  }

  // LOG(ERROR) << "vec.size = " << vec.size() << std::endl;
}

void TrajectoryEstimator2::SaveMarginalizationInfo(
    MarginalizationInfo::Ptr& marg_info_out,
    std::vector<double*>& marg_param_blocks_out) {
  // prepare the schur complement
  marginalization_info_->preMarginalize();
  bool ret = marginalization_info_->marginalize();

  if (ret) {
    marg_info_out = marginalization_info_;
    marg_param_blocks_out = marginalization_info_->getParameterBlocks();
  } else {
    // 边缘化后剩下的参数个数为零时,先验项没有意义
    marg_info_out = nullptr;
    marg_param_blocks_out.clear();
  }
}

void TrajectoryEstimator2::AddMarginalizationFactor(
    MarginalizationInfo::Ptr last_marginalization_info,
    std::vector<double*>& last_marginalization_parameter_blocks) {
  MarginalizationFactor* marginalization_factor =
      new MarginalizationFactor(last_marginalization_info);
  problem_->AddResidualBlock(marginalization_factor, NULL,
                             last_marginalization_parameter_blocks);

  if (options.show_residual_summary) {
    residual_summary_.AddResidualInfo(RType_Prior, marginalization_factor,
                                      last_marginalization_parameter_blocks);
  }
}

void TrajectoryEstimator2::PrepareMarginalizationInfo(
    ResidualType r_type, ceres::CostFunction* cost_function,
    ceres::LossFunction* loss_function, std::vector<double*>& parameter_blocks,
    std::vector<int>& drop_set) {
  ResidualBlockInfo* residual_block_info = new ResidualBlockInfo(
      r_type, cost_function, NULL, parameter_blocks, drop_set);
  marginalization_info_->addResidualBlockInfo(residual_block_info);
}

void TrajectoryEstimator2::PrepareMarginalizationInfo(
    ResidualType r_type, const SplineMeta<SplineOrder>& spline_meta,
    ceres::CostFunction* cost_function, ceres::LossFunction* loss_function,
    std::vector<double*>& parameter_blocks,
    std::vector<int>& drop_set_wo_ctrl_point) {

  // LOG(ERROR) << "save marginize info for " << ResidualTypeStr[r_type] << std::endl;
  // LOG(ERROR) << "spline_meta.NumParameters() = " << spline_meta.NumParameters() << std::endl;
  // LOG(ERROR) << "drop_set_wo_ctrl_point.size() = " << drop_set_wo_ctrl_point.size() << std::endl;

  // add contrl point id to drop set
  LOG(ERROR) << "PrepareMarginalizationInfo: options marginize = [ " << options.ctrl_to_be_opt_now << ", " 
            << options.ctrl_to_be_opt_later << "] " << std::endl;
  std::vector<int> drop_set = drop_set_wo_ctrl_point;
  if (options.ctrl_to_be_opt_later > options.ctrl_to_be_opt_now) {
    std::vector<int> ctrl_id;
    trajectory_->GetCtrlIdxs(spline_meta, ctrl_id);
    for (int i = 0; i < (int)ctrl_id.size(); ++i) {
      // LOG(ERROR) << "i = " << i << std::endl;
      // LOG(ERROR) << "ctrl_id = " << ctrl_id[i] << std::endl;
      if (ctrl_id[i] < options.ctrl_to_be_opt_later) {
        drop_set.emplace_back(i);
        // drop_set.emplace_back(i + spline_meta.NumParameters());
      }

      LOG(ERROR) << "ctrl_id[i] = " << ctrl_id[i] << std::endl;
    }
  }

  // std::fstream drop_set;

  LOG(ERROR) << "Marginize " << ResidualTypeStr[r_type] << " INFO drop information" << std::endl;
  // for(auto& d: drop_set)
  // {
  //   LOG(ERROR) << d << " ";
  // }

  LOG(ERROR) << "Marginize Spline Meta in size = " << drop_set.size() << std::endl;
  // 对之后的优化没有约束的因子直接丢就行,因为留下来也没有约束作用
  if (drop_set.size() > 0) {
    ResidualBlockInfo* residual_block_info = new ResidualBlockInfo(
        r_type, cost_function, loss_function, parameter_blocks, drop_set);
    marginalization_info_->addResidualBlockInfo(residual_block_info);
  }
}

void TrajectoryEstimator2::PrepareMarginalizationInfo(
    ResidualType r_type, const SplineMeta<SplineOrder>& linear_spline_meta,
    const SplineMeta<SplineOrder>& angular_spline_meta,
    ceres::CostFunction* cost_function, ceres::LossFunction* loss_function,
    std::vector<double*>& parameter_blocks,
    std::vector<int>& drop_set_wo_ctrl_point) {

  // LOG(ERROR) << "save marginize info for " << ResidualTypeStr[r_type] << std::endl;
  // LOG(ERROR) << "spline_meta.NumParameters() = " << spline_meta.NumParameters() << std::endl;
  // LOG(ERROR) << "drop_set_wo_ctrl_point.size() = " << drop_set_wo_ctrl_point.size() << std::endl;

  // add contrl point id to drop set
  std::vector<int> drop_set = drop_set_wo_ctrl_point;
  if (options.ctrl_to_be_opt_later > options.ctrl_to_be_opt_now) {
    LOG(ERROR) << "PrepareMarginalizationInfo: options marginize = [ " << options.ctrl_to_be_opt_now << ", " 
               << options.ctrl_to_be_opt_later << "] " << std::endl;
    std::vector<int> ctrl_id;
    trajectory_->GetCtrlIdxs(linear_spline_meta, ctrl_id);
    for (int i = 0; i < (int)ctrl_id.size(); ++i) {
      LOG(ERROR) << "i = " << i << std::endl;
      LOG(ERROR) << "ctrl_id = " << ctrl_id[i] << std::endl;
      if (ctrl_id[i] < options.ctrl_to_be_opt_later) {
        drop_set.emplace_back(i);
        // drop_set.emplace_back(i + spline_meta.NumParameters());
      }
    }
    int skip_id = ctrl_id.size();
    ctrl_id.clear();
    trajectory_->GetCtrlIdxs(angular_spline_meta, ctrl_id);
    for (int i = 0; i < (int)ctrl_id.size(); ++i) {
      LOG(ERROR) << "i = " << i << std::endl;
      LOG(ERROR) << "ctrl_id = " << ctrl_id[i] << std::endl;
      if (ctrl_id[i] < options.ctrl_to_be_opt_later) {
        drop_set.emplace_back(i + skip_id);
        // drop_set.emplace_back(i + spline_meta.NumParameters());
      }
    }
  }

  LOG(ERROR) << "Marginize " << ResidualTypeStr[r_type] << " INFO drop information" << std::endl;
  // for(auto& d: drop_set)
  // {
  //   LOG(ERROR) << d << " ";
  // }

  LOG(ERROR) << "Marginize Spline Meta in size = " << drop_set.size() << std::endl;
  // 对之后的优化没有约束的因子直接丢就行,因为留下来也没有约束作用
  if (drop_set.size() > 0) {
    ResidualBlockInfo* residual_block_info = new ResidualBlockInfo(
        r_type, cost_function, loss_function, parameter_blocks, drop_set);
    marginalization_info_->addResidualBlockInfo(residual_block_info);
  }
}



void TrajectoryEstimator2::AddCallback2(
    const std::vector<std::string>& descriptions,
    const std::vector<size_t>& block_size, 
    std::vector<double*>& param_block) {
  
  // Add callback for debug
  std::unique_ptr<CheckStateCallback> cb = std::make_unique<CheckStateCallback>();

  // 获取问题中参数块的数量
  int num_parameter_blocks = problem_->NumParameterBlocks();

  // 遍历每个参数块
  for (int i = 0; i < num_parameter_blocks; ++i) {
    // 获取每个参数块
    double* param_block_ptr = nullptr;
    // problem_->parameter_block(i, &param_block_ptr);

    // 使用预定义的 block_size 获取大小（假设你已经知道每个参数块的大小）
    size_t block_size_for_param = block_size[i];

    // 将描述符、参数块大小和参数块指针添加到回调中
    cb->addCheckState("ParamBlock_" + std::to_string(i), block_size_for_param, param_block_ptr);
  }

  // 将回调函数添加到 callbacks_ 中
  callbacks_.push_back(std::move(cb));

  // 如果需要状态更新，设置标志
  callback_needs_state_ = true;
}



ceres::Solver::Summary TrajectoryEstimator2::Solve(int max_iterations,
                                                  bool progress,
                                                  int num_threads) {
  ceres::Solver::Options options;

  // 可以尝试 线搜索 方法
  // options.minimizer_type = ceres::LINE_SEARCH; 

  options.minimizer_type = ceres::TRUST_REGION;

  // options.gradient_tolerance = 0.01 * Sophus::Constants<double>::epsilon();
  // options.function_tolerance = 0.01 * Sophus::Constants<double>::epsilon();

  // options.gradient_tolerance = 1e-20;
  // options.function_tolerance = 1e-20;
  // options.parameter_tolerance = 1e-20;

  options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
  //    options.trust_region_strategy_type = ceres::DOGLEG;
  //    options.dogleg_type = ceres::SUBSPACE_DOGLEG;
  // options.trust_region_strategy_type = ceres::DOGLEG;
  //    options.linear_solver_type = ceres::SPARSE_SCHUR;
  options.trust_region_strategy_type = ceres::LEVENBERG_MARQUARDT;
  // options.initial_lm_damping = 1;  // 设置初始阻尼
  // options.lm_min_norm = 0.8;   // 阻尼参数最小值

  // options.min_lm_diagonal = 0.5;
  // options.min_lm_diagonal = 0.05;
  // options.min_lm_diagonal = 0.5;
  // options.min_lm_diagonal = 1.0;
  options.min_lm_diagonal = 0.1;

  options.minimizer_progress_to_stdout = progress;

  // 调试信息
  // options.logging_type = ceres::PER_ITERATION; // 输出每次迭代的详细信息


  if (num_threads < 1) {
    num_threads = 1;  // std::thread::hardware_concurrency(); // mine is 8
    LOG(ERROR) << "use num_threads = " << num_threads << std::endl;
  }
  options.num_threads = num_threads;
  options.max_num_iterations = max_iterations;

  // std::vector<std::string> descriptions;
  // std::vector<size_t> block_size;
  // std::vector<double*> param_block;

  // AddCallback2(descriptions, block_size, param_block);
  // std::cout << "Check callbacks_ size = " << callbacks_.size() << std::endl;
  // LOG(ERROR) << "Check callbacks_ size = " << callbacks_.size() << std::endl;

  if (callbacks_.size() > 0) {
    LOG(ERROR) << "create callbacks = " << callbacks_.size() << std::endl;
    for (auto& cb : callbacks_) {
      options.callbacks.push_back(cb.get());
    }

    if (callback_needs_state_) options.update_state_every_iteration = true;
  }

  options.callbacks.push_back(new LMCallback()); // 注意：会自动释放
  options.update_state_every_iteration = true;

  LOG(ERROR) << "use ceres to solve " << std::endl;

  ceres::Solver::Summary summary;
  ceres::Solve(options, problem_.get(), &summary);

  LOG(ERROR) << "solve done!" << std::endl;

  trajectory_->UpdateExtrinsics();
  // trajectory_->UpdateTimeOffset(t_offset_ns_opt_params_);

  if (this->options.show_residual_summary) {
    residual_summary_.PrintSummary(trajectory_->minTimeNs(),
                                   trajectory_->getDtNs(),
                                   fixed_control_point_index_);
  }

  return summary;
}

// void TrajectoryEstimator2::AddDopplerMeasurementAnalytic(
//   double timestamp, const std::vector<Eigen::Vector3d>& point_vec, double* gyro_bias,
//   const std::vector<double>& doppler_vec, const Eigen::Matrix3d R_e_r,
//   double weight, bool marg_this_factor)
void TrajectoryEstimator2::AddDopplerMeasurementAnalytic(
  double timestamp, const Eigen::Vector3d& point, double* linear_bias,
  const double& doppler, const Eigen::Matrix3d R_e_r,
  double weight, bool marg_this_factor)
{

  // LOG(ERROR) << "before timestamp = " << std::setprecision(20) << timestamp << std::endl;

  int64_t time_ns;
  if (!MeasuredTimeToNs(RadarSensor, timestamp, time_ns)) return;
  SplineMeta<SplineOrder> spline_meta;

  LOG(ERROR) << "timestamp = " << std::setprecision(20) << timestamp << ", " << time_ns << std::endl;
  LOG(ERROR) << " Doppler Time: " << time_ns << std::endl;

  // if (option_Ep.lock_t_offset) {
  trajectory_->CaculateSplineMeta({{time_ns, time_ns}}, spline_meta);

  // LOG(ERROR) << "Spline meta time check: " 
  //             << spline_meta.segments.size() 
  //             << ", min = " << spline_meta.segments.front().MinTimeNs() 
  //             << ", max = " << spline_meta.segments.back().MaxTimeNs() << std::endl;

  // LOG(ERROR) << "Spline.segment(0) meta time check: " 
  //             << spline_meta.segments.size() 
  //             << ", min = " << spline_meta.segments.at(0).MinTimeNs() 
  //             << ", max = " << spline_meta.segments.at(0).MaxTimeNs()
  //             << ", data time = " << time_ns << std::endl;              

  // size_t idx = 0;
  // double u = 0;
  // spline_meta.ComputeSplineIndex(time_ns, idx, u);
  
  // 发现只使用了第一段
  // LOG(ERROR) << "check all spline_meta.size = " << spline_meta.segments.size() << std::endl;
  // std::pair<double, size_t> result = spline_meta.segments.at(0).computeTIndexNs(time_ns);
  // // std::pair<double, size_t> result = spline_meta.segments.at(0).computeTIndexNs(time_ns);
  // LOG(ERROR) << " Doppler Time in spline meta index = " << result.first << " u = " << result.second << std::endl;
  // LOG(ERROR) << "spline_meta.NumParameters = " << spline_meta.NumParameters() << std::endl;


  // HAO TODO: CHECK Address of bias
  // if(vel_bias_ != nullptr)
  //   LOG(ERROR) << "Doppler: para_bv_vec[1] address " << vel_bias_ << std::endl;
  // else
  //   LOG(ERROR) << "Doppler: para_bv_vec[1] not exist " << std::endl;

  // TODO:
  using Functor = DopplerFactor;
  // using Functor = ;
  // using Functor = NewDopplerFactor;
  ceres::CostFunction* cost_function =
  // new Functor(time_ns, point, doppler, 
  //     R_e_r, spline_meta, weight);
  new Functor(time_ns, point, - 1.0 * doppler,  // doppler 需要 取反
      R_e_r, spline_meta, weight, 0.00005);


  std::vector<double*> vec;
  AddControlPoints(spline_meta, vec);
  AddControlPoints(spline_meta, vec, true);
  vec.emplace_back(linear_bias);

  LOG(ERROR) << "Debug vec len = " << vec.size() << std::endl;

  // 4-3 修改
  // 锁存 旋转曲线
  // for (size_t i = 0; i < vec.size(); ++i) {
  //     if (i < 4) {  
  //         problem_->SetParameterBlockConstant(vec[i]);
  //     }
  // }


  problem_->AddParameterBlock(linear_bias, 3);

  // 1-11 添加
  // 设置参数的下界
  problem_->SetParameterLowerBound(linear_bias, 0, -1.2);
  // 设置参数的上界
  problem_->SetParameterUpperBound(linear_bias, 0, 1.2);
  // 设置参数的下界
  problem_->SetParameterLowerBound(linear_bias, 1, -1.2);
  // 设置参数的上界
  problem_->SetParameterUpperBound(linear_bias, 1, 1.2);
  // 设置参数的下界
  problem_->SetParameterLowerBound(linear_bias, 2, -1.2);
  // 设置参数的上界
  problem_->SetParameterUpperBound(linear_bias, 2, 1.2);


  // HAO Debug
  int num_r = cost_function->num_residuals();
  LOG(ERROR) << "Check Residuals: " << num_r << std::endl;

  std::vector<int> num_p = cost_function->parameter_block_sizes();
  LOG(ERROR) << "Check Parameters: " << num_p.size() << std::endl;
  // for(int i = 0;i < num_p.size(); i++)
  // {
  //   LOG(ERROR) << "block " << i << num_p[i] << std::endl;
  // }

  if (options.is_marg_state && marg_this_factor) {
    int num_residuals = cost_function->num_residuals();
    Eigen::MatrixXd residuals;
    residuals.setZero(num_residuals, 1);

    LOG(ERROR) << "Radar Doppler options.is_marg_state = " 
              << ((options.is_marg_state)? "True" : "False") << std::endl;

    LOG(ERROR) << "Radar Doppler magnize" << std::endl; 

    LOG(ERROR) << "Radar: residuals evaluate before magnize" << std::endl; 
    cost_function->Evaluate(vec.data(), residuals.data(), nullptr);

    LOG(ERROR) << "Radar residuals: " << residuals << std::endl; 
    
    // 检查毫米波雷达速度估计的残差
    // double dist = (residuals / weight).norm();
    // if (dist < 0.05) {
      std::vector<int> drop_set_wo_ctrl_point;
      int Knot_size = 2 * spline_meta.NumParameters();  
      drop_set_wo_ctrl_point.emplace_back(Knot_size);   // 边缘化 vel_bias
      PrepareMarginalizationInfo(RType_Radar, spline_meta, cost_function, NULL,
                                vec, drop_set_wo_ctrl_point);
    // }

  } else {
    // 2025-4-9 加入鲁棒估计
    // Tukey 核示例
    ceres::LossFunction* loss_function = new ceres::TukeyLoss(1.0);
    problem_->AddResidualBlock(cost_function, loss_function, vec);

    // LOG(ERROR) << "start to AddResidualBlock "  << std::endl;
    // problem_->AddResidualBlock(cost_function, NULL, vec);
  }

  if (options.show_residual_summary) {
    residual_summary_.AddResidualTimestamp(RType_Radar_Doppler, time_ns);
    residual_summary_.AddResidualInfo(RType_Radar_Doppler, cost_function, vec);

    // LOG(ERROR) << "residual summary start to AddResidualBlock " << std::endl;
  }

  /*{ /// debug: 
    // 假设 problem 是一个 ceres::Problem 对象
    // 假设 problem 是一个 ceres::Problem 对象
    int parameter_block_count = cost_function->parameter_block_sizes().size();
    LOG(ERROR) << "Number of parameter blocks: " << parameter_block_count;

    // 计算雅可比矩阵数量（通过残差块）
    int jacobian_count = 0;
    std::vector<ceres::ResidualBlockId> residual_block_ids;
    problem_->GetResidualBlocks(&residual_block_ids);  // 获取所有残差块的 ID

    for (const auto& residual_block_id : residual_block_ids) {
        // ceres::CostFunction* cost_function = problem_->GetCostFunction(residual_block_id);
        if (cost_function) {
            // 获取残差块的维度
            // int num_residuals = cost_function->GetNumResiduals();  // 获取残差的维度
            int num_residuals = cost_function->num_residuals();

            // 获取每个参数块的维度
            int num_parameters = 0;
            for (int i = 0; i < parameter_block_count; ++i) {
                num_parameters += cost_function->GetParameterBlockSize(i);  // 累加每个参数块的大小
            }

            // 如果需要计算雅可比矩阵，可以使用 Evaluate
            std::vector<double*> jacobians(num_parameters, nullptr);

            // 假设我们有某个参数值，这里以 `parameters` 为例
            double* parameters[parameter_block_count];

            // 对 cost_function 进行 Evaluate 操作
            double residuals[num_residuals];
            cost_function->Evaluate(parameters, residuals, jacobians.data());

            // 你可以在这里根据需要访问和处理计算出的雅可比矩阵
            jacobian_count++;  // 这里的计数器与残差块数相对应
        }
    }

    LOG(ERROR) << "Number of jacobians: " << jacobian_count;

  }*/

}



// void TrajectoryEstimator2::AddEventFlowMeasurementAnalytic(
//   double timestamp,
//   std::vector<cv::Point2d>& pixel_p, event_flow_velocity flow, 
//   const Eigen::Vector3d doppler_velocity,
//   Eigen::Quaterniond & q_e_r, Eigen::Vector3d& t_e_r,
//   Eigen::Vector3d angular_bias, double time_offset,
//   double weight, bool marg_this_factor);
  // void TrajectoryEstimator2::AddEventFlowMeasurementAnalytic(
  //   double timestamp,
  //   cv::Point2d pixel_p, event_flow_velocity flow, 
  //   const Eigen::Vector3d doppler_velocity,
  //   Eigen::Quaterniond & q_e_r, Eigen::Vector3d& t_e_r,
  //   Eigen::Vector3d angular_bias, double time_offset,
  //   double weight, bool marg_this_factor)
  void TrajectoryEstimator2::AddEventFlowMeasurementAnalytic(
    double timestamp,
    // cv::Point2d pixel_p, event_flow_velocity flow, 
    Eigen::Vector3d pixel_p, event_flow_velocity flow, 
    const Eigen::Vector3d doppler_velocity,
    Eigen::Quaterniond & q_e_r, Eigen::Vector3d& t_e_r, double* linear_bias,
    double* angular_bias, double* time_offset,
    double weight, bool marg_this_factor)
  {
    int64_t time_ns;
    if (!MeasuredTimeToNs(EventSensor, timestamp, time_ns)) return;
    SplineMeta<SplineOrder> spline_meta;
    trajectory_->CaculateSplineMeta({{time_ns, time_ns}}, spline_meta);

    size_t idx = 0;
    double u = 0;
    spline_meta.ComputeSplineIndex(time_ns, idx, u);
    LOG(ERROR) << " Event Time in spline meta index = " << idx << " u = " << u << std::endl;

    // LOG(WARNING) << "before contrust EventAgularFactor" << std::endl;

    // TODO: EventAgularFactor
    // using Functor = NewEventAgularFactor;
    using Functor = EventAgularFactor;
    ceres::CostFunction* cost_function =
    new Functor(time_ns, pixel_p, flow, doppler_velocity, q_e_r, t_e_r,
        spline_meta, weight, 0.00005);

    // LOG(WARNING) << "after contrust EventAgularFactor" << std::endl;

    std::vector<double*> vec;
    AddControlPoints(spline_meta, vec);
    AddControlPoints(spline_meta, vec, true); 
    // vec.push_back(angular_bias);  

    // HAO TODO: ADD Extrincs Optimization
    // if(false)
    // {
    //   vec.push_back(q_e_r.coeffs().data());
    //   vec.push_back(t_e_r.data());
    // }  

    // vec.push_back(time_offset);
    vec.emplace_back(linear_bias); 
    vec.emplace_back(angular_bias); 
    vec.emplace_back(time_offset);

    // 4-3 修改
    // 锁存 旋转曲线
    // for (size_t i = 0; i < vec.size(); ++i) {
    //   if (i > 3 && i < 9) {                               // pose + linear_bias
    //       problem_->SetParameterBlockConstant(vec[i]);
    //   }
    // }


    // HAO TODO: CHECK Address of bias
    // if(angular_bias != nullptr)
    //   LOG(ERROR) << "Event: para_bw_vec[1] address " << angular_bias << std::endl;
    // else
    //   LOG(ERROR) << "Event: para_bw_vec[1] not exist " << std::endl;

    // problem_->AddParameterBlock(linear_bias, 3);
    problem_->AddParameterBlock(angular_bias, 3);
    problem_->AddParameterBlock(time_offset, 1);


    // 1-11 添加
    // // 设置参数的下界
    // problem_->SetParameterLowerBound(linear_bias, 0, -1.2);
    // // 设置参数的上界
    // problem_->SetParameterUpperBound(linear_bias, 0, 1.2);
    // // 设置参数的下界
    // problem_->SetParameterLowerBound(linear_bias, 1, -1.2);
    // // 设置参数的上界
    // problem_->SetParameterUpperBound(linear_bias, 1, 1.2);
    //     // 设置参数的下界
    // problem_->SetParameterLowerBound(linear_bias, 2, -1.2);
    // // 设置参数的上界
    // problem_->SetParameterUpperBound(linear_bias, 2, 1.2);

    // 设置参数的下界
    problem_->SetParameterLowerBound(angular_bias, 0, -1.2);
    // 设置参数的上界
    problem_->SetParameterUpperBound(angular_bias, 0, 1.2);
    // 设置参数的下界
    problem_->SetParameterLowerBound(angular_bias, 1, -1.2);
    // 设置参数的上界
    problem_->SetParameterUpperBound(angular_bias, 1, 1.2);
    // 设置参数的下界
    problem_->SetParameterLowerBound(angular_bias, 2, -1.2);
    // 设置参数的上界
    problem_->SetParameterUpperBound(angular_bias, 2, 1.2);


    // LOG(WARNING) << "after add residual" << std::endl;

    if (options.is_marg_state && marg_this_factor) {
      int num_residuals = cost_function->num_residuals();
      Eigen::MatrixXd residuals;
      residuals.setZero(num_residuals, 1);

      LOG(ERROR) << "Marginize: Event options.is_marg_state = " 
                << ((options.is_marg_state)? "True" : "False") << std::endl;

      // LOG(ERROR) << "Event Flow magnize" << std::endl; 

      // LOG(ERROR) << "Marginize: Event residuals evaluate before magnize" << std::endl; 
      cost_function->Evaluate(vec.data(), residuals.data(), nullptr);

      LOG(ERROR) << "Marginize: Event residuals " << residuals << std::endl; 
      
      std::vector<int> drop_set_wo_ctrl_point;

      int Knot_size = 2 * spline_meta.NumParameters();
      drop_set_wo_ctrl_point.emplace_back(Knot_size);       // angular_bias 
      drop_set_wo_ctrl_point.emplace_back(Knot_size + 1);       // angular_bias     // 原来是 Knot_size
      // drop_set_wo_ctrl_point.emplace_back(Knot_size + 1);   // q_e_r
      // drop_set_wo_ctrl_point.emplace_back(Knot_size + 2);   // t_e_r
      // drop_set_wo_ctrl_point.emplace_back(Knot_size + 3);   // time_offset
      drop_set_wo_ctrl_point.emplace_back(Knot_size + 2);   // time_offset          // 原来是 Knot_size ++ 1
      PrepareMarginalizationInfo(RType_Event, spline_meta, cost_function, NULL,
                                vec, drop_set_wo_ctrl_point);
    } else {
      // 2025-4-9 加入鲁棒估计
      // Tukey 核示例
      ceres::LossFunction* loss_function = new ceres::TukeyLoss(1.0);
      problem_->AddResidualBlock(cost_function, loss_function, vec);


      // problem_->AddResidualBlock(cost_function, NULL, vec);
    }

    if (options.show_residual_summary) {
      residual_summary_.AddResidualTimestamp(RType_Event, time_ns);
      residual_summary_.AddResidualInfo(RType_Event, cost_function, vec);
    }
  }

/*
void TrajectoryEstimator2::AddDopplerMeasurementAnalytic2(
  double timestamp, SplineMeta<SplineOrder>& spline_meta, const Eigen::Vector3d& point, double* linear_bias,
  const double& doppler, const Eigen::Matrix3d R_e_r,
  double weight, bool marg_this_factor)
{

  // LOG(ERROR) << "before timestamp = " << std::setprecision(20) << timestamp << std::endl;

  int64_t time_ns;
  if (!MeasuredTimeToNs(RadarSensor, timestamp, time_ns)) return;
  // SplineMeta<SplineOrder> spline_meta;

  LOG(ERROR) << "timestamp = " << std::setprecision(20) << timestamp << ", " << time_ns << std::endl;
  LOG(ERROR) << " Doppler Time: " << time_ns << std::endl;

  // if (option_Ep.lock_t_offset) {
  trajectory_->CaculateSplineMeta({{time_ns, time_ns}}, spline_meta);

  // LOG(ERROR) << "Spline meta time check: " 
  //             << spline_meta.segments.size() 
  //             << ", min = " << spline_meta.segments.front().MinTimeNs() 
  //             << ", max = " << spline_meta.segments.back().MaxTimeNs() << std::endl;

  // LOG(ERROR) << "Spline.segment(0) meta time check: " 
  //             << spline_meta.segments.size() 
  //             << ", min = " << spline_meta.segments.at(0).MinTimeNs() 
  //             << ", max = " << spline_meta.segments.at(0).MaxTimeNs()
  //             << ", data time = " << time_ns << std::endl;              

  // size_t idx = 0;
  // double u = 0;
  // spline_meta.ComputeSplineIndex(time_ns, idx, u);
  
  // 发现只使用了第一段
  // LOG(ERROR) << "check all spline_meta.size = " << spline_meta.segments.size() << std::endl;
  // std::pair<double, size_t> result = spline_meta.segments.at(0).computeTIndexNs(time_ns);
  // // std::pair<double, size_t> result = spline_meta.segments.at(0).computeTIndexNs(time_ns);
  // LOG(ERROR) << " Doppler Time in spline meta index = " << result.first << " u = " << result.second << std::endl;
  // LOG(ERROR) << "spline_meta.NumParameters = " << spline_meta.NumParameters() << std::endl;


  // HAO TODO: CHECK Address of bias
  // if(vel_bias_ != nullptr)
  //   LOG(ERROR) << "Doppler: para_bv_vec[1] address " << vel_bias_ << std::endl;
  // else
  //   LOG(ERROR) << "Doppler: para_bv_vec[1] not exist " << std::endl;

  // TODO:
  // using Functor = NewDopplerFactor;
  using Functor = DopplerFactor;
  ceres::CostFunction* cost_function =
  // new Functor(time_ns, point, doppler, 
  //     R_e_r, spline_meta, weight);
  new Functor(time_ns, point, - 1.0 * doppler,  // doppler 需要 取反
      R_e_r, spline_meta, weight, 0.00005);


  std::vector<double*> vec;
  AddControlPoints(spline_meta, vec);
  AddControlPoints(spline_meta, vec, true);
  vec.emplace_back(linear_bias);

  LOG(ERROR) << "Debug vec len = " << vec.size() << std::endl;

  // 4-3 修改
  // 锁存 旋转曲线
  // for (size_t i = 0; i < vec.size(); ++i) {
  //     if (i < 4) {  
  //         problem_->SetParameterBlockConstant(vec[i]);
  //     }
  // }


  problem_->AddParameterBlock(linear_bias, 3);

  // 1-11 添加
  // 设置参数的下界
  problem_->SetParameterLowerBound(linear_bias, 0, -1.2);
  // 设置参数的上界
  problem_->SetParameterUpperBound(linear_bias, 0, 1.2);
  // 设置参数的下界
  problem_->SetParameterLowerBound(linear_bias, 1, -1.2);
  // 设置参数的上界
  problem_->SetParameterUpperBound(linear_bias, 1, 1.2);
  // 设置参数的下界
  problem_->SetParameterLowerBound(linear_bias, 2, -1.2);
  // 设置参数的上界
  problem_->SetParameterUpperBound(linear_bias, 2, 1.2);


  // HAO Debug
  int num_r = cost_function->num_residuals();
  LOG(ERROR) << "Check Residuals: " << num_r << std::endl;

  std::vector<int> num_p = cost_function->parameter_block_sizes();
  LOG(ERROR) << "Check Parameters: " << num_p.size() << std::endl;
  // for(int i = 0;i < num_p.size(); i++)
  // {
  //   LOG(ERROR) << "block " << i << num_p[i] << std::endl;
  // }

  if (options.is_marg_state && marg_this_factor) {
    int num_residuals = cost_function->num_residuals();
    Eigen::MatrixXd residuals;
    residuals.setZero(num_residuals, 1);

    LOG(ERROR) << "Radar Doppler options.is_marg_state = " 
              << ((options.is_marg_state)? "True" : "False") << std::endl;

    LOG(ERROR) << "Radar Doppler magnize" << std::endl; 

    LOG(ERROR) << "Radar: residuals evaluate before magnize" << std::endl; 
    cost_function->Evaluate(vec.data(), residuals.data(), nullptr);

    LOG(ERROR) << "Radar residuals: " << residuals << std::endl; 
    
    // 检查毫米波雷达速度估计的残差
    // double dist = (residuals / weight).norm();
    // if (dist < 0.05) {
      std::vector<int> drop_set_wo_ctrl_point;
      int Knot_size = 2 * spline_meta.NumParameters();  
      drop_set_wo_ctrl_point.emplace_back(Knot_size);   // 边缘化 vel_bias
      PrepareMarginalizationInfo(RType_Radar, spline_meta, cost_function, NULL,
                                vec, drop_set_wo_ctrl_point);
    // }

  } else {
    // LOG(ERROR) << "start to AddResidualBlock "  << std::endl;
    problem_->AddResidualBlock(cost_function, NULL, vec);
  }

  if (options.show_residual_summary) {
    residual_summary_.AddResidualTimestamp(RType_Radar_Doppler, time_ns);
    residual_summary_.AddResidualInfo(RType_Radar_Doppler, cost_function, vec);

    // LOG(ERROR) << "residual summary start to AddResidualBlock " << std::endl;
  }

  /*{ /// debug: 
    // 假设 problem 是一个 ceres::Problem 对象
    // 假设 problem 是一个 ceres::Problem 对象
    int parameter_block_count = cost_function->parameter_block_sizes().size();
    LOG(ERROR) << "Number of parameter blocks: " << parameter_block_count;

    // 计算雅可比矩阵数量（通过残差块）
    int jacobian_count = 0;
    std::vector<ceres::ResidualBlockId> residual_block_ids;
    problem_->GetResidualBlocks(&residual_block_ids);  // 获取所有残差块的 ID

    for (const auto& residual_block_id : residual_block_ids) {
        // ceres::CostFunction* cost_function = problem_->GetCostFunction(residual_block_id);
        if (cost_function) {
            // 获取残差块的维度
            // int num_residuals = cost_function->GetNumResiduals();  // 获取残差的维度
            int num_residuals = cost_function->num_residuals();

            // 获取每个参数块的维度
            int num_parameters = 0;
            for (int i = 0; i < parameter_block_count; ++i) {
                num_parameters += cost_function->GetParameterBlockSize(i);  // 累加每个参数块的大小
            }

            // 如果需要计算雅可比矩阵，可以使用 Evaluate
            std::vector<double*> jacobians(num_parameters, nullptr);

            // 假设我们有某个参数值，这里以 `parameters` 为例
            double* parameters[parameter_block_count];

            // 对 cost_function 进行 Evaluate 操作
            double residuals[num_residuals];
            cost_function->Evaluate(parameters, residuals, jacobians.data());

            // 你可以在这里根据需要访问和处理计算出的雅可比矩阵
            jacobian_count++;  // 这里的计数器与残差块数相对应
        }
    }

    LOG(ERROR) << "Number of jacobians: " << jacobian_count;

  }

*/

/*
void TrajectoryEstimator2::AddEventFlowMeasurementAnalytic2(
    double timestamp, SplineMeta<SplineOrder> spline_meta,
    // cv::Point2d pixel_p, event_flow_velocity flow, 
    Eigen::Vector3d pixel_p, event_flow_velocity flow, 
    const Eigen::Vector3d doppler_velocity,
    Eigen::Quaterniond & q_e_r, Eigen::Vector3d& t_e_r, double* linear_bias,
    double* angular_bias, double* time_offset,
    double weight, bool marg_this_factor)
  {
    int64_t time_ns;
    if (!MeasuredTimeToNs(EventSensor, timestamp, time_ns)) return;
    // SplineMeta<SplineOrder> spline_meta;
    // trajectory_->CaculateSplineMeta({{time_ns, time_ns}}, spline_meta);

    size_t idx = 0;
    double u = 0;
    spline_meta.ComputeSplineIndex(time_ns, idx, u);
    LOG(ERROR) << " Event Time in spline meta index = " << idx << " u = " << u << std::endl;

    // LOG(WARNING) << "before contrust EventAgularFactor" << std::endl;

    // TODO: EventAgularFactor
    // using Functor = NewEventAgularFactor;
    using Functor = EventAgularFactor;
    ceres::CostFunction* cost_function =
    new Functor(time_ns, pixel_p, flow, doppler_velocity, q_e_r, t_e_r,
        spline_meta, weight, 0.00005);

    // LOG(WARNING) << "after contrust EventAgularFactor" << std::endl;

    std::vector<double*> vec;
    AddControlPoints(spline_meta, vec);
    AddControlPoints(spline_meta, vec, true); 
    // vec.push_back(angular_bias);  

    // HAO TODO: ADD Extrincs Optimization
    // if(false)
    // {
    //   vec.push_back(q_e_r.coeffs().data());
    //   vec.push_back(t_e_r.data());
    // }  

    // vec.push_back(time_offset);
    vec.emplace_back(linear_bias); 
    vec.emplace_back(angular_bias); 
    vec.emplace_back(time_offset);

    // 4-3 修改
    // 锁存 旋转曲线
    // for (size_t i = 0; i < vec.size(); ++i) {
    //   if (i > 3 && i < 9) {                               // pose + linear_bias
    //       problem_->SetParameterBlockConstant(vec[i]);
    //   }
    // }


    // HAO TODO: CHECK Address of bias
    // if(angular_bias != nullptr)
    //   LOG(ERROR) << "Event: para_bw_vec[1] address " << angular_bias << std::endl;
    // else
    //   LOG(ERROR) << "Event: para_bw_vec[1] not exist " << std::endl;

    // problem_->AddParameterBlock(linear_bias, 3);
    problem_->AddParameterBlock(angular_bias, 3);
    problem_->AddParameterBlock(time_offset, 1);


    // 1-11 添加
    // // 设置参数的下界
    // problem_->SetParameterLowerBound(linear_bias, 0, -1.2);
    // // 设置参数的上界
    // problem_->SetParameterUpperBound(linear_bias, 0, 1.2);
    // // 设置参数的下界
    // problem_->SetParameterLowerBound(linear_bias, 1, -1.2);
    // // 设置参数的上界
    // problem_->SetParameterUpperBound(linear_bias, 1, 1.2);
    //     // 设置参数的下界
    // problem_->SetParameterLowerBound(linear_bias, 2, -1.2);
    // // 设置参数的上界
    // problem_->SetParameterUpperBound(linear_bias, 2, 1.2);

    // 设置参数的下界
    problem_->SetParameterLowerBound(angular_bias, 0, -1.2);
    // 设置参数的上界
    problem_->SetParameterUpperBound(angular_bias, 0, 1.2);
    // 设置参数的下界
    problem_->SetParameterLowerBound(angular_bias, 1, -1.2);
    // 设置参数的上界
    problem_->SetParameterUpperBound(angular_bias, 1, 1.2);
    // 设置参数的下界
    problem_->SetParameterLowerBound(angular_bias, 2, -1.2);
    // 设置参数的上界
    problem_->SetParameterUpperBound(angular_bias, 2, 1.2);


    // LOG(WARNING) << "after add residual" << std::endl;

    if (options.is_marg_state && marg_this_factor) {
      int num_residuals = cost_function->num_residuals();
      Eigen::MatrixXd residuals;
      residuals.setZero(num_residuals, 1);

      LOG(ERROR) << "Marginize: Event options.is_marg_state = " 
                << ((options.is_marg_state)? "True" : "False") << std::endl;

      // LOG(ERROR) << "Event Flow magnize" << std::endl; 

      // LOG(ERROR) << "Marginize: Event residuals evaluate before magnize" << std::endl; 
      cost_function->Evaluate(vec.data(), residuals.data(), nullptr);

      LOG(ERROR) << "Marginize: Event residuals " << residuals << std::endl; 
      
      std::vector<int> drop_set_wo_ctrl_point;

      int Knot_size = 2 * spline_meta.NumParameters();
      drop_set_wo_ctrl_point.emplace_back(Knot_size);       // angular_bias 
      drop_set_wo_ctrl_point.emplace_back(Knot_size + 1);       // angular_bias     // 原来是 Knot_size
      // drop_set_wo_ctrl_point.emplace_back(Knot_size + 1);   // q_e_r
      // drop_set_wo_ctrl_point.emplace_back(Knot_size + 2);   // t_e_r
      // drop_set_wo_ctrl_point.emplace_back(Knot_size + 3);   // time_offset
      drop_set_wo_ctrl_point.emplace_back(Knot_size + 2);   // time_offset          // 原来是 Knot_size ++ 1
      PrepareMarginalizationInfo(RType_Event, spline_meta, cost_function, NULL,
                                vec, drop_set_wo_ctrl_point);
    } else {
      problem_->AddResidualBlock(cost_function, NULL, vec);
    }

    if (options.show_residual_summary) {
      residual_summary_.AddResidualTimestamp(RType_Event, time_ns);
      residual_summary_.AddResidualInfo(RType_Event, cost_function, vec);
    }
  }
*/


// 松耦合
void TrajectoryEstimator2::AddBodyLocalVelocityMeasurementAnalytic(
    double timestamp, double* linear_bias,
    const Eigen::Vector3d local_vel,
    // Eigen::Vector3d pixel_p, event_flow_velocity flow, 
    // const Eigen::Vector3d doppler_velocity,
    // Eigen::Quaterniond & q_e_r, Eigen::Vector3d& t_e_r, 
    // double* time_offset,
    double weight, double w_weight, double R_weight,
    bool marg_this_factor)
  {
    int64_t time_ns;
    if (!MeasuredTimeToNs(RadarSensor, timestamp, time_ns)) return;
    SplineMeta<SplineOrder> linear_spline_meta;
    trajectory_->CaculateLinearSplineMeta({{time_ns, time_ns}}, linear_spline_meta);

    size_t idx = 0;
    double u = 0;
    linear_spline_meta.ComputeSplineIndex(time_ns, idx, u);
    LOG(ERROR) << " Event Time in spline meta index = " << idx << " u = " << u << std::endl;

    // LOG(WARNING) << "before contrust EventAgularFactor" << std::endl;
    LOG(ERROR) << "AddBodyLocalVelocityMeasurementAnalytic" << std::endl;
    // TODO: EventAgularFactor
    // using Functor = NewEventAgularFactor;
    // using Functor = BodyLocalVelocityFactor2; // BodyLocalVelocityFactor

    using Functor = BodyLocalVelocityFactor2;
    ceres::CostFunction* cost_function =
    new Functor(time_ns, local_vel, linear_spline_meta, weight, w_weight, R_weight);

    // LOG(WARNING) << "after contrust EventAgularFactor" << std::endl;

    std::vector<double*> vec;
    // AddControlPoints(spline_meta, vec);
    AddControlPoints(linear_spline_meta, vec, true); // linear knot
    vec.push_back(linear_bias); 

    // vec.push_back(angular_bias);  

    // HAO TODO: ADD Extrincs Optimization
    // if(false)
    // {
    //   vec.push_back(q_e_r.coeffs().data());
    //   vec.push_back(t_e_r.data());
    // }  

    // vec.push_back(time_offset);
    // vec.emplace_back(linear_bias); 
    // vec.emplace_back(angular_bias); 
    // vec.emplace_back(time_offset);

    // 4-3 修改
    // 锁存 旋转曲线
    // for (size_t i = 0; i < vec.size(); ++i) {
    //   if (i > 3 && i < 9) {                               // pose + linear_bias
    //       problem_->SetParameterBlockConstant(vec[i]);
    //   }
    // }


    // HAO TODO: CHECK Address of bias
    // if(angular_bias != nullptr)
    //   LOG(ERROR) << "Event: para_bw_vec[1] address " << angular_bias << std::endl;
    // else
    //   LOG(ERROR) << "Event: para_bw_vec[1] not exist " << std::endl;

    // problem_->AddParameterBlock(linear_bias, 3);
    // problem_->AddParameterBlock(linear_bias, 3);
    // problem_->AddParameterBlock(time_offset, 1);
    LOG(ERROR) << "linear body vec.size = " << vec.size() << std::endl;

    /*
    // 1-11 添加
    // 设置参数的下界
    problem_->SetParameterLowerBound(linear_bias, 0, -1.2);
    // 设置参数的上界
    problem_->SetParameterUpperBound(linear_bias, 0, 1.2);
    // 设置参数的下界
    problem_->SetParameterLowerBound(linear_bias, 1, -1.2);
    // 设置参数的上界
    problem_->SetParameterUpperBound(linear_bias, 1, 1.2);
        // 设置参数的下界
    problem_->SetParameterLowerBound(linear_bias, 2, -1.2);
    // 设置参数的上界
    problem_->SetParameterUpperBound(linear_bias, 2, 1.2);
    */

    // // 设置参数的下界
    // problem_->SetParameterLowerBound(angular_bias, 0, -1.2);
    // // 设置参数的上界
    // problem_->SetParameterUpperBound(angular_bias, 0, 1.2);
    // // 设置参数的下界
    // problem_->SetParameterLowerBound(angular_bias, 1, -1.2);
    // // 设置参数的上界
    // problem_->SetParameterUpperBound(angular_bias, 1, 1.2);
    // // 设置参数的下界
    // problem_->SetParameterLowerBound(angular_bias, 2, -1.2);
    // // 设置参数的上界
    // problem_->SetParameterUpperBound(angular_bias, 2, 1.2);


    // LOG(WARNING) << "after add residual" << std::endl;

    if (options.is_marg_state && marg_this_factor) {
      int num_residuals = cost_function->num_residuals();
      Eigen::MatrixXd residuals;
      residuals.setZero(num_residuals, 1);

      LOG(ERROR) << "Marginize: Event options.is_marg_state = " 
                << ((options.is_marg_state)? "True" : "False") << std::endl;

      // LOG(ERROR) << "Event Flow magnize" << std::endl; 

      // LOG(ERROR) << "Marginize: Event residuals evaluate before magnize" << std::endl; 
      cost_function->Evaluate(vec.data(), residuals.data(), nullptr);

      LOG(ERROR) << "Marginize: Event residuals " << residuals << std::endl; 
      
      std::vector<int> drop_set_wo_ctrl_point;

      int Knot_size = linear_spline_meta.NumParameters();
      drop_set_wo_ctrl_point.emplace_back(Knot_size);       // linear_bias 
      // drop_set_wo_ctrl_point.emplace_back(Knot_size + 1);       // angular_bias     // 原来是 Knot_size
      // drop_set_wo_ctrl_point.emplace_back(Knot_size + 1);   // q_e_r
      // drop_set_wo_ctrl_point.emplace_back(Knot_size + 2);   // t_e_r
      // drop_set_wo_ctrl_point.emplace_back(Knot_size + 3);   // time_offset
      // drop_set_wo_ctrl_point.emplace_back(Knot_size + 2);   // time_offset          // 原来是 Knot_size ++ 1

      LOG(ERROR) << "marginize size = " << drop_set_wo_ctrl_point.size() << std::endl;
      PrepareMarginalizationInfo(RType_Radar, linear_spline_meta, cost_function, NULL,
                                vec, drop_set_wo_ctrl_point);
    } else {
      problem_->AddResidualBlock(cost_function, NULL, vec);
    }

    if (options.show_residual_summary) {
      residual_summary_.AddResidualTimestamp(RType_Event, time_ns);
      residual_summary_.AddResidualInfo(RType_Event, cost_function, vec);
    }
  }


void TrajectoryEstimator2::AddBodyLocalAngularVelocityMeasurementAnalytic(
    double timestamp, double* angular_bias, 
    const Eigen::Vector3d local_angular_vel,
    // Eigen::Vector3d pixel_p, event_flow_velocity flow, 
    // const Eigen::Vector3d doppler_velocity,
    // Eigen::Quaterniond & q_e_r, Eigen::Vector3d& t_e_r, 
    // double* time_offset,
    double weight, double w_weight_,
    bool marg_this_factor)
  {
    int64_t time_ns;
    if (!MeasuredTimeToNs(RadarSensor, timestamp, time_ns)) return;
    SplineMeta<SplineOrder> spline_meta;
    trajectory_->CaculateSplineMeta({{time_ns, time_ns}}, spline_meta);

    size_t idx = 0;
    double u = 0;
    spline_meta.ComputeSplineIndex(time_ns, idx, u);
    LOG(ERROR) << " Event Time in spline meta index = " << idx << " u = " << u << std::endl;

    // LOG(WARNING) << "before contrust EventAgularFactor" << std::endl;
    
    // TODO: EventAgularFactor
    // using Functor = NewEventAgularFactor;
    using Functor = BodyLocalAngularVelocityFactor2;
    ceres::CostFunction* cost_function =
    new Functor(time_ns, local_angular_vel, spline_meta, weight, w_weight_);

    // LOG(WARNING) << "after contrust EventAgularFactor" << std::endl;

    std::vector<double*> vec;
    AddControlPoints(spline_meta, vec);
    // AddControlPoints(spline_meta, vec, true); 
    vec.push_back(angular_bias);  

    // HAO TODO: ADD Extrincs Optimization
    // if(false)
    // {
    //   vec.push_back(q_e_r.coeffs().data());
    //   vec.push_back(t_e_r.data());
    // }  

    // vec.push_back(time_offset);
    // vec.emplace_back(linear_bias); 
    // HAO TODO: 3-23 去除
    // vec.emplace_back(angular_bias); 
    // vec.emplace_back(time_offset);

    // 4-3 修改
    // 锁存 旋转曲线
    // for (size_t i = 0; i < vec.size(); ++i) {
    //   if (i > 3 && i < 9) {                               // pose + linear_bias
    //       problem_->SetParameterBlockConstant(vec[i]);
    //   }
    // }


    // HAO TODO: CHECK Address of bias
    // if(angular_bias != nullptr)
    //   LOG(ERROR) << "Event: para_bw_vec[1] address " << angular_bias << std::endl;
    // else
    //   LOG(ERROR) << "Event: para_bw_vec[1] not exist " << std::endl;

    // problem_->AddParameterBlock(linear_bias, 3);
    // // HAO TODO: 3-23 去除
    problem_->AddParameterBlock(angular_bias, 3);

    // problem_->AddParameterBlock(time_offset, 1);


    // // 1-11 添加
    // // 设置参数的下界
    // problem_->SetParameterLowerBound(linear_bias, 0, -1.2);
    // // 设置参数的上界
    // problem_->SetParameterUpperBound(linear_bias, 0, 1.2);
    // // 设置参数的下界
    // problem_->SetParameterLowerBound(linear_bias, 1, -1.2);
    // // 设置参数的上界
    // problem_->SetParameterUpperBound(linear_bias, 1, 1.2);
    //     // 设置参数的下界
    // problem_->SetParameterLowerBound(linear_bias, 2, -1.2);
    // // 设置参数的上界
    // problem_->SetParameterUpperBound(linear_bias, 2, 1.2);

    // HAO TODO: 3-23 去除
    
    // 设置参数的下界
    problem_->SetParameterLowerBound(angular_bias, 0, -1.2);
    // 设置参数的上界
    problem_->SetParameterUpperBound(angular_bias, 0, 1.2);
    // 设置参数的下界
    problem_->SetParameterLowerBound(angular_bias, 1, -1.2);
    // 设置参数的上界
    problem_->SetParameterUpperBound(angular_bias, 1, 1.2);
    // 设置参数的下界
    problem_->SetParameterLowerBound(angular_bias, 2, -1.2);
    // 设置参数的上界
    problem_->SetParameterUpperBound(angular_bias, 2, 1.2);
    


    // LOG(WARNING) << "after add residual" << std::endl;

    if (options.is_marg_state && marg_this_factor) {
      int num_residuals = cost_function->num_residuals();
      Eigen::MatrixXd residuals;
      residuals.setZero(num_residuals, 1);

      LOG(ERROR) << "Marginize: Event options.is_marg_state = " 
                << ((options.is_marg_state)? "True" : "False") << std::endl;

      // LOG(ERROR) << "Event Flow magnize" << std::endl; 

      // LOG(ERROR) << "Marginize: Event residuals evaluate before magnize" << std::endl; 
      cost_function->Evaluate(vec.data(), residuals.data(), nullptr);

      LOG(ERROR) << "Marginize: Event residuals " << residuals << std::endl; 
      
      std::vector<int> drop_set_wo_ctrl_point;

      // int Knot_size = 2 * spline_meta.NumParameters();
      int Knot_size = spline_meta.NumParameters();
      // drop_set_wo_ctrl_point.emplace_back(Knot_size);       // linear_bias 
      // drop_set_wo_ctrl_point.emplace_back(Knot_size + 1);
      drop_set_wo_ctrl_point.emplace_back(Knot_size);       // angular_bias     // 原来是 Knot_size
      // drop_set_wo_ctrl_point.emplace_back(Knot_size + 1);   // q_e_r
      // drop_set_wo_ctrl_point.emplace_back(Knot_size + 2);   // t_e_r
      // drop_set_wo_ctrl_point.emplace_back(Knot_size + 3);   // time_offset
      // drop_set_wo_ctrl_point.emplace_back(Knot_size + 2);   // time_offset          // 原来是 Knot_size ++ 1
      LOG(ERROR) << "marginize size = " << drop_set_wo_ctrl_point.size() << std::endl;
      PrepareMarginalizationInfo(RType_Event, spline_meta, cost_function, NULL,
                                vec, drop_set_wo_ctrl_point);
    } else {
      problem_->AddResidualBlock(cost_function, NULL, vec);
    }

    if (options.show_residual_summary) {
      residual_summary_.AddResidualTimestamp(RType_Event, time_ns);
      residual_summary_.AddResidualInfo(RType_Event, cost_function, vec);
    }
  }



void TrajectoryEstimator2::AddEventFlowMeasurementAnalytic2(
    double timestamp,
    Eigen::Vector3d pixel_p, event_flow_velocity flow, 
    const Eigen::Vector3d doppler_velocity,
    Eigen::Quaterniond & q_e_r, Eigen::Vector3d& t_e_r, double* linear_bias,
    double* angular_bias, double* time_offset,
    double weight, double w_weight, bool marg_this_factor)
  {
    int64_t time_ns;
    if (!MeasuredTimeToNs(EventSensor, timestamp, time_ns)) return;
    // SplineMeta<SplineOrder> linear_spline_meta;
    SplineMeta<SplineOrder> angular_spline_meta;
    // trajectory_->CaculateSplineMeta({{time_ns, time_ns}}, linear_spline_meta, angular_spline_meta);
    trajectory_->CaculateAngularSplineMeta({{time_ns, time_ns}}, angular_spline_meta);


    size_t idx = 0;
    double u = 0;
    angular_spline_meta.ComputeSplineIndex(time_ns, idx, u);
    LOG(ERROR) << " Event Time in spline meta index = " << idx << " u = " << u << std::endl;

    // LOG(WARNING) << "before contrust EventAgularFactor" << std::endl;
    LOG(ERROR) << "AddEventFlowMeasurementAnalytic2" << std::endl;

    // TODO: EventAgularFactor
    // using Functor = NewEventAgularFactor;
    using Functor = EventAgularFactor2;
    ceres::CostFunction* cost_function =
    new Functor(time_ns, pixel_p, flow, doppler_velocity, q_e_r, t_e_r,
        angular_spline_meta, weight, w_weight);

    // LOG(WARNING) << "after contrust EventAgularFactor" << std::endl;

    std::vector<double*> vec;
    // AddControlPoints(spline_meta, vec);
    AddControlPoints(angular_spline_meta, vec); 
    // vec.push_back(angular_bias);  

    // HAO TODO: ADD Extrincs Optimization
    // if(false)
    // {
    //   vec.push_back(q_e_r.coeffs().data());
    //   vec.push_back(t_e_r.data());
    // }  

    // vec.push_back(time_offset);

    // // HAO TODO: 3-23 去除
    // vec.emplace_back(linear_bias); 
    // vec.emplace_back(angular_bias);

    // vec.emplace_back(time_offset);
    // LOG(ERROR) << "angular vec.size = " << vec.size() << std::endl;
    LOG(ERROR) << "angular body vec.size = " << vec.size() << std::endl;
    // 4-3 修改
    // 锁存 旋转曲线
    // for (size_t i = 0; i < vec.size(); ++i) {
    //   if (i > 3 && i < 9) {                               // pose + linear_bias
    //       problem_->SetParameterBlockConstant(vec[i]);
    //   }
    // }


    // HAO TODO: CHECK Address of bias
    // if(angular_bias != nullptr)
    //   LOG(ERROR) << "Event: para_bw_vec[1] address " << angular_bias << std::endl;
    // else
    //   LOG(ERROR) << "Event: para_bw_vec[1] not exist " << std::endl;

    // // HAO TODO: 3-23 去除
    /*
    problem_->AddParameterBlock(linear_bias, 3);
    problem_->AddParameterBlock(angular_bias, 3);

    // problem_->AddParameterBlock(time_offset, 1);


    // 1-11 添加
    // 设置参数的下界
    problem_->SetParameterLowerBound(linear_bias, 0, -1.2);
    // 设置参数的上界
    problem_->SetParameterUpperBound(linear_bias, 0, 1.2);
    // 设置参数的下界
    problem_->SetParameterLowerBound(linear_bias, 1, -1.2);
    // 设置参数的上界
    problem_->SetParameterUpperBound(linear_bias, 1, 1.2);
        // 设置参数的下界
    problem_->SetParameterLowerBound(linear_bias, 2, -1.2);
    // 设置参数的上界
    problem_->SetParameterUpperBound(linear_bias, 2, 1.2);

    // 设置参数的下界
    problem_->SetParameterLowerBound(angular_bias, 0, -1.2);
    // 设置参数的上界
    problem_->SetParameterUpperBound(angular_bias, 0, 1.2);
    // 设置参数的下界
    problem_->SetParameterLowerBound(angular_bias, 1, -1.2);
    // 设置参数的上界
    problem_->SetParameterUpperBound(angular_bias, 1, 1.2);
    // 设置参数的下界
    problem_->SetParameterLowerBound(angular_bias, 2, -1.2);
    // 设置参数的上界
    problem_->SetParameterUpperBound(angular_bias, 2, 1.2);
    */

    // LOG(WARNING) << "after add residual" << std::endl;

    if (options.is_marg_state && marg_this_factor) {
      int num_residuals = cost_function->num_residuals();
      Eigen::MatrixXd residuals;
      residuals.setZero(num_residuals, 1);

      LOG(ERROR) << "Marginize: Event options.is_marg_state = " 
                << ((options.is_marg_state)? "True" : "False") << std::endl;

      // LOG(ERROR) << "Event Flow magnize" << std::endl; 

      // LOG(ERROR) << "Marginize: Event residuals evaluate before magnize" << std::endl; 
      cost_function->Evaluate(vec.data(), residuals.data(), nullptr);

      LOG(ERROR) << "Marginize: Event residuals " << residuals << std::endl; 
      
      std::vector<int> drop_set_wo_ctrl_point;

      int Knot_size = angular_spline_meta.NumParameters();
      // drop_set_wo_ctrl_point.emplace_back(Knot_size);       // linear_bias 
      // drop_set_wo_ctrl_point.emplace_back(Knot_size + 1);       // angular_bias     // 原来是 Knot_size
      // drop_set_wo_ctrl_point.emplace_back(Knot_size + 1);   // q_e_r
      // drop_set_wo_ctrl_point.emplace_back(Knot_size + 2);   // t_e_r
      // drop_set_wo_ctrl_point.emplace_back(Knot_size + 3);   // time_offset
      // drop_set_wo_ctrl_point.emplace_back(Knot_size + 2);   // time_offset          // 原来是 Knot_size ++ 1
      PrepareMarginalizationInfo(RType_Event, angular_spline_meta, cost_function, NULL,
                                vec, drop_set_wo_ctrl_point);
    } else {
      // 2025-4-9 加入鲁棒估计
      // Tukey 核示例
      // ceres::LossFunction* loss_function = new ceres::TukeyLoss(1.0);
      // problem_->AddResidualBlock(cost_function, loss_function,s vec);

      ceres::LossFunction* loss = new ceres::SoftLOneLoss(1e-16);
      problem_->AddResidualBlock(cost_function, loss, vec);
    }

    if (options.show_residual_summary) {
      residual_summary_.AddResidualTimestamp(RType_Event, time_ns);
      residual_summary_.AddResidualInfo(RType_Event, cost_function, vec);
    }
  }

void EvaluateCostFunctionWithJacobians(
    ceres::CostFunction* cost_function,
    double** parameters,
    Eigen::VectorXd& residuals,
    std::vector<Eigen::MatrixXd>& jacobians) {

  if (cost_function == nullptr || parameters == nullptr) {
    std::cerr << "Null pointer input!" << std::endl;
  }

  int num_residuals = cost_function->num_residuals();
  const std::vector<int>& param_sizes = cost_function->parameter_block_sizes();

  // 分配残差向量
  residuals.resize(num_residuals);
  residuals.setZero();

  // 准备雅可比矩阵缓冲区
  jacobians.clear();
  jacobians.reserve(param_sizes.size());
  std::vector<double*> jac_ptrs(param_sizes.size());

  for (size_t i = 0; i < param_sizes.size(); ++i) {
    int dim = param_sizes[i];
    jacobians.emplace_back(num_residuals, dim);
    jacobians.back().setZero();
    jac_ptrs[i] = jacobians.back().data();
  }

  // 调用Evaluate
  bool ret = cost_function->Evaluate(parameters, residuals.data(), jac_ptrs.data());
  if (!ret) {
    std::cerr << "Cost function Evaluate failed!" << std::endl;
  }
}



void TrajectoryEstimator2::AddDopplerMeasurementAnalytic2(
  double timestamp, const Eigen::Vector3d& point, double* linear_bias,
  const double& doppler, const Eigen::Matrix3d R_e_r, bool use_order_opti,
  double weight, double linear_w_weight, bool marg_this_factor)
{

  // LOG(ERROR) << "before timestamp = " << std::setprecision(20) << timestamp << std::endl;
  int64_t time_ns;
  if (!MeasuredTimeToNs(RadarSensor, timestamp, time_ns)) return;
  SplineMeta<SplineOrder> linear_spline_meta;
  trajectory_->CaculateLinearSplineMeta({{time_ns, time_ns}}, linear_spline_meta);
  // LOG(ERROR) << "timestamp = " << std::setprecision(20) << timestamp << ", " << time_ns << std::endl;
  // LOG(ERROR) << " Doppler Time: " << time_ns << std::endl;

  // TODO:
  // using Functor = NewDopplerFactor;
  using Functor = NewDopplerFactor;
  // using Functor = NewDopplerFactor;
  ceres::CostFunction* cost_function =
  // new Functor(time_ns, point, doppler, 
  //     R_e_r, spline_meta, weight);
  // new Functor(time_ns, point, - 1.0 * doppler,  // doppler 需要 取反
  //     R_e_r, linear_spline_meta, weight, 0.00005);

  new Functor(time_ns, point, - 1.0 * doppler,  // doppler 需要 取反
      linear_spline_meta, weight, linear_w_weight);

  std::vector<double*> vec;
  // AddControlPoints(spline_meta, vec);
  AddControlPoints(linear_spline_meta, vec, true, use_order_opti);
  vec.emplace_back(linear_bias);
  

  LOG(ERROR) << "Debug vec len = " << vec.size() << std::endl;

  // 4-3 修改
  // 锁存 旋转曲线
  // for (size_t i = 0; i < vec.size(); ++i) {
  //     if (i < 4) {  
  //         problem_->SetParameterBlockConstant(vec[i]);
  //     }
  // }


  problem_->AddParameterBlock(linear_bias, 3);
  if(use_order_opti)
  {
    ordering->AddElementToGroup(linear_bias, 1);
  }

  // 1-11 添加
  // 设置参数的下界
  problem_->SetParameterLowerBound(linear_bias, 0, -1.2);
  // 设置参数的上界
  problem_->SetParameterUpperBound(linear_bias, 0, 1.2);
  // 设置参数的下界
  problem_->SetParameterLowerBound(linear_bias, 1, -1.2);
  // 设置参数的上界
  problem_->SetParameterUpperBound(linear_bias, 1, 1.2);
  // 设置参数的下界
  problem_->SetParameterLowerBound(linear_bias, 2, -1.2);
  // 设置参数的上界
  problem_->SetParameterUpperBound(linear_bias, 2, 1.2);


  // HAO Debug
  /*
  int num_r = cost_function->num_residuals();
  LOG(ERROR) << "Check Residuals: " << num_r << std::endl;

  std::vector<int> num_p = cost_function->parameter_block_sizes();
  LOG(ERROR) << "Check Parameters: " << num_p.size() << std::endl;
  // for(int i = 0;i < num_p.size(); i++)
  // {
  //   LOG(ERROR) << "block " << i << num_p[i] << std::endl;
  // }
  */

  ceres::LossFunction* loss_function = new ceres::CauchyLoss(48.0); // 12.0
  // if (options.is_marg_state && marg_this_factor) {
  if (marg_this_factor) {
    int num_residuals = cost_function->num_residuals();
    Eigen::MatrixXd residuals;
    residuals.setZero(num_residuals, 1);

    // LOG(ERROR) << "Radar Doppler options.is_marg_state = " 
    //           << ((options.is_marg_state)? "True" : "False") << std::endl;

    // LOG(ERROR) << "Radar Doppler magnize" << std::endl; 

    // LOG(ERROR) << "Radar: residuals evaluate before magnize" << std::endl; 
    // cost_function->Evaluate(vec.data(), residuals.data(), nullptr);

    // LOG(ERROR) << "Radar residuals: " << residuals << std::endl; 
    
    // 检查毫米波雷达速度估计的残差
    // double dist = (residuals / weight).norm();
    // if (dist < 0.05) {
      std::vector<int> drop_set_wo_ctrl_point;
      int Knot_size = linear_spline_meta.NumParameters();  
      drop_set_wo_ctrl_point.emplace_back(Knot_size);   // 边缘化 vel_bias
      // PrepareMarginalizationInfo(RType_Radar, linear_spline_meta, cost_function, NULL,
      //                           vec, drop_set_wo_ctrl_point);
      PrepareMarginalizationInfo(RType_Radar, linear_spline_meta, cost_function, loss_function,
                                vec, drop_set_wo_ctrl_point);
    // }

  } else {
    // 2025-4-9 加入鲁棒估计
    // Tukey 核示例
    // ceres::LossFunction* loss_function = new ceres::TukeyLoss(1.0);
    // problem_->AddResidualBlock(cost_function, loss_function, vec);

    // ceres::LossFunction* loss_function = new ceres::HuberLoss(0.5);
    // problem_->AddResidualBlock(cost_function, loss_function, vec);


    
    // ceres::LossFunction* loss_function = new ceres::CauchyLoss(12.0); // 12.0
    // ceres::LossFunction* loss_function = new ceres::HuberLoss(1.2);
    problem_->AddResidualBlock(cost_function, loss_function, vec);

    // LOG(ERROR) << "start to AddResidualBlock "  << std::endl;
    // problem_->AddResidualBlock(cost_function, NULL, vec);
  }

  if (options.show_residual_summary) {
    residual_summary_.AddResidualTimestamp(RType_Radar_Doppler, time_ns);
    residual_summary_.AddResidualInfo(RType_Radar_Doppler, cost_function, vec);

    // LOG(ERROR) << "residual summary start to AddResidualBlock " << std::endl;
  }

  /*{ /// debug: 
    // 假设 problem 是一个 ceres::Problem 对象
    // 假设 problem 是一个 ceres::Problem 对象
    int parameter_block_count = cost_function->parameter_block_sizes().size();
    LOG(ERROR) << "Number of parameter blocks: " << parameter_block_count;

    // 计算雅可比矩阵数量（通过残差块）
    int jacobian_count = 0;
    std::vector<ceres::ResidualBlockId> residual_block_ids;
    problem_->GetResidualBlocks(&residual_block_ids);  // 获取所有残差块的 ID

    for (const auto& residual_block_id : residual_block_ids) {
        // ceres::CostFunction* cost_function = problem_->GetCostFunction(residual_block_id);
        if (cost_function) {
            // 获取残差块的维度
            // int num_residuals = cost_function->GetNumResiduals();  // 获取残差的维度
            int num_residuals = cost_function->num_residuals();

            // 获取每个参数块的维度
            int num_parameters = 0;
            for (int i = 0; i < parameter_block_count; ++i) {
                num_parameters += cost_function->GetParameterBlockSize(i);  // 累加每个参数块的大小
            }

            // 如果需要计算雅可比矩阵，可以使用 Evaluate
            std::vector<double*> jacobians(num_parameters, nullptr);

            // 假设我们有某个参数值，这里以 `parameters` 为例
            double* parameters[parameter_block_count];

            // 对 cost_function 进行 Evaluate 操作
            double residuals[num_residuals];
            cost_function->Evaluate(parameters, residuals, jacobians.data());

            // 你可以在这里根据需要访问和处理计算出的雅可比矩阵
            jacobian_count++;  // 这里的计数器与残差块数相对应
        }
    }

    LOG(ERROR) << "Number of jacobians: " << jacobian_count;

  }*/

}

// 只要后一项的误差
void TrajectoryEstimator2::AddEventFlowMeasurementAnalytic3(
    double timestamp,
    Eigen::Vector3d pixel_p, event_flow_velocity flow, 
    const Eigen::Vector3d doppler_velocity,
    Eigen::Quaterniond & q_e_r, Eigen::Vector3d& t_e_r, double* linear_bias,
    double* angular_bias, double* time_offset,
    std::shared_ptr<FEJ_STATE> global_fej_state_, bool use_fej, bool use_order_opti, 
    double weight, double w_weight, bool marg_this_factor)
  {
    int64_t time_ns;
    if (!MeasuredTimeToNs(EventSensor, timestamp, time_ns)) return;
    SplineMeta<SplineOrder> linear_spline_meta;
    SplineMeta<SplineOrder> angular_spline_meta;
    // trajectory_->CaculateSplineMeta({{time_ns, time_ns}}, linear_spline_meta, angular_spline_meta);
    // trajectory_->CaculateAngularSplineMeta({{time_ns, time_ns}}, linear_spline_meta);
    // trajectory_->CaculateAngularSplineMeta({{time_ns, time_ns}}, angular_spline_meta);
    trajectory_->CaculateSplineMeta({{time_ns, time_ns}}, linear_spline_meta, angular_spline_meta);


    // size_t idx = 0;
    // double u = 0;
    // angular_spline_meta.ComputeSplineIndex(time_ns, idx, u);
    // LOG(ERROR) << " Event Time in spline meta index = " << idx << " u = " << u << std::endl;

    // LOG(WARNING) << "before contrust EventAgularFactor" << std::endl;
    LOG(ERROR) << "AddEventFlowMeasurementAnalytic2" << std::endl;

    // TODO: EventAgularFactor
    // using Functor = NewEventAgularFactor;
    // using Functor = EventAgularFactor3;
    // ceres::CostFunction* cost_function =
    // new Functor(time_ns, pixel_p, flow, doppler_velocity, q_e_r, t_e_r,
    //    linear_spline_meta, angular_spline_meta, linear_bias, angular_bias, weight, w_weight);

    using Functor = EventAgularFactor4;
    ceres::CostFunction* cost_function =
    new Functor(time_ns, pixel_p, flow, doppler_velocity, q_e_r, t_e_r,
       linear_spline_meta, angular_spline_meta, 
       global_fej_state_, use_fej,
       weight, w_weight);



    // LOG(WARNING) << "after contrust EventAgularFactor" << std::endl;

    std::vector<double*> vec;
    // AddControlPoints(spline_meta, vec);
    AddControlPoints(linear_spline_meta, vec, true, use_order_opti); 
    AddControlPoints(angular_spline_meta, vec, false, use_order_opti); 

    vec.push_back(linear_bias);  
    vec.push_back(angular_bias); 
    
    // HAO TODO: ADD Extrincs Optimization
    // if(false)
    // {
    //   vec.push_back(q_e_r.coeffs().data());
    //   vec.push_back(t_e_r.data());
    // }  

    // vec.push_back(time_offset);

    // // HAO TODO: 3-23 去除
    // vec.emplace_back(linear_bias); 
    // vec.emplace_back(angular_bias);

    // vec.emplace_back(time_offset);
    // LOG(ERROR) << "angular vec.size = " << vec.size() << std::endl;
    LOG(ERROR) << "angular body vec.size = " << vec.size() << std::endl;
    // 4-3 修改
    // 锁存 旋转曲线
    // for (size_t i = 0; i < vec.size(); ++i) {
    //   if (i > 3 && i < 9) {                               // pose + linear_bias
    //       problem_->SetParameterBlockConstant(vec[i]);
    //   }
    // }


    // HAO TODO: CHECK Address of bias
    // if(angular_bias != nullptr)
    //   LOG(ERROR) << "Event: para_bw_vec[1] address " << angular_bias << std::endl;
    // else
    //   LOG(ERROR) << "Event: para_bw_vec[1] not exist " << std::endl;

    // // HAO TODO: 3-23 去除

    problem_->AddParameterBlock(linear_bias, 3);
    problem_->AddParameterBlock(angular_bias, 3);

    // problem_->AddParameterBlock(time_offset, 1);


    // 1-11 添加
    // 设置参数的下界
    problem_->SetParameterLowerBound(linear_bias, 0, -1.2);
    // 设置参数的上界
    problem_->SetParameterUpperBound(linear_bias, 0, 1.2);
    // 设置参数的下界
    problem_->SetParameterLowerBound(linear_bias, 1, -1.2);
    // 设置参数的上界
    problem_->SetParameterUpperBound(linear_bias, 1, 1.2);
        // 设置参数的下界
    problem_->SetParameterLowerBound(linear_bias, 2, -1.2);
    // 设置参数的上界
    problem_->SetParameterUpperBound(linear_bias, 2, 1.2);

    // 设置参数的下界
    problem_->SetParameterLowerBound(angular_bias, 0, -1.2);
    // 设置参数的上界
    problem_->SetParameterUpperBound(angular_bias, 0, 1.2);
    // 设置参数的下界
    problem_->SetParameterLowerBound(angular_bias, 1, -1.2);
    // 设置参数的上界
    problem_->SetParameterUpperBound(angular_bias, 1, 1.2);
    // 设置参数的下界
    problem_->SetParameterLowerBound(angular_bias, 2, -1.2);
    // 设置参数的上界
    problem_->SetParameterUpperBound(angular_bias, 2, 1.2);
    if(use_order_opti)
    {
      ordering->AddElementToGroup(linear_bias, 1);
      ordering->AddElementToGroup(angular_bias, 3);
    }

    // LOG(WARNING) << "after add residual" << std::endl;

    // Evaluate this problem
    {
      Eigen::VectorXd residuals;
      std::vector<Eigen::MatrixXd> jacobians;

      EvaluateCostFunctionWithJacobians(cost_function, vec.data(), residuals, jacobians);

      std::ofstream residual_file("/home/hao/Desktop/radar-event-new/src/TwistEstimator/output/res.txt", std::ios::out | std::ios::app);
      residual_file << residuals << std::endl;
      residual_file.close();

      std::ofstream jac_knot_file("/home/hao/Desktop/radar-event-new/src/TwistEstimator/output/jac.txt", std::ios::out | std::ios::app);
      for (const auto& jac : jacobians)
      {
          for (int i = 0; i < jac.rows(); ++i) {
              for (int j = 0; j < jac.cols(); ++j) {
                  jac_knot_file << jac(i, j) << " ";
              }
          // jac_knot_file << jac << " ";
          }
      }
      // jac_knot_file << "\n";
      // jac_knot_file << "done " << std::endl;
      jac_knot_file << std::endl;
      jac_knot_file.close();
    }


    ceres::LossFunction* loss_function = new ceres::CauchyLoss(65.0);
    if (marg_this_factor) {
      int num_residuals = cost_function->num_residuals();
      Eigen::MatrixXd residuals;
      residuals.setZero(num_residuals, 1);

      LOG(ERROR) << "Marginize: Event options.is_marg_state = " 
                << ((options.is_marg_state)? "True" : "False") << std::endl;

      // LOG(ERROR) << "Event Flow magnize" << std::endl; 

      // LOG(ERROR) << "Marginize: Event residuals evaluate before magnize" << std::endl; 
      // cost_function->Evaluate(vec.data(), residuals.data(), nullptr);

      LOG(ERROR) << "Marginize: Event residuals " << residuals << std::endl; 
      
      std::vector<int> drop_set_wo_ctrl_point;

      int Knot_size = angular_spline_meta.NumParameters();
      drop_set_wo_ctrl_point.emplace_back(2 * Knot_size);       // linear_bias 
      drop_set_wo_ctrl_point.emplace_back(2 * Knot_size + 1);       // angular_bias     // 原来是 Knot_size
      // drop_set_wo_ctrl_point.emplace_back(Knot_size + 1);   // q_e_r
      // drop_set_wo_ctrl_point.emplace_back(Knot_size + 2);   // t_e_r
      // drop_set_wo_ctrl_point.emplace_back(Knot_size + 3);   // time_offset
      // drop_set_wo_ctrl_point.emplace_back(Knot_size + 2);   // time_offset          // 原来是 Knot_size ++ 1
      // PrepareMarginalizationInfo(RType_Event, linear_spline_meta, angular_spline_meta, cost_function, NULL,
      //                           vec, drop_set_wo_ctrl_point);
      PrepareMarginalizationInfo(RType_Event, linear_spline_meta, angular_spline_meta, cost_function, loss_function,
                                vec, drop_set_wo_ctrl_point);
      // PrepareMarginalizationInfo(RType_Event, linear_spline_meta, cost_function, NULL,
      //                     vec, drop_set_wo_ctrl_point);
      // PrepareMarginalizationInfo(RType_Event, angular_spline_meta, cost_function, NULL,
      //                     vec, drop_set_wo_ctrl_point);

                          
    } else {
      // 2025-4-9 加入鲁棒估计
      // Tukey 核示例
      // ceres::LossFunction* loss_function = new ceres::TukeyLoss(1.0);
      // problem_->AddResidualBlock(cost_function, loss_function,s vec);

      // ceres::LossFunction* loss = new ceres::SoftLOneLoss(1e-16);
      // problem_->AddResidualBlock(cost_function, loss, vec);


       // 25.0
      // ceres::LossFunction* loss_function = new ceres::CauchyLoss(25.0); // 25.0
      // ceres::LossFunction* loss_function = new ceres::HuberLoss(1.5);
      problem_->AddResidualBlock(cost_function, loss_function, vec);

      // LOG(ERROR) << "ADD" << std::endl;
      // problem_->AddResidualBlock(cost_function, NULL, vec);
      // LOG(ERROR) << "ADD Done" << std::endl;
    }

    if (options.show_residual_summary) {
      residual_summary_.AddResidualTimestamp(RType_Event, time_ns);
      residual_summary_.AddResidualInfo(RType_Event, cost_function, vec);
    }
  }


void TrajectoryEstimator2::AddBiasFactor(
    double* bias_gyr_i, double* bias_gyr_j, double* bias_acc_i,
    double* bias_acc_j, double dt, const Eigen::Matrix<double, 6, 1>& info_vec,
    bool marg_this_factor, bool marg_all_bias) {
  NewBiasFactor* cost_function =
      new NewBiasFactor(dt, info_vec);

  std::vector<double*> vec;
  vec.emplace_back(bias_gyr_i);
  vec.emplace_back(bias_gyr_j);
  vec.emplace_back(bias_acc_i);
  vec.emplace_back(bias_acc_j);

  // HAO TODO: CHECK Address of bias
  if(bias_gyr_i != nullptr)
    LOG(ERROR) << "Bias: para_bw_vec[0] address " << bias_gyr_i << std::endl;
  else
    LOG(ERROR) << "Bias: para_bw_vec[0] not exist " << std::endl;
  if(bias_gyr_j != nullptr)
    LOG(ERROR) << "Bias: para_bw_vec[1] address " << bias_gyr_j << std::endl;
  else
    LOG(ERROR) << "Bias: para_bw_vec[1] not exist " << std::endl;
  if(bias_acc_j != nullptr)
    LOG(ERROR) << "Bias: para_bv_vec[0] address " << bias_acc_i << std::endl;
  else
    LOG(ERROR) << "Bias: para_bv_vec[0] not exist " << std::endl;
  if(bias_acc_j != nullptr)
    LOG(ERROR) << "Bias: para_bv_vec[1] address " << bias_acc_j << std::endl;
  else
    LOG(ERROR) << "Bias: para_bv_vec[1] not exist " << std::endl;

  LOG(ERROR) << "options.lock_wb? " << ((options.lock_wb)? "True": "False");
  LOG(ERROR) << "options.lock_ab? " << ((options.lock_ab)? "True": "False");

  /*{
    problem_->AddParameterBlock(bias_gyr_i, 3);
    problem_->AddParameterBlock(bias_gyr_j, 3);

    if (options.lock_wb) {
      problem_->SetParameterBlockConstant(bias_gyr_i);
      problem_->SetParameterBlockConstant(bias_gyr_j);
    }

    problem_->AddParameterBlock(bias_acc_i, 3);
    problem_->AddParameterBlock(bias_acc_j, 3);

    if (options.lock_ab) {
      problem_->SetParameterBlockConstant(bias_acc_i);
      problem_->SetParameterBlockConstant(bias_acc_j);
    }
      }*/


  problem_->AddParameterBlock(bias_gyr_i, 3);
  problem_->AddParameterBlock(bias_gyr_j, 3);
  problem_->AddParameterBlock(bias_acc_i, 3);
  problem_->AddParameterBlock(bias_acc_j, 3);

  ordering->AddElementToGroup(bias_gyr_i, 3); // 假设控制点分组为0
  ordering->AddElementToGroup(bias_gyr_j, 3); // 假设控制点分组为0
  ordering->AddElementToGroup(bias_acc_i, 1); // 假设控制点分组为0
  ordering->AddElementToGroup(bias_acc_j, 1); // 假设控制点分组为0

  problem_->SetParameterBlockConstant(bias_gyr_i);
  problem_->SetParameterBlockConstant(bias_acc_i);

  // LOG(ERROR) << "check vec" << std::endl;
  // int count = 0;
  // for(auto& v: vec)
  // {
  //   LOG(ERROR) << "vec[" << count++ << "] = " << v[0] << ", " << v[1] << ", " << v[2] << std::endl;
  // }


  if (options.is_marg_state && marg_this_factor) {
    LOG(ERROR) << "Marginize Twist Bias" << std::endl;
    // bias_gyr_i, bias_acc_i
    std::vector<int> drop_set = {0, 2};
    if (!options.marg_bias_param) drop_set.clear();
    if (options.marg_bias_param && marg_all_bias) {
      drop_set = {0, 1, 2, 3};
    }

    Eigen::MatrixXd residuals;
    LOG(ERROR) << "options.is_marg_state = " 
          << ((options.is_marg_state)? "True" : "False") << std::endl;

    LOG(ERROR) << "Bias magnize" << std::endl; 

    LOG(ERROR) << "Bias: residuals evaluate before magnize" << std::endl; 
    cost_function->Evaluate(vec.data(), residuals.data(), nullptr);

    LOG(ERROR) << "Bias residuals: " << residuals << std::endl; 
    PrepareMarginalizationInfo(RType_Bias, cost_function, NULL, vec, drop_set);
  } else {
    problem_->AddResidualBlock(cost_function, NULL, vec);
  }

  if (options.show_residual_summary) {
    residual_summary_.AddResidualInfo(RType_Bias, cost_function, vec);
  }
}


void TrajectoryEstimator2::AddTwistBiasAnalytic(
    double* bias_w_i, double* bias_w_j, double* bias_v_i,
    double* bias_v_j, double dt, const Eigen::Matrix<double, 6, 1>& info_vec,
    bool marg_this_factor, bool marg_all_bias) {
  NewBiasFactor* cost_function =
      new NewBiasFactor(dt, info_vec);

  std::vector<double*> vec;
  vec.emplace_back(bias_w_i);
  vec.emplace_back(bias_w_j);
  vec.emplace_back(bias_v_i);
  vec.emplace_back(bias_v_j);

  if (options.lock_wb) {
    problem_->AddParameterBlock(bias_w_i, 3);
    problem_->AddParameterBlock(bias_w_j, 3);
    problem_->SetParameterBlockConstant(bias_w_i);
    problem_->SetParameterBlockConstant(bias_w_j);
  }
  if (options.lock_ab) {
    problem_->AddParameterBlock(bias_v_i, 3);
    problem_->AddParameterBlock(bias_v_j, 3);
    problem_->SetParameterBlockConstant(bias_v_i);
    problem_->SetParameterBlockConstant(bias_v_j);
  }

  if (options.is_marg_state && marg_this_factor) {
    
    // bias_w_i, bias_v_i
    std::vector<int> drop_set = {0, 2};
    if (!options.marg_bias_param) drop_set.clear();
    if (options.marg_bias_param && marg_all_bias) {
      drop_set = {0, 1, 2, 3};
    }
    PrepareMarginalizationInfo(RType_Bias, cost_function, NULL, vec, drop_set);
  } else {
    problem_->AddResidualBlock(cost_function, NULL, vec);
  }

  if (options.show_residual_summary) {
    residual_summary_.AddResidualInfo(RType_Bias, cost_function, vec);
  }
}
