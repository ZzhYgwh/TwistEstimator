#ifndef __TwistEstimator__H__
#define __TwistEstimator__H__

#define UPDATE_PRIOR 1  // 定义一个宏
#define VERBOSE 0  // 定义一个宏

#include <boost/filesystem.hpp>

#include <sensor_msgs/point_cloud2_iterator.h>

#include "event_flow_detector/event_flow_detector.h"
// #include "event_flow_detector/event_ct_flow_LP.h"

#include <yaml-cpp/yaml.h>

#include <fstream>

#include "utils/log_utils.h"
#include "utils/parameter_struct.h"
#include "utils/yaml_utils.h"

// #include "spline/se3_spline.h"

// #include "spline/trajectory.cpp"
// #include "trajectory_estimator_options.h"

#include "spline/rd6_spline.h"

#include "estimator/trajectory_estimator.h"


#include "estimator/trajectory_estimator2.h"

// #include "factor/TwistFactor.h"
#include "factor/marginalization_factor.h"

#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/TwistWithCovarianceStamped.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl/common/transforms.h>            //pcl::transformPointCloud
#include <pcl_conversions/pcl_conversions.h>  //pcl::fromROSMsg
#include <eigen_conversions/eigen_msg.h>

// #include "spline/rd6_spline.h"
#include <sys/sysinfo.h>

using SO3d = Sophus::SO3<double>;
using SE3d = Sophus::SE3<double>;

// typedef pcl::PointXYZI VPoint;
// typedef pcl::PointCloud<VPoint> VPointCloud;

// #define gravity_ 9.81

// namespace twist_estimator{

struct TwistBias {
  TwistBias()
      : omega_bias(Eigen::Vector3d::Zero()),
        vel_bias(Eigen::Vector3d::Zero()) {}
  Eigen::Vector3d omega_bias;
  Eigen::Vector3d vel_bias;
};

class TwistEstimator
{
EIGEN_MAKE_ALIGNED_OPERATOR_NEW
public:
// **** Global Variable ****
// TODO: add this to yaml file
const int gyro_weight = 50;
const int velocity_weight = 50;

Trajectory::Ptr trajectory_;
TrajectoryEstimatorOptions options;
int update_every_k_knot_;
// double time_interval;

// ----------------------- 速度空间的优化 ---------------------------
// std::shared_ptr<RdSpline> linear_spline;
// std::shared_ptr<RdSpline> angular_spline;
// Rd6Spline twist_spline;


std::string output_path;
std::fstream output_file;
double output_dt;

double local_dt = 3.0;  // 定义局部时间范围

// std::string ceres_debug_path;
// std::fstream ceres_debug_file;

Eigen::Quaterniond q0;
Eigen::Vector3d t0;

bool opt_time_offset_ = true;
bool start_opt_time_offset_ = true;

//声明两个时间
double last_time;   
double cur_time;
double relative_start_time;

double omega_weight = 1.0;
double linear_weight = 1.0;

double omega_w_weight = 1.0;
double linear_w_weight = 1.0;
double R_weight = 1.0;

bool use_prior;
bool use_fej;
bool use_order_opti;
std::shared_ptr<FEJ_STATE> global_fej_state_; // FEJ State

// time offset event w.r.t radar
double* time_offset;

VPointCloud marg_ctrl_point_;
VPointCloud init_ctrl_point_;

std::vector<TwistData> twist_vec_;
std::vector<TwistData2> twist2_vec_;

std::vector<TwistData2> twist2_margin_vec_;
bool initial_done;
double data_start_time;

// #ifdef USE_IWE_ESTI
  std::vector<TwistVelocity> twist_velocity_vec_;
// #endif
// 滑窗内， twist_vec_ 的终止正向迭代器
std::vector<TwistData>::iterator it_base;
std::vector<TwistVelocity>::iterator it_base_vel;

std::vector<TwistData2>::iterator it_base2;

int before_index = -1;

// 2025-1-1
// 加入

Eigen::Matrix3d K;

ros::NodeHandle this_nh_;
ros::Publisher pub_spline_trajectory_;
ros::Publisher pub_spline_ctrl_;
ros::Publisher pub_spline_ctrl_cloud_;
ros::Publisher pub_spline_active_ctrl_cloud_;
ros::Publisher pub_spline_marg_ctrl_cloud_;
ros::Publisher pub_radar_odom;
ros::Publisher pub_radar_path;

ros::Publisher pub_spline_twist_;

TwistEstimator(ros::NodeHandle& nh)
{
  this_nh_ = nh;

  Init();
}

~TwistEstimator()
{

}

std::map<double, TwistBias> all_twist_bias_;    // 存储的Bias数值

TwistBias GetLatestBias() {
  TwistBias bias;
  bias = all_twist_bias_.rbegin()->second;
  return bias;
}

// Marginazation info [lio system]
MarginalizationInfo::Ptr marg_info;
std::vector<double*> marg_parameter_blocks;
std::pair<int, int> prior_ctrl_id = std::make_pair(0, 0);

// clear the prior after loop closure
void ClearPrior() {
  marg_info = nullptr;
  prior_ctrl_id = std::make_pair(0, 0);
  marg_parameter_blocks.clear();
}

// 状态向量定义
/*
SO3 R
R3  p
R3  bias_v
R3  bias_w
SE3 Tre
R1  tre
*/
double t_add = -1;
void Init()
{
  // 后续可以改为初始化,加入重力估计
  // std::string file = "/home/hao/Desktop/radar-event-new/src/TwistEstimator/config/params.yaml";
  // std::string file = "/home/hao/Desktop/radar-event-new/src/TwistEstimator/config/demo.yaml";
  // std::string file = "/home/hao/Desktop/radar-event-new/src/TwistEstimator/config/demo_228.yaml";
  std::string file = "/home/hao/Desktop/radar-event-new/src/TwistEstimator/config/dji2.yaml";
  ParseYamlFile(file);

  CreateEstimator();

  twist_vec_.reserve(50);
  
  if(t_add < 0)
    t_add = update_every_k_knot_ * trajectory_->getDt();

  std::cout << YELLOW << "\n\t- Update traj every " << t_add << " second\n"
            << RESET;
  LOG(INFO) << "Update traj every " << t_add << " second\n"
              << std::endl;

  LOG(ERROR) << "Update traj every " << t_add << " second\n"
             << " trajectory_->getDt() = " << trajectory_->getDt() << " second\n"
             << std::endl;

  pub_spline_trajectory_ =
      this_nh_.advertise<nav_msgs::Path>("/spline/trajectory", 10);
  pub_spline_ctrl_ = this_nh_.advertise<nav_msgs::Path>("/spline/ctrl_path", 10);
  pub_spline_ctrl_cloud_ =
      this_nh_.advertise<sensor_msgs::PointCloud2>("/spline/ctrl_cloud", 10);
  pub_spline_active_ctrl_cloud_ =
      this_nh_.advertise<sensor_msgs::PointCloud2>("/spline/active_ctrl_cloud", 10);
  pub_spline_marg_ctrl_cloud_ =
      this_nh_.advertise<sensor_msgs::PointCloud2>("/spline/marg_ctrl_cloud", 10);

  pub_spline_marg_ctrl_cloud_ =
      this_nh_.advertise<sensor_msgs::PointCloud2>("/spline/marg_ctrl_cloud", 10);

  pub_spline_twist_ = 
      this_nh_.advertise<geometry_msgs::TwistWithCovarianceStamped>("radar/twist_esti", 1000);

  // all_twist_bias_.reserve(100000); 

  // 不单独指定旋转方向
  // SO3d R0(q);
  // for (size_t i = 0; i <= trajectory_->numKnots(); i++) {
  //   trajectory_->setKnotSO3(R0, i);
  // }

  // 初始化条件
  // 8
  // [8 7]
  // [8 8] 12
  // [9 12]
  // InitPrior();
  // prior_ctrl_id.second = 3;
  // prior_ctrl_id.first = 0;

  q0.setIdentity();
  t0.setZero();

  LOG(INFO) << "Initialized Successfully" << std::endl;

}

/*
[0 1 2 3] 4 5 6 7       8
[4 5 6 7] 8 9 10 11     12
*/
void InitPrior()
{
  prior_ctrl_id.first = twist_trajctory->numKnots() - 2 * update_every_k_knot_;
  prior_ctrl_id.second = twist_trajctory->numKnots() - update_every_k_knot_ - 1;
  LOG(ERROR) << "Init Prior: twist_trajctory->numKnots() = " << twist_trajctory->numKnots() << std::endl;
  LOG(ERROR) << "Init Prior: update_every_k_knot_ = " << update_every_k_knot_ << std::endl;
}

void CreateEstimator()
{
  // 必要的时间说明
  last_time = 0;   // relative start time
  cur_time = last_time + 1e-3;
  // relative_start_time = -1;

  // 初始化偏置列表
  // 浅拷贝
  // all_twist_bias_[last_time].omega_bias = Eigen::Map<Eigen::Vector3d>(para_bw_vec[0]);
  // all_twist_bias_[last_time].vel_bias = Eigen::Map<Eigen::Vector3d>(para_bv_vec[0]);

  // all_twist_bias_[cur_time].omega_bias = Eigen::Map<Eigen::Vector3d>(para_bw_vec[0]);
  // all_twist_bias_[cur_time].vel_bias = Eigen::Map<Eigen::Vector3d>(para_bv_vec[0]);

  // para_bw_vec[1] = Eigen::Vector3d(para_bw_vec[0]);
  // para_bv_vec[1] = Eigen::Vector3d(para_bv_vec[0]);

  all_twist_bias_[cur_time] = all_twist_bias_[last_time];
  // all_twist_bias_[cur_time].omega_bias = Eigen::Vector3d(para_bw_vec[1]);
  // all_twist_bias_[cur_time].vel_bias = Eigen::Vector3d(para_bv_vec[1]);

  time_offset = new double(0.0);

}


void UpdatePrior()
{
  // TODO: 修改
  TrajectoryEstimatorOptions option;

  option.is_marg_state = true;
  option.marg_bias_param = true;  
  // option.marg_gravity_param = true;
  if (opt_time_offset_ && start_opt_time_offset_) {
    option.marg_t_offset_param = false;
  } else {
    option.marg_t_offset_param = true;
  }

  LOG(ERROR) << "Marginasize: ADD I" << std::endl;

  // Splint Control Points Marginized
  // decide the ctrl points to be margalized
  // double opt_min_time = std::min(tparam_.cur_time[0], tparam_.imu_time[0]);s

  //TODO: 检查 last_time 是否有更新
  // double opt_min_time = last_time;

  // 以雷达点云的时间
  double opt_min_time = twist_vec_.begin()->timestamp;
  

  int opt_idx = trajectory_->GetCtrlIndex(opt_min_time * S_TO_NS);
  option.ctrl_to_be_opt_now = std::min(prior_ctrl_id.first, opt_idx);     // HAO TODO: 暂定
  // TODO: 确定一下 prior_ctrl_id

  LOG(ERROR) << "Marginasize: ADD II" << std::endl;

  // TODO: 确定一下 tparam_.cur_time[0] 和 tparam_.cur_time[1]
  double opt_max_time = it_base->timestamp;
  int scan_idx = trajectory_->GetCtrlIndex(opt_max_time * S_TO_NS);
  const int Spline_Order = trajectory_->Get_N();
  option.ctrl_to_be_opt_later = std::max(scan_idx, Spline_Order - 1);     // HAO TODO: 暂定

  LOG(ERROR) << "Marginasize: ADD III" << std::endl;

  LOG(ERROR) << "Marginasize: ctrl idx = [" << option.ctrl_to_be_opt_now << ", " 
             << option.ctrl_to_be_opt_later << "]" << std::endl;

  // UpdatePrior 提前于时间变换,因此还是原来的数据
  // auto& cur_bias = all_twist_bias_[cur_time];
  // auto& last_bias = all_twist_bias_[last_time];

  LOG(ERROR) << "Marginasize: ADD IV" << std::endl;

  // 系统维护状态
  // 保留最近的两个bias数据
  std::map<int, double*> para_bw_vec;
  std::map<int, double*> para_bv_vec;

  // 确保 para_bw_vec 和 para_bv_vec 指针已分配
  // para_bw_vec[0] = all_twist_bias_[last_time].omega_bias.data();
  // para_bv_vec[0] = all_twist_bias_[last_time].vel_bias.data();
  // para_bw_vec[1] = all_twist_bias_[cur_time].omega_bias.data();
  // para_bv_vec[1] = all_twist_bias_[cur_time].vel_bias.data();

  // std::memcpy(para_bw_vec[0], all_twist_bias_[last_time].omega_bias.data(), 3 * sizeof(double));
  // std::memcpy(para_bv_vec[0], all_twist_bias_[last_time].vel_bias.data(), 3 * sizeof(double));
  // std::memcpy(para_bw_vec[1], all_twist_bias_[cur_time].omega_bias.data(), 3 * sizeof(double));
  // std::memcpy(para_bv_vec[1], all_twist_bias_[cur_time].vel_bias.data(), 3 * sizeof(double));

  // 分配内存并深拷贝 omega_bias 数据
  para_bw_vec[0] = new double[3];
  std::memcpy(para_bw_vec[0], all_twist_bias_.at(last_time).omega_bias.data(), 3 * sizeof(double));

  para_bw_vec[1] = new double[3];
  std::memcpy(para_bw_vec[1], all_twist_bias_.at(cur_time).omega_bias.data(), 3 * sizeof(double));

  // 分配内存并深拷贝 vel_bias 数据
  para_bv_vec[0] = new double[3];
  std::memcpy(para_bv_vec[0], all_twist_bias_.at(last_time).vel_bias.data(), 3 * sizeof(double));

  para_bv_vec[1] = new double[3];
  std::memcpy(para_bv_vec[1], all_twist_bias_.at(cur_time).vel_bias.data(), 3 * sizeof(double));

  // 初始化偏置列表
  // 深拷贝 给 bias的时间窗口
  // all_twist_bias_[last_time].omega_bias = Eigen::Vector3d(para_bw_vec[0]);
  // all_twist_bias_[last_time].vel_bias = Eigen::Vector3d(para_bv_vec[0]);

  TrajectoryEstimator::Ptr estimator(
      new TrajectoryEstimator(trajectory_, option, "Update Prior"));

  /// [step0] add marginalization_factor
  LOG(ERROR) << "marg_info = " << ((marg_info)? "True" : "False");
  if (marg_info) {
    std::vector<double*> drop_param_set;
    for (int i = option.ctrl_to_be_opt_now; i < option.ctrl_to_be_opt_later;
         ++i) {
      drop_param_set.emplace_back(trajectory_->getKnotSO3(i).data());
      drop_param_set.emplace_back(trajectory_->getKnotPos(i).data());

      LOG(ERROR) << "marg spline in " << i << std::endl;
    }
    // last bias
    drop_param_set.emplace_back(para_bv_vec[0]);  // in the prior bias
    drop_param_set.emplace_back(para_bw_vec[0]);

    // drop_param_set.emplace_back(last_bias.vel_bias.data());  // in the prior
    // drop_param_set.emplace_back(last_bias.omega_bias.data());

    std::vector<int> drop_set;
    for (int j = 0; j < (int)marg_parameter_blocks.size(); j++) {
      for (auto const& drop_param : drop_param_set) {
        if (marg_parameter_blocks[j] == drop_param) {
          drop_set.emplace_back(j);
          break;
        }
      }
    }
    if (!drop_set.empty()) {
      MarginalizationFactor* marginalization_factor =
          new MarginalizationFactor(marg_info);

      estimator->PrepareMarginalizationInfo(RType_Prior, marginalization_factor,
                                            NULL, marg_parameter_blocks,
                                            drop_set);
      LOG(ERROR) << "Marginasize: Prepare Marginalization Info" << std::endl;
    }
  }
  else
  {
      LOG(ERROR) << "Marginasize: No Marginalization Info" << std::endl;
  }


  /// [step1] add radar doppler features 这里和优化部分是基本一致的,除了加入先验信息
// #ifdef UPDATE_PRIOR
  bool marg_this_factor = true;
  // [1] 多普勒残差
      Eigen::Matrix3d R_r_e = trajectory_->GetSensorEP(EventSensor).q.toRotationMatrix();
      // LOG(ERROR) << " Twist Size = " << (it_base - twist_vec_.begin() + 1) << std::endl;
      // LOG(ERROR) << " Aver Freq = " << 1.0 / (it_base->timestamp - twist_vec_.begin()->timestamp) / (it_base - twist_vec_.begin() + 1) << " Hz" << std::endl;
      for(auto it_temp_ = twist_vec_.begin(); it_temp_ <= it_base; it_temp_++)
      {
        const double twist_timestamp = it_temp_->timestamp;
        {
          double linear_weight = 1.0 / it_temp_->linear_cov.norm();
          // double* vel_bias_ = para_bv_vec[1];

          sensor_msgs::PointCloud2ConstIterator<float> iter_x(it_temp_->point_cloud, "x");
          sensor_msgs::PointCloud2ConstIterator<float> iter_y(it_temp_->point_cloud, "y");
          sensor_msgs::PointCloud2ConstIterator<float> iter_z(it_temp_->point_cloud, "z");
          sensor_msgs::PointCloud2ConstIterator<float> iter_doppler(it_temp_->point_cloud, "doppler");
          
          // std::vector<Eigen::Vector3d> pt_vec;
          // std::vector<double> pt_doppler_vec;
          for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z, ++iter_doppler) {
              //
              Eigen::Vector3d pt;
              pt << *iter_x, *iter_y, *iter_z;
              // 提取当前点的 doppler 值
              float pt_doppler = *iter_doppler;

              // pt_vec.push_back(pt);
              // pt_doppler_vec.push_back(pt_doppler);

            // estimator->AddDopplerMeasurementAnalytic(twist_timestamp, pt, vel_bias_,
            //   pt_doppler, R_r_e, linear_weight, false);    // 边缘化在先验处进行
            estimator->AddDopplerMeasurementAnalytic(twist_timestamp, pt, para_bv_vec[1],
              pt_doppler, R_r_e, linear_weight, option.is_marg_state);    // 边缘化在先验处进行

          }
          // LOG(ERROR) << " Add Doppler Once: " << radar_doppler_num << std::endl;

          // assert(pt_vec.size() == pt_doppler_vec.size() && "PCL Inconsistent Dimensions!");
          // // 改为批处理
          // estimator->AddDopplerMeasurementAnalytic(twist_timestamp, pt_vec, vel_bias_,
          //   pt_doppler_vec, R_r_e, linear_weight, false);    // 边缘化在先验处进行

          // TODO: timestamp is not relative to start time, trajectory_->getLinearBias()
          // LOG(INFO) << " Add Doppler " << std::endl;
          // LOG(ERROR) << " Add Doppler " << std::endl;
        }
        
        // [2] 光流残差
        {
          Eigen::Quaterniond q_e_r = trajectory_->GetSensorEP(EventSensor).q;
          Eigen::Vector3d t_e_r = trajectory_->GetSensorEP(EventSensor).p;
          double omega_weight = 1.0 / (it_temp_->angular_cov.norm() + 1e-2);
          LOG(ERROR) << " before angular_bias: " << std::endl;

          // double* angular_bias_ = para_bw_vec[1];

          LOG(ERROR) << " Add Flow Points: " << it_temp_->best_inliers.size() << std::endl;
          for(auto& in: it_temp_->best_inliers)
          {
            // estimator->AddEventFlowMeasurementAnalytic(twist_timestamp, in, twist.best_flow, twist.linear_vel_vec_,
            //     q_e_r, t_e_r, angular_bias, time_offset, omega_weight, false);
            // estimator->AddEventFlowMeasurementAnalytic(twist_timestamp, in, twist.best_flow, twist.linear_vel_vec_,
            //     q_e_r, t_e_r, angular_bias_, time_offset, omega_weight, false);
            // estimator->AddEventFlowMeasurementAnalytic(twist_timestamp, in, twist.best_flow, twist.linear_vel_vec_,
            //     q_e_r, t_e_r, para_bw_vec[1], time_offset, omega_weight, option.is_marg_state);

            Eigen::Vector3d pixel(in.x, in.y, 1.0);
            Eigen::Vector3d pixel_cord = K.inverse() * pixel;            
            // estimator->AddEventFlowMeasurementAnalytic(twist_timestamp, pixel_cord, twist.best_flow, it_temp_->linear_vel_vec_,
            //     q_e_r, t_e_r, para_bv_vec[1], para_bw_vec[1], time_offset, omega_weight, option.is_marg_state);
          }

          // estimator->AddEventFlowMeasurementAnalytic(twist_timestamp, it_temp_->best_inliers, twist.best_flow, twist.linear_vel_vec_,
          //     q_e_r, t_e_r, angular_bias, time_offset, omega_weight, false);

          // TODO: twist.best_flow 是统一的光流,可以修改为每个事件的光流
          // LOG(INFO) << " Add Event Flow " << std::endl;
          LOG(ERROR) << " Add Event Flow " << std::endl;
        }
      } 

    
    LOG(ERROR) << "Marginasize: Add Event Flow Measurement" << std::endl;

// #endif

  estimator->SaveMarginalizationInfo(marg_info,
                                     marg_parameter_blocks);

  LOG(ERROR) << "Save Marginalization Info " << std::endl;

  // ======= debug output ======= //
  if (marg_info) {
    prior_ctrl_id.first = option.ctrl_to_be_opt_later;
    prior_ctrl_id.second = trajectory_->numKnots() - 1;

    LOG(INFO) << "[After Prior]  marg/left: " << marg_info->m << "/"
              << marg_info->n;
    LOG(INFO) << "[LIO Prior Ctrl ID] = [" << prior_ctrl_id.first << ","
              << prior_ctrl_id.second << "] "
              << prior_ctrl_id.second - prior_ctrl_id.first;
  }

  marg_ctrl_point_.clear();
  for (int i = prior_ctrl_id.first; i < prior_ctrl_id.second; ++i) {
    const auto& p = trajectory_->getKnotPos(i);
    VPoint vp;
    vp.x = p[0];
    vp.y = p[1];
    vp.z = p[2];
    marg_ctrl_point_.push_back(vp);
  }

  LOG(ERROR) << "Marginasize: Update Done!" << std::endl;
}

void UpdatePrior2()
{
  // TODO: 修改
  TrajectoryEstimatorOptions option;

  option.is_marg_state = true;
  option.marg_bias_param = true;  
  // option.marg_gravity_param = true;
  if (opt_time_offset_ && start_opt_time_offset_) {
    option.marg_t_offset_param = false;
  } else {
    option.marg_t_offset_param = true;
  }

  LOG(ERROR) << "Marginasize: ADD I" << std::endl;

  // 以雷达点云的时间
  double opt_min_time = twist2_vec_.begin()->timestamp;
  int opt_idx = trajectory_->GetCtrlIndex(opt_min_time * S_TO_NS);
  option.ctrl_to_be_opt_now = std::min(prior_ctrl_id.first, opt_idx);     // HAO TODO: 暂定
  // TODO: 确定一下 prior_ctrl_id
  LOG(ERROR) << "Marginasize: option.ctrl_to_be_opt_now = " << option.ctrl_to_be_opt_now << std::endl;
  // LOG(ERROR) << "Marginasize: ADD II" << std::endl;

  // TODO: 确定一下 tparam_.cur_time[0] 和 tparam_.cur_time[1]
  double opt_max_time = it_base2->timestamp;
  // LOG(ERROR) << "Marginasize: opt_max_time = " << opt_max_time << std::endl;
  int scan_idx = trajectory_->GetCtrlIndex(opt_max_time * S_TO_NS);
  const int Spline_Order = trajectory_->Get_N();
  option.ctrl_to_be_opt_later = std::max(scan_idx, Spline_Order - 1);     // HAO TODO: 暂定
  LOG(ERROR) << "Marginasize: option.ctrl_to_be_opt_later = " << option.ctrl_to_be_opt_later << std::endl;

  // LOG(ERROR) << "Marginasize: ADD III" << std::endl;

  LOG(ERROR) << "Marginasize: ctrl idx = [" << option.ctrl_to_be_opt_now << ", " 
             << option.ctrl_to_be_opt_later << "]" << std::endl;

  LOG(ERROR) << "Marginasize: ADD IV" << std::endl;

  // 系统维护状态
  // 保留最近的两个bias数据
  std::map<int, double*> para_bw_vec;
  std::map<int, double*> para_bv_vec;

  // 分配内存并深拷贝 omega_bias 数据
  para_bw_vec[0] = new double[3];
  std::memcpy(para_bw_vec[0], all_twist_bias_.at(last_time).omega_bias.data(), 3 * sizeof(double));

  para_bw_vec[1] = new double[3];
  std::memcpy(para_bw_vec[1], all_twist_bias_.at(cur_time).omega_bias.data(), 3 * sizeof(double));

  // 分配内存并深拷贝 vel_bias 数据
  para_bv_vec[0] = new double[3];
  std::memcpy(para_bv_vec[0], all_twist_bias_.at(last_time).vel_bias.data(), 3 * sizeof(double));

  para_bv_vec[1] = new double[3];
  std::memcpy(para_bv_vec[1], all_twist_bias_.at(cur_time).vel_bias.data(), 3 * sizeof(double));

  TrajectoryEstimator::Ptr estimator(
      new TrajectoryEstimator(trajectory_, option, "Update Prior2"));

  /// [step0] add marginalization_factor
  LOG(ERROR) << "marg_info = " << ((marg_info)? "True" : "False");
  if (marg_info) {
    std::vector<double*> drop_param_set;
    for (int i = option.ctrl_to_be_opt_now; i < option.ctrl_to_be_opt_later;
         ++i) {
      drop_param_set.emplace_back(trajectory_->getKnotSO3(i).data());
      drop_param_set.emplace_back(trajectory_->getKnotPos(i).data());

      LOG(ERROR) << "marg spline in " << i << std::endl;
    }
    // last bias
    drop_param_set.emplace_back(para_bv_vec[0]);  // in the prior bias
    drop_param_set.emplace_back(para_bw_vec[0]);

    std::vector<int> drop_set;
    for (int j = 0; j < (int)marg_parameter_blocks.size(); j++) {
      for (auto const& drop_param : drop_param_set) {
        if (marg_parameter_blocks[j] == drop_param) {
          // LOG(ERROR) << "drop set " << std::endl;
          drop_set.emplace_back(j);
          break;
        }
      }
    }
    if (!drop_set.empty()) {
      MarginalizationFactor* marginalization_factor =
          new MarginalizationFactor(marg_info);

      estimator->PrepareMarginalizationInfo(RType_Prior, marginalization_factor,
                                            NULL, marg_parameter_blocks,
                                            drop_set);
      LOG(ERROR) << "Marginasize: Prepare Marginalization Info" << std::endl;
      LOG(ERROR) << "Marginasize: Add Marginalization Size " << drop_set.size() << std::endl;
    }
  }
  else
    LOG(ERROR) << "Marginasize: No Marginalization Info" << std::endl;



  /// [step1] add radar doppler features 这里和优化部分是基本一致的,除了加入先验信息
  // 评估本次优化的误差,加入下一次先验
  bool marg_this_factor = true;
  // [1] 多普勒残差
  Eigen::Matrix3d R_r_e = trajectory_->GetSensorEP(EventSensor).q.toRotationMatrix();
  for(auto it_temp_ = twist2_vec_.begin(); it_temp_ <= it_base2; it_temp_++)
  {
    const double twist_timestamp = it_temp_->timestamp;
    {
      // double linear_weight = 1.0 / it_temp_->linear_cov.norm();
      // 1-11 修改
      double linear_weight = 1.0;

      sensor_msgs::PointCloud2ConstIterator<float> iter_x(it_temp_->point_cloud, "x");
      sensor_msgs::PointCloud2ConstIterator<float> iter_y(it_temp_->point_cloud, "y");
      sensor_msgs::PointCloud2ConstIterator<float> iter_z(it_temp_->point_cloud, "z");
      sensor_msgs::PointCloud2ConstIterator<float> iter_doppler(it_temp_->point_cloud, "doppler");
      
      // std::vector<Eigen::Vector3d> pt_vec;
      // std::vector<double> pt_doppler_vec;
      for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z, ++iter_doppler) {
          //
          Eigen::Vector3d pt;
          pt << *iter_x, *iter_y, *iter_z;
          // 提取当前点的 doppler 值
          float pt_doppler = *iter_doppler;

          estimator->AddDopplerMeasurementAnalytic(twist_timestamp, pt, para_bv_vec[1],
            pt_doppler, R_r_e, linear_weight, option.is_marg_state);    // 边缘化在先验处进行
      }

      LOG(ERROR) << "Marginasize: Add Doppler " << std::endl;
    }
      
    // [2] 光流残差
    {
      Eigen::Quaterniond q_e_r = trajectory_->GetSensorEP(EventSensor).q;
      Eigen::Vector3d t_e_r = trajectory_->GetSensorEP(EventSensor).p;
      // 1-11 修改
      double omega_weight = 1.0;
      // double omega_weight = 1.0 / (it_temp_->angular_cov.norm() + 1e-2);

      // LOG(ERROR) << " before angular_bias: " << std::endl;

      // double* angular_bias_ = para_bw_vec[1];

      // LOG(ERROR) << " Add Flow Points: " << it_temp_->best_inliers.size() << std::endl;
      // for(auto& in: it_temp_->best_inliers)
      for(int i = 0; i < it_temp_->best_inliers.size(); i++)
      {
        Eigen::Vector3d pixel(it_temp_->best_inliers[i].x, it_temp_->best_inliers[i].y, 1.0);
        Eigen::Vector3d pixel_cord = K.inverse() * pixel;         
        estimator->AddEventFlowMeasurementAnalytic(twist_timestamp, pixel_cord, it_temp_->best_flow[i], it_temp_->linear_vel_vec_,
            q_e_r, t_e_r, para_bv_vec[1], para_bw_vec[1], time_offset, omega_weight, option.is_marg_state);
      }
      LOG(ERROR) << "Marginasize: Add Event Flow " << std::endl;
    }
  } 
  // LOG(ERROR) << "Marginasize: Add Event Flow Measurement" << std::endl;

  estimator->SaveMarginalizationInfo(marg_info,
                                     marg_parameter_blocks);
  LOG(ERROR) << "[After Prior]  marg/left: " << marg_info->m << "/"
          << marg_info->n;

  LOG(ERROR) << "Save Marginalization Info " << std::endl;

  // ======= debug output ======= //
  /*if (marg_info) {
    prior_ctrl_id.first = option.ctrl_to_be_opt_later;
    prior_ctrl_id.second = trajectory_->numKnots() - 1;

    LOG(INFO) << "[After Prior]  marg/left: " << marg_info->m << "/"
              << marg_info->n;
    LOG(INFO) << "[LIO Prior Ctrl ID] = [" << prior_ctrl_id.first << ","
              << prior_ctrl_id.second << "] "
              << prior_ctrl_id.second - prior_ctrl_id.first + 1;
  }*/

  // marg_ctrl_point_.clear();
  // for (int i = prior_ctrl_id.first; i < prior_ctrl_id.second; ++i) {
  //   const auto& p = trajectory_->getKnotPos(i);
  //   VPoint vp;
  //   vp.x = p[0];
  //   vp.y = p[1];
  //   vp.z = p[2];
  //   marg_ctrl_point_.push_back(vp);
  // }

  // LOG(ERROR) << "Marginasize: Update Done!" << std::endl;
}


void UpdatePrior3()
{
  // TODO: 修改
  TrajectoryEstimatorOptions option;

  option.is_marg_state = true;
  option.marg_bias_param = true;  
  // option.marg_gravity_param = true;
  if (opt_time_offset_ && start_opt_time_offset_) {
    option.marg_t_offset_param = false;
  } else {
    option.marg_t_offset_param = true;
  }
  LOG(ERROR) << "Marginasize: ADD I" << std::endl;

  // 以雷达点云的时间
  double opt_min_time = twist2_vec_.begin()->timestamp;
  int opt_idx = twist_trajctory->GetCtrlIndex(opt_min_time * S_TO_NS);
  option.ctrl_to_be_opt_now = std::min(prior_ctrl_id.first, opt_idx);     // HAO TODO: 暂定

  int KnotsNum = twist_trajctory->getKnotSize();
  int SplineOrder = twist_trajctory->Get_N();
  // option.ctrl_to_be_opt_now = KnotsNum - 3 * SplineOrder;
  option.ctrl_to_be_opt_now = KnotsNum - 3 * SplineOrder + 1;
  
  // TODO: 确定一下 prior_ctrl_id
  // LOG(ERROR) << "Marginasize: option.ctrl_to_be_opt_now = " << option.ctrl_to_be_opt_now << std::endl;
  // LOG(ERROR) << "Marginasize: ADD II" << std::endl;

  // TODO: 确定一下 tparam_.cur_time[0] 和 tparam_.cur_time[1]
  double opt_max_time = it_base2->timestamp;
  // LOG(ERROR) << "Marginasize: opt_max_time = " << opt_max_time << std::endl;
  int scan_idx = twist_trajctory->GetCtrlIndex(opt_max_time * S_TO_NS);
  const int Spline_Order = twist_trajctory->Get_N();
  // option.ctrl_to_be_opt_later = std::max(scan_idx, Spline_Order - 1);     // HAO TODO: 暂定
  // option.ctrl_to_be_opt_later = KnotsNum - 2 * SplineOrder - 1; 
  option.ctrl_to_be_opt_later = KnotsNum - 2 * SplineOrder; 
  // LOG(ERROR) << "Marginasize: option.ctrl_to_be_opt_later = " << option.ctrl_to_be_opt_later << std::endl;

  // LOG(ERROR) << "Marginasize: ADD III" << std::endl;

  LOG(ERROR) << "Marginasize: ctrl idx = [" << option.ctrl_to_be_opt_now << ", " 
             << option.ctrl_to_be_opt_later << "]" << std::endl;

  LOG(ERROR) << "Marginasize: ADD IV" << std::endl;

  // 系统维护状态
  // 保留最近的两个bias数据
  std::map<int, double*> para_bw_vec;
  std::map<int, double*> para_bv_vec;

  // 分配内存并深拷贝 omega_bias 数据
  para_bw_vec[0] = new double[3];
  std::memcpy(para_bw_vec[0], all_twist_bias_.at(last_time).omega_bias.data(), 3 * sizeof(double));

  para_bw_vec[1] = new double[3];
  std::memcpy(para_bw_vec[1], all_twist_bias_.at(cur_time).omega_bias.data(), 3 * sizeof(double));

  // 分配内存并深拷贝 vel_bias 数据
  para_bv_vec[0] = new double[3];
  std::memcpy(para_bv_vec[0], all_twist_bias_.at(last_time).vel_bias.data(), 3 * sizeof(double));

  para_bv_vec[1] = new double[3];
  std::memcpy(para_bv_vec[1], all_twist_bias_.at(cur_time).vel_bias.data(), 3 * sizeof(double));

 TrajectoryEstimator2::Ptr estimator(new TrajectoryEstimator2(twist_trajctory, option, "Update Prior3"));

  /// [step0] add marginalization_factor
  LOG(ERROR) << "marg_info = " << ((marg_info)? "True" : "False");
  if (marg_info) {
    std::vector<double*> drop_param_set;
    for (int i = option.ctrl_to_be_opt_now; i < option.ctrl_to_be_opt_later;
         ++i) {
      drop_param_set.emplace_back(twist_trajctory->GetAngularSpline().getKnot(i).data());
      drop_param_set.emplace_back(twist_trajctory->GetLinearSpline().getKnot(i).data());

      // LOG(ERROR) << "marg spline in " << i << std::endl;
    }
    // last bias
    // drop_param_set.emplace_back(para_bv_vec[0]);  // in the prior bias
    // drop_param_set.emplace_back(para_bw_vec[0]);

    std::vector<int> drop_set;
    for (int j = 0; j < (int)marg_parameter_blocks.size(); j++) {
      for (auto const& drop_param : drop_param_set) {
        if (marg_parameter_blocks[j] == drop_param) {
          // LOG(ERROR) << "drop set " << std::endl;
          drop_set.emplace_back(j);
          break;
        }
      }
    }
    LOG(ERROR) << "drop_set = " << drop_set.size() << std::endl;
    if (!drop_set.empty()) {
      MarginalizationFactor* marginalization_factor =
          new MarginalizationFactor(marg_info);

      estimator->PrepareMarginalizationInfo(RType_Prior, marginalization_factor,
                                            NULL, marg_parameter_blocks,
                                            drop_set);
      LOG(ERROR) << "Marginasize: Prepare Marginalization Info" << std::endl;
      LOG(ERROR) << "Marginasize: Add Marginalization Size " << drop_set.size() << std::endl;
    }
  }
  else
    LOG(ERROR) << "Marginasize: No Marginalization Info" << std::endl;

     /// [step1] add radar doppler features 这里和优化部分是基本一致的,除了加入先验信息
      // 评估本次优化的误差,加入下一次先验
      bool marg_this_factor = true;
      // [1] 多普勒残差
      Eigen::Matrix3d R_r_e = twist_trajctory->GetSensorEP(EventSensor).q.toRotationMatrix();
      for(auto it_temp_ = twist2_vec_.begin(); it_temp_ <= it_base2; it_temp_++)
      {
        // [step5-1] 多普勒残差
        const double twist_timestamp = it_temp_->timestamp;
        // LOG(ERROR) << "twist time = " << twist_timestamp << ", maxTime = " << trajectory_->maxTime(RadarSensor) << " dt = " << trajectory_->getDt() << std::endl;
        // LOG(ERROR) << "twist in " << (int)((twist_timestamp - last_max_time) / trajectory_->getDt());
        // if(last_max_time > twist_timestamp)
        //   continue;

        int idx = twist_trajctory->GetCtrlIndex(twist_timestamp * S_TO_NS);
        LOG(ERROR) << "idx = " << idx << std::endl;

        // LOG(ERROR) << "twist time = " << twist_timestamp << ", maxTime = " << twist_trajctory->maxTime(RadarSensor) << " dt = " << trajectory_->getDt() << std::endl;
        // LOG(ERROR) << "twist in " << (int)((twist_timestamp - last_max_time) / twist_trajctory->getDt());

        Eigen::Quaterniond q_e_r = twist_trajctory->GetSensorEP(EventSensor).q;
        Eigen::Vector3d t_e_r = twist_trajctory->GetSensorEP(EventSensor).p;
        // double omega_weight = 1.0 / (it_temp_->angular_cov.norm() + 1e-2);
        // LOG(ERROR) << " before angular_bias: " << std::endl;

        // LOG(ERROR) << "it_temp_->linear_vel_vec_ = " << it_temp_->linear_vel_vec_.transpose() << std::endl;
        // estimator->AddBodyLocalVelocityMeasurementAnalytic(twist_timestamp, para_bv_vec[1], 
        //             it_temp_->linear_vel_vec_, linear_weight, linear_w_weight, R_weight, false);

        // double* angular_bias_ = para_bw_vec[1];
        // event_flow_factors_num += it_temp_->best_inliers.size();
        // LOG(ERROR) << " Add Flow Points: " << it_temp_->best_inliers.size() << std::endl;
        for(int i = 0; i < it_temp_->best_inliers.size(); i++)
        {
          Eigen::Vector3d pixel(it_temp_->best_inliers[i].x, it_temp_->best_inliers[i].y, 1.0);
          Eigen::Vector3d pixel_cord = K.inverse() * pixel;         
          // estimator->AddEventFlowMeasurementAnalytic2(twist_timestamp, pixel_cord, it_temp_->best_flow[i], it_temp_->linear_vel_vec_,
          //     q_e_r, t_e_r, para_bv_vec[1], para_bw_vec[1], time_offset, omega_weight, omega_w_weight, false);
          // estimator->AddEventFlowMeasurementAnalytic3(twist_timestamp, pixel_cord, it_temp_->best_flow[i], it_temp_->linear_vel_vec_,
          //     q_e_r, t_e_r, para_bv_vec[1], para_bw_vec[1], time_offset, global_fej_state_, use_fej, 
          //     omega_weight, omega_w_weight, marg_this_factor);
          estimator->AddEventFlowMeasurementAnalytic3(twist_timestamp, pixel_cord, it_temp_->best_flow[i], it_temp_->linear_vel_vec_,
              q_e_r, t_e_r, para_bv_vec[1], para_bw_vec[1], time_offset, global_fej_state_, use_fej, 
              use_order_opti, omega_weight, omega_w_weight, marg_this_factor);

        }
        LOG(ERROR) << "Marginasize: Add Event Flow " << std::endl;
      }
  // LOG(ERROR) << "Marginasize: Add Event Flow Measurement" << std::endl;

  estimator->SaveMarginalizationInfo(marg_info,
                                     marg_parameter_blocks);
  if(marg_info!=nullptr)
    LOG(ERROR) << "[After Prior]  marg/left: " << marg_info->m << "/"
            << marg_info->n;

  LOG(ERROR) << "Save Marginalization Info " << std::endl;
}

  // ======= debug output ======= //
  /*if (marg_info) {
    prior_ctrl_id.first = option.ctrl_to_be_opt_later;
    prior_ctrl_id.second = trajectory_->numKnots() - 1;

    LOG(INFO) << "[After Prior]  marg/left: " << marg_info->m << "/"
              << marg_info->n;
    LOG(INFO) << "[LIO Prior Ctrl ID] = [" << prior_ctrl_id.first << ","
              << prior_ctrl_id.second << "] "
              << prior_ctrl_id.second - prior_ctrl_id.first + 1;
  }*/

  // marg_ctrl_point_.clear();
  // for (int i = prior_ctrl_id.first; i < prior_ctrl_id.second; ++i) {
  //   const auto& p = trajectory_->getKnotPos(i);
  //   VPoint vp;
  //   vp.x = p[0];
  //   vp.y = p[1];
  //   vp.z = p[2];
  //   marg_ctrl_point_.push_back(vp);
  // }

  // LOG(ERROR) << "Marginasize: Update Done!" << std::endl;


void UpdatePriorLoose()
{
  // TODO: 修改
  TrajectoryEstimatorOptions option;

  option.is_marg_state = true;
  option.marg_bias_param = true;  
  // option.marg_gravity_param = true;
  if (opt_time_offset_ && start_opt_time_offset_) {
    option.marg_t_offset_param = false;
  } else {
    option.marg_t_offset_param = true;
  }

  LOG(ERROR) << "Marginasize: ADD I" << std::endl;

  // 以雷达点云的时间
  double opt_min_time = twist2_vec_.begin()->timestamp;
  int opt_idx = trajectory_->GetCtrlIndex(opt_min_time * S_TO_NS);
  option.ctrl_to_be_opt_now = std::min(prior_ctrl_id.first, opt_idx);     // HAO TODO: 暂定
  // TODO: 确定一下 prior_ctrl_id
  LOG(ERROR) << "Marginasize: option.ctrl_to_be_opt_now = " << option.ctrl_to_be_opt_now << std::endl;
  // LOG(ERROR) << "Marginasize: ADD II" << std::endl;

  // TODO: 确定一下 tparam_.cur_time[0] 和 tparam_.cur_time[1]
  double opt_max_time = it_base2->timestamp;
  // LOG(ERROR) << "Marginasize: opt_max_time = " << opt_max_time << std::endl;
  int scan_idx = trajectory_->GetCtrlIndex(opt_max_time * S_TO_NS);
  const int Spline_Order = trajectory_->Get_N();
  option.ctrl_to_be_opt_later = std::max(scan_idx, Spline_Order - 1);     // HAO TODO: 暂定
  LOG(ERROR) << "Marginasize: option.ctrl_to_be_opt_later = " << option.ctrl_to_be_opt_later << std::endl;

  // LOG(ERROR) << "Marginasize: ADD III" << std::endl;

  LOG(ERROR) << "Marginasize: ctrl idx = [" << option.ctrl_to_be_opt_now << ", " 
             << option.ctrl_to_be_opt_later << "]" << std::endl;

  LOG(ERROR) << "Marginasize: ADD IV" << std::endl;

  // 系统维护状态
  // 保留最近的两个bias数据
  std::map<int, double*> para_bw_vec;
  std::map<int, double*> para_bv_vec;

  // 分配内存并深拷贝 omega_bias 数据
  para_bw_vec[0] = new double[3];
  std::memcpy(para_bw_vec[0], all_twist_bias_.at(last_time).omega_bias.data(), 3 * sizeof(double));

  para_bw_vec[1] = new double[3];
  std::memcpy(para_bw_vec[1], all_twist_bias_.at(cur_time).omega_bias.data(), 3 * sizeof(double));

  // 分配内存并深拷贝 vel_bias 数据
  para_bv_vec[0] = new double[3];
  std::memcpy(para_bv_vec[0], all_twist_bias_.at(last_time).vel_bias.data(), 3 * sizeof(double));

  para_bv_vec[1] = new double[3];
  std::memcpy(para_bv_vec[1], all_twist_bias_.at(cur_time).vel_bias.data(), 3 * sizeof(double));

  TrajectoryEstimator::Ptr estimator(
      new TrajectoryEstimator(trajectory_, option, "Update Prior2"));

  /// [step0] add marginalization_factor
  LOG(ERROR) << "marg_info = " << ((marg_info)? "True" : "False");
  if (marg_info) {
    std::vector<double*> drop_param_set;
    for (int i = option.ctrl_to_be_opt_now; i < option.ctrl_to_be_opt_later;
         ++i) {
      drop_param_set.emplace_back(trajectory_->getKnotSO3(i).data());
      drop_param_set.emplace_back(trajectory_->getKnotPos(i).data());

      LOG(ERROR) << "marg spline in " << i << std::endl;
    }
    // last bias
    drop_param_set.emplace_back(para_bv_vec[0]);  // in the prior bias
    drop_param_set.emplace_back(para_bw_vec[0]);

    std::vector<int> drop_set;
    for (int j = 0; j < (int)marg_parameter_blocks.size(); j++) {
      for (auto const& drop_param : drop_param_set) {
        if (marg_parameter_blocks[j] == drop_param) {
          // LOG(ERROR) << "drop set " << std::endl;
          drop_set.emplace_back(j);
          break;
        }
      }
    }
    if (!drop_set.empty()) {
      MarginalizationFactor* marginalization_factor =
          new MarginalizationFactor(marg_info);

      estimator->PrepareMarginalizationInfo(RType_Prior, marginalization_factor,
                                            NULL, marg_parameter_blocks,
                                            drop_set);
      LOG(ERROR) << "Marginasize: Prepare Marginalization Info" << std::endl;
      LOG(ERROR) << "Marginasize: Add Marginalization Size " << drop_set.size() << std::endl;
    }
  }
  else
    LOG(ERROR) << "Marginasize: No Marginalization Info" << std::endl;



  /// [step1] add radar doppler features 这里和优化部分是基本一致的,除了加入先验信息
  // 评估本次优化的误差,加入下一次先验
  bool marg_this_factor = true;
  // [1] 多普勒残差
  Eigen::Matrix3d R_r_e = trajectory_->GetSensorEP(EventSensor).q.toRotationMatrix();

  for(auto it_temp_ = twist2_vec_.begin(); it_temp_ <= it_base2; it_temp_++)
  {
    // [step5-1] 多普勒残差
    const double twist_timestamp = it_temp_->timestamp;
    // LOG(ERROR) << "twist time = " << twist_timestamp << ", maxTime = " << trajectory_->maxTime(RadarSensor) << " dt = " << trajectory_->getDt() << std::endl;
    // LOG(ERROR) << "twist in " << (int)((twist_timestamp - last_max_time) / trajectory_->getDt());
    {
      
      estimator->AddBodyLocalVelocityMeasurementAnalytic(twist_timestamp, para_bv_vec[1], 
                                                      it_temp_->linear_vel_vec_, linear_weight, linear_w_weight, R_weight, true);

      estimator->AddBodyLocalAngularVelocityMeasurementAnalytic(twist_timestamp, para_bw_vec[1], 
                                                      it_temp_-> angular_vel_vec_, omega_weight, omega_w_weight, true);                                            
    }

  }

  /*{
    double delta_time = cur_time - last_time;
    // double dt = 1. / 200.; // opt_weight_.imu_noise.imu_frequency;
    // double dt = 1. / 10.; // opt_weight_.radar_frequency;
    double dt = 1. / 20.; // opt_weight_.radar_frequency;
    double cov = delta_time / dt * (dt * dt);
    Eigen::Matrix<double, 6, 1> sqrt_info;
    sqrt_info.setOnes();
    sqrt_info *=  (1. / std::sqrt(cov)); // * opt_weight_.bias_info_vec;

    LOG(ERROR) << "100 * sqrt_info = " << 100 * sqrt_info << std::endl;

    estimator->AddBiasFactor(para_bw_vec[0], para_bw_vec[1], 
                              para_bv_vec[0], para_bv_vec[1], 
                              1, 100 * sqrt_info, true); // 100 * sqrt_info);

    LOG(ERROR) << " Add Bias Factor " << std::endl;                          
}*/
  // LOG(ERROR) << "Marginasize: Add Event Flow Measurement" << std::endl;

  estimator->SaveMarginalizationInfo(marg_info,
                                     marg_parameter_blocks);
  LOG(ERROR) << "[After Prior]  marg/left: " << marg_info->m << "/"
          << marg_info->n;

  LOG(ERROR) << "Save Marginalization Info " << std::endl;

  // ======= debug output ======= //
  /*if (marg_info) {
    prior_ctrl_id.first = option.ctrl_to_be_opt_later;
    prior_ctrl_id.second = trajectory_->numKnots() - 1;

    LOG(INFO) << "[After Prior]  marg/left: " << marg_info->m << "/"
              << marg_info->n;
    LOG(INFO) << "[LIO Prior Ctrl ID] = [" << prior_ctrl_id.first << ","
              << prior_ctrl_id.second << "] "
              << prior_ctrl_id.second - prior_ctrl_id.first + 1;
  }*/

  // marg_ctrl_point_.clear();
  // for (int i = prior_ctrl_id.first; i < prior_ctrl_id.second; ++i) {
  //   const auto& p = trajectory_->getKnotPos(i);
  //   VPoint vp;
  //   vp.x = p[0];
  //   vp.y = p[1];
  //   vp.z = p[2];
  //   marg_ctrl_point_.push_back(vp);
  // }

  // LOG(ERROR) << "Marginasize: Update Done!" << std::endl;
}

//进入运行
void Estimate(TwistData& twist)
{
  LOG(INFO) << "  ----------------- Estimate -------------------- " << std::endl;

  // 设置系统初始时间
  if (trajectory_->GetDataStartTime() < 0) {
    trajectory_->SetDataStartTime(twist.timestamp);
    relative_start_time = twist.timestamp;

    std::cout << YELLOW << " trajectory set start time: " << std::setprecision(18) << twist.timestamp << std::endl;
    LOG(INFO) << " trajectory set start time:  " << std::setprecision(18) << twist.timestamp << std::endl;
  }

  // 转换数据时间
  double data_start_time = trajectory_->GetDataStartTime();
  if(!twist.is_relative_time)
  {
    twist.timestamp -= relative_start_time; //转换到相对时间
    twist.is_relative_time = true;
  }
  
  // 存储到一个vector中,方便查找
  twist_vec_.push_back(twist);

  assert(twist.is_relative_time);

  // std::cout << " twist_vec_ add new data " << std::endl;

  // 设置优化时间 traj_max_time ,判断是否满足时间
  double traj_max_time = trajectory_->maxTime(RadarSensor) + t_add; // 单位: 秒
  LOG(ERROR) << "last traj_max_time = " << trajectory_->maxTime(RadarSensor) << std::endl;
  LOG(ERROR) << "traj_max_time = " << traj_max_time << std::endl;

  if(twist_vec_.back().timestamp < traj_max_time)
    return;
  // else if(twist_vec_.front().timestamp < traj_max_time){
  //   LOG(ERROR) << "not valid twist data to update trajctory in [" 
  //               << trajectory_->maxTime(RadarSensor) << "," << traj_max_time << "]" << std::endl;
  //   return;
  // }

  // LOG(ERROR) << " traj_max_time = " << traj_max_time << std::endl;

  // LOG(ERROR) << " extend time to = " << std::setprecision(9) << traj_max_time << std::endl;
  // LOG(ERROR) << "twist data size = " << std::setprecision(20) << twist_vec_.size() << std::endl;

  // 查询时间戳恰好小于更新时间的速度数据 (it it_base 指向同一个元素)
  auto it = std::find_if(twist_vec_.rbegin(), twist_vec_.rend(), 
                        [traj_max_time](const TwistData& data) {
                            return data.timestamp < traj_max_time;
                        });
  // auto  做成了成员变量
  it_base = ((it == twist_vec_.rend())? twist_vec_.begin() : it.base() - 1); // it 的正向跌代器

  assert(it->timestamp == it_base->timestamp && "Not Same Place");

  // 更新下一个时间窗口
  // cur_time是相对时间
  cur_time = it_base->timestamp;

  // LOG(ERROR) << " actual extend time to = " << std::setprecision(9) << cur_time << std::endl;

  // 偏置顺延
  // LOG(ERROR) << "initial twist bias" << std::endl;
  all_twist_bias_[cur_time] = all_twist_bias_[last_time];

  // 开始正常优化
  // HAO TODO:
  // 需要区分 traj_max_time 是划分数据的时间,而不是优化维护的时间 (后面优化也不用)
  // 需要区分 update_time 是优化维护的时间 (后面在这个时间段中进行优化)
  double update_time = it->timestamp;  // 注意是相对时间的 秒
  SE3d last_knot = trajectory_->getLastKnot();
  LOG(ERROR) << "CHECK max time 1 = " << trajectory_->maxTime(RadarSensor) << std::endl;
  trajectory_->extendKnotsTo(it->timestamp * S_TO_NS, last_knot); // 扩展到最新的数据
  LOG(ERROR) << "CHECK max time 2 = " << trajectory_->maxTime(RadarSensor) << std::endl;

  // Extract translation and rotation
  // Eigen::Vector3d translation = last_knot.translation();
  // Eigen::Quaterniond quaternion(last_knot.rotationMatrix());
  // LOG(ERROR) << "TEST: last_knot =[[" << quaternion.coeffs().transpose() 
  //             << "], [" << translation.transpose() << "]]" << std::endl;

  LOG(ERROR) << " trajectory_ extend knots timestamp: " << std::setprecision(9) << it->timestamp << std::endl;
  LOG(ERROR) << " check update timestamp: " << std::setprecision(9) << it->timestamp - traj_max_time << std::endl;  // should be <
  // 如果下一个存在,检查一下下一个的时间戳
  if(it_base != twist_vec_.end() && (it_base + 1) != twist_vec_.end())
  {
    LOG(ERROR) << " trajectory_ extend knots timestamp: " << std::setprecision(9) << (it + 1)->timestamp << std::endl;
    LOG(ERROR) << " check update timestamp: " << std::setprecision(9) << (it - 1)->timestamp - traj_max_time << std::endl;
  }

  // LOG(INFO) << " check update timestamp: " << it->timestamp - traj_max_time << std::endl;

  // auto check_it = it_base;
  // while((check_it++ ) != twist_vec_.end())
  // {
  //   LOG(INFO) << " update index: " << (check_it - twist_vec_.begin()) / (twist_vec_.end() - twist_vec_.begin())  << std::endl;
  //   LOG(INFO) << " update timestamp: " << check_it->timestamp - traj_max_time << std::endl;
  // }

  // LOG(INFO) << "end set timestamp: " << it_base->timestamp << std::endl;
  // LOG(INFO) << "trajectory max timestamp: " << traj_max_time << std::endl;
  // for(auto tw : twist_vec_)
  // {
  //   LOG(INFO) << "check timestamp: " << tw.timestamp << std::endl;
  // }

  // 后面均使用 para_bw_vec 和 para_bv_vec 参与优化
  // para_bw_vec[0] = all_twist_bias_.at(last_time).omega_bias.data();
  // para_bw_vec[1] = all_twist_bias_.at(cur_time).omega_bias.data();

  // para_bv_vec[0] = all_twist_bias_.at(last_time).vel_bias.data();
  // para_bv_vec[1] = all_twist_bias_.at(cur_time).vel_bias.data();
  // 确保 para_bw_vec 和 para_bv_vec 的内存分配正确
  std::map<int, double*> para_bw_vec;
  std::map<int, double*> para_bv_vec;
  // if (para_bw_vec[0] != nullptr) {
  //     delete[] para_bw_vec[0];
  // }
  // if (para_bw_vec[1] != nullptr) {
  //     delete[] para_bw_vec[1];
  // }
  // if (para_bv_vec[0] != nullptr) {
  //     delete[] para_bv_vec[0];
  // }
  // if (para_bv_vec[1] != nullptr) {
  //     delete[] para_bv_vec[1];
  // }

  // 分配内存并深拷贝 omega_bias 数据
  para_bw_vec[0] = new double[3];
  std::memcpy(para_bw_vec[0], all_twist_bias_.at(last_time).omega_bias.data(), 3 * sizeof(double));

  para_bw_vec[1] = new double[3];
  std::memcpy(para_bw_vec[1], all_twist_bias_.at(cur_time).omega_bias.data(), 3 * sizeof(double));

  // 分配内存并深拷贝 vel_bias 数据
  para_bv_vec[0] = new double[3];
  std::memcpy(para_bv_vec[0], all_twist_bias_.at(last_time).vel_bias.data(), 3 * sizeof(double));

  para_bv_vec[1] = new double[3];
  std::memcpy(para_bv_vec[1], all_twist_bias_.at(cur_time).vel_bias.data(), 3 * sizeof(double));

  // LOG(ERROR) << "Estimator: " 
  //            << "\npara_bw_vec[0] address at: " << para_bw_vec[0]
  //            << "\npara_bw_vec[1] address at: " << para_bw_vec[1]
  //            << "\npara_bv_vec[0] address at: " << para_bv_vec[0]
  //            << "\npara_bv_vec[1] address at: " << para_bv_vec[1]
  //            << std::endl;             

  TrajectoryEstimatorOptions option;
  option.lock_EPs.at(EventSensor).Unlock();

  // option.is_marg_state = true;
  // option.marg_t_offset_param = false;
  // option.lock_EPs.at(EventSensor).lock_t_offset = true;
  // option.ctrl_to_be_opt_now = 0;  // idx
  // option.ctrl_to_be_opt_later = prior_ctrl_id.first;
    
  option.is_marg_state = false;
  option.show_residual_summary = false;
  TrajectoryEstimator::Ptr estimator(new TrajectoryEstimator(trajectory_, option, "Update Traj"));

  // LOG(INFO) << " start estimate " << std::endl;
  LOG(ERROR) << " start estimate " << std::endl;

  // 获取估计曲线末端,按照时间扩展曲线控制点,不更新时间
  // [opt_min_time, prior_ctrl_id.first]   
  // max_time_ns 是NS单位,应该是事件和雷达融合数据的时间戳
  {
    // LOG(INFO) << "[InitTrajWithPropagation] traj involved: ["
    // << trajectory_->GetCtrlIndex(opt_min_time * S_TO_NS) << ","
    // << trajectory_->numKnots() << "]";

    /*
      int idx = trajectory_->GetCtrlIndex(tparam_.cur_time[0] * S_TO_NS);
      double t_min = trajectory_->minTime(RadarSensor) + trajectory_->getDt() * idx;
      if (t_min < 0) t_min = 0;
      tparam_.imu_time[0] = iter->timestamp + t_offset_imu;
      double opt_min_time = tparam_.imu_time[0];
    */
    // LOG(ERROR) << " cur_time = " << std::setprecision(18) << cur_time << std::endl;   
    // LOG(ERROR) << " cur_time = " << std::setprecision(18) << cur_time << std::endl;
    // int idx = trajectory_->GetCtrlIndex(cur_time * S_TO_NS);
    // LOG(ERROR) << " last_time = " << std::setprecision(18) << last_time << std::endl;
    // int idx = trajectory_->GetCtrlIndex(last_time * S_TO_NS);
    // double opt_min_time = trajectory_->minTime(RadarSensor) + trajectory_->getDt() * idx;
    // double opt_min_time = twist_vec_.front().timestamp;

    // int idx = trajectory_->GetCtrlIndex(last_time * S_TO_NS);
    // double opt_min_time = last_time;
    // if(opt_min_time < 0) opt_min_time = 0;
    // LOG(ERROR) << " opt_min_time = " << opt_min_time << std::endl;
    // if (LocatedInFirstSegment(opt_min_time)) {
    //   estimator->SetFixedIndex(trajectory_->N - 1);
    //   LOG(ERROR) << " first Fixed Index " << trajectory_->N - 1 << std::endl;
    //   // estimator->SetFixedIndex(3);
    // } else {
    //   estimator->SetFixedIndex(before_index - 1);    // HAO TODO: 修改
    //   LOG(ERROR) << " Need to Set Fixed Index for before_index " << trajectory_->N - 1 << std::endl;
      
    // }

    // int scan_idx = trajectory_->GetCtrlIndex(tparam_.cur_time[1] * S_TO_NS);
    // option.ctrl_to_be_opt_later = std::max(scan_idx, trajectory_->N);

    // int idx = trajectory_->GetCtrlIndex(last_time * S_TO_NS);

    // before_index = -1 + idx;

    // LOG(ERROR) << "Fixed Index = " << before_index << std::endl;
    // estimator->SetFixedIndex(before_index);

    // estimator->SetFixedIndex(-1);

  // decide prior index
  double opt_min_time = twist_vec_.begin()->timestamp;
  int opt_idx = trajectory_->GetCtrlIndex(opt_min_time * S_TO_NS);
  option.ctrl_to_be_opt_now = std::min(prior_ctrl_id.first, opt_idx);

  double opt_max_time = it_base->timestamp;
  int scan_idx = trajectory_->GetCtrlIndex(opt_max_time * S_TO_NS);
  option.ctrl_to_be_opt_later = std::max(scan_idx, trajectory_->Get_N());
  LOG(ERROR) << "check ctrl later idx = std::max [" << scan_idx 
             << ", " << trajectory_->Get_N() << "]" << std::endl;


  LOG(ERROR) << "Radar max time = " << trajectory_->maxTime(RadarSensor)  << std::endl;
  LOG(ERROR) << "activate time = " << trajectory_->GetActiveTime()  << std::endl;
  LOG(ERROR) << "forcefixed time = " << trajectory_->GetForcedFixedTime()  << std::endl;
  LOG(ERROR) << "optimization time between [" << opt_min_time << ", " << opt_max_time << "]" << std::endl;
  LOG(ERROR) << "optimization ctrl idx between [" << option.ctrl_to_be_opt_now 
             << ", " << option.ctrl_to_be_opt_later << "]" << std::endl;
  
  /// HAO TODO: 这部分暂时不需要
  if (LocatedInFirstSegment(opt_min_time)) {
    // estimator->SetFixedIndex(trajectory_->Get_N() - 1);
    LOG(ERROR) << "estimator->SetFixedIndex = " << trajectory_->Get_N() - 1 << std::endl;
    LOG(ERROR) << "estimator->SetFixedIndex = " << trajectory_->Get_N() - 1 << std::endl;
    // estimator->AddStartTimePose(original_pose_);
  } else {
    estimator->SetFixedIndex(prior_ctrl_id.first - 1);
    LOG(ERROR) << "estimator->SetFixedIndex = " << prior_ctrl_id.first - 1 << std::endl;
  // }

  

  LOG(ERROR) << "trajectory_.fixed_control_point_index_ = " << estimator->GetFixedControlIndex() << std::endl;

    // trajectory_->opt_min_init_time_tmp = opt_min_time;
    // trajectory_->opt_min_init_time_tmp = last_time;
    // trajectory_->opt_init_fixed_idx_tmp = estimator->GetFixedControlIndex();
  }

  // 使用融合数据优化轨迹
  /// 构建因子图
  // []
  long int doppler_factors_num = 0;
  long int event_flow_factors_num = 0;
  {
    // assert(marg_info == nullptr);
    LOG(ERROR) << "Estimation marg_info = " << ((marg_info)? "True" : "False") << std::endl;
    // marg_info 后面会一直存在
    // [0] prior
    if (true && marg_info) {
      // 确定下一次边缘化的部分
      prior_ctrl_id.first = option.ctrl_to_be_opt_later;
      prior_ctrl_id.second = trajectory_->numKnots() - 1;

      LOG(INFO) << "[Prior Ctrl ID] = [" << prior_ctrl_id.first << ","
          << prior_ctrl_id.second << "] "
          << prior_ctrl_id.second - prior_ctrl_id.first;

      estimator->AddMarginalizationFactor(marg_info,
                                          marg_parameter_blocks);
      LOG(ERROR) << " Add Marginalize " << std::endl;
    }

    // [1] 多普勒残差
    Eigen::Matrix3d R_r_e = trajectory_->GetSensorEP(EventSensor).q.toRotationMatrix();
    LOG(ERROR) << " Twist Size = " << (it_base - twist_vec_.begin() + 1) << std::endl;
    LOG(ERROR) << " Aver Freq = " << 1.0 / (it_base->timestamp - twist_vec_.begin()->timestamp) / (it_base - twist_vec_.begin() + 1) << " Hz" << std::endl;
    for(auto it_temp_ = twist_vec_.begin(); it_temp_ <= it_base; it_temp_++)
    {

      if(it_temp_->linear_vel_vec_.norm() < 1e-10)
        continue;

      const double twist_timestamp = it_temp_->timestamp;
      {
        double linear_weight = 1.0 / it_temp_->linear_cov.norm();
        // double* vel_bias_ = para_bv_vec[1];

        sensor_msgs::PointCloud2ConstIterator<float> iter_x(it_temp_->point_cloud, "x");
        sensor_msgs::PointCloud2ConstIterator<float> iter_y(it_temp_->point_cloud, "y");
        sensor_msgs::PointCloud2ConstIterator<float> iter_z(it_temp_->point_cloud, "z");
        sensor_msgs::PointCloud2ConstIterator<float> iter_doppler(it_temp_->point_cloud, "doppler");
        
        // std::vector<Eigen::Vector3d> pt_vec;
        // std::vector<double> pt_doppler_vec;
        int radar_doppler_num = 0;
        for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z, ++iter_doppler) {
            //
            Eigen::Vector3d pt;
            pt << *iter_x, *iter_y, *iter_z;
            // 提取当前点的 doppler 值
            float pt_doppler = *iter_doppler;

            // pt_vec.push_back(pt);
            // pt_doppler_vec.push_back(pt_doppler);

          // estimator->AddDopplerMeasurementAnalytic(twist_timestamp, pt, vel_bias_,
          //   pt_doppler, R_r_e, linear_weight, false);    // 边缘化在先验处进行
          estimator->AddDopplerMeasurementAnalytic(twist_timestamp, pt, para_bv_vec[1],
            pt_doppler, R_r_e, linear_weight, false);    // 边缘化在先验处进行

          radar_doppler_num++;
        }
        LOG(ERROR) << " Add Doppler Once: " << radar_doppler_num << std::endl;
        doppler_factors_num += radar_doppler_num;

        // assert(pt_vec.size() == pt_doppler_vec.size() && "PCL Inconsistent Dimensions!");
        // // 改为批处理
        // estimator->AddDopplerMeasurementAnalytic(twist_timestamp, pt_vec, vel_bias_,
        //   pt_doppler_vec, R_r_e, linear_weight, false);    // 边缘化在先验处进行

        // TODO: timestamp is not relative to start time, trajectory_->getLinearBias()
        // LOG(INFO) << " Add Doppler " << std::endl;
        // LOG(ERROR) << " Add Doppler " << std::endl;
      }
      
      // [2] 光流残差
      {
        Eigen::Quaterniond q_e_r = trajectory_->GetSensorEP(EventSensor).q;
        Eigen::Vector3d t_e_r = trajectory_->GetSensorEP(EventSensor).p;
        double omega_weight = 1.0 / (it_temp_->angular_cov.norm() + 1e-2);
        LOG(ERROR) << " before angular_bias: " << std::endl;

        // double* angular_bias_ = para_bw_vec[1];

        event_flow_factors_num += it_temp_->best_inliers.size();
        LOG(ERROR) << " Add Flow Points: " << it_temp_->best_inliers.size() << std::endl;
        for(auto& in: it_temp_->best_inliers)
        {

          // estimator->AddEventFlowMeasurementAnalytic(twist_timestamp, in, twist.best_flow, twist.linear_vel_vec_,
          //     q_e_r, t_e_r, angular_bias, time_offset, omega_weight, false);
          // estimator->AddEventFlowMeasurementAnalytic(twist_timestamp, in, twist.best_flow, twist.linear_vel_vec_,
          //     q_e_r, t_e_r, angular_bias_, time_offset, omega_weight, false);
          // estimator->AddEventFlowMeasurementAnalytic(twist_timestamp, in, it_temp_->best_flow, it_temp_->linear_vel_vec_,
          //     q_e_r, t_e_r, para_bv_vec[1], para_bw_vec[1], time_offset, omega_weight, false);
          
          // Eigen::Vector3d pixel(in.x, in.y, 1.0);
          // Eigen::Vector3d pixel_cord = K.inverse() * pixel;            
          // estimator->AddEventFlowMeasurementAnalytic(twist_timestamp, pixel_cord, it_temp_->best_flow, it_temp_->linear_vel_vec_,
          //     q_e_r, t_e_r, para_bv_vec[1], para_bw_vec[1], time_offset, omega_weight, option.is_marg_state);
        
        }

        // estimator->AddEventFlowMeasurementAnalytic(twist_timestamp, it_temp_->best_inliers, twist.best_flow, twist.linear_vel_vec_,
        //     q_e_r, t_e_r, angular_bias, time_offset, omega_weight, false);

        // TODO: twist.best_flow 是统一的光流,可以修改为每个事件的光流
        // LOG(INFO) << " Add Event Flow " << std::endl;
        LOG(ERROR) << " Add Event Flow " << std::endl;
      }
      
      // HAO TODO: 修改Bias估计的部分(相邻两真)
      // LOG(ERROR) << " all_twist_bias_.size() = " << all_twist_bias_.size() << std::endl;
      // if(all_twist_bias_.size() > 2)
      // {
      //   double delta_time = cur_time - last_time;
      //   // double dt = 1. / 200.; // opt_weight_.imu_noise.imu_frequency;
      //   double dt = 1. / 10.; // opt_weight_.radar_frequency;
      //   double cov = delta_time / dt * (dt * dt);
      //   Eigen::Matrix<double, 6, 1> sqrt_info;
      //   sqrt_info.setOnes();
      //   sqrt_info *=  (1. / std::sqrt(cov)) ; // * opt_weight_.bias_info_vec;
      //   // LOG(ERROR) << " prepare to Add Bias Factor " << all_twist_bias_.size() << std::endl;
      //   // estimator->AddBiasFactor(all_twist_bias_[2 * last_time - cur_time].omega_bias.data() , all_twist_bias_[last_time].omega_bias.data(), 
      //   //                           all_twist_bias_[2 * last_time - cur_time].vel_bias.data(), all_twist_bias_[last_time].vel_bias.data(), 
      //   //                           1, sqrt_info);
      
      //   // double * last_omega_bias = all_twist_bias_[last_time].omega_bias.data();
      //   // double * cur_omega_bias = all_twist_bias_[cur_time].omega_bias.data();

      //   // double * last_vel_bias = all_twist_bias_[last_time].vel_bias.data();
      //   // double * cur_vel_bias = all_twist_bias_[cur_time].vel_bias.data(); 

      //   // auto it = all_twist_bias_.find(last_time);
      //   // if(it == all_twist_bias_.end())
      //   // {
      //   //   std::cerr << last_time << " is not in all_twist_bias, Check!" << std::endl;
      //   //   // return;
      //   // }

      //   // it = all_twist_bias_.find(cur_time);
      //   // if(it == all_twist_bias_.end())
      //   // {
      //   //   std::cerr << cur_time << " is not in all_twist_bias, Check!" << std::endl;
      //   //   // return;
      //   // }

      //   // LOG(ERROR) << "Main: last_time = " << last_time << std::endl;
      //   // LOG(ERROR) << "Main: cur_time = " << cur_time << std::endl;  

      //   // 进行深拷贝，确保新内存空间
      //   // Eigen::VectorXd last_omega_bias_copy = all_twist_bias_.at(last_time).omega_bias;
      //   // Eigen::VectorXd cur_omega_bias_copy = all_twist_bias_.at(cur_time).omega_bias;

      //   // Eigen::VectorXd last_vel_bias_copy = all_twist_bias_.at(last_time).vel_bias;
      //   // Eigen::VectorXd cur_vel_bias_copy = all_twist_bias_.at(cur_time).vel_bias;

      //   // 或者你可以将 `copy` 版本的数据指向新的内存
      //   // 比如将 copy 后的数据存储到新的指针中
      //   // double* last_omega_bias = last_omega_bias_copy.data();
      //   // double* cur_omega_bias = cur_omega_bias_copy.data();

      //   // double* last_vel_bias = last_vel_bias_copy.data();
      //   // double* cur_vel_bias = cur_vel_bias_copy.data();


      //   // para_bw_vec[0] = all_twist_bias_.at(last_time).omega_bias.data();
      //   // para_bw_vec[1] = all_twist_bias_.at(cur_time).omega_bias.data();

      //   // para_bv_vec[0] = all_twist_bias_.at(last_time).vel_bias.data();
      //   // para_bv_vec[1] = all_twist_bias_.at(cur_time).vel_bias.data();

      //   // 深拷贝
      //   // HAO TODO: 这一部分不应该这样
      //   // double* omega_bias_i_copy = new double[3];  // 为深拷贝分配内存
      //   // std::copy(para_bw_vec[0], para_bw_vec[0] + 3, omega_bias_i_copy);  // 复制数据
      //   // double* vel_bias_i_copy = new double[3];  // 为深拷贝分配内存
      //   // std::copy(para_bv_vec[0], para_bv_vec[0] + 3, vel_bias_i_copy);  // 复制数据
      //   // double* omega_bias_j_copy = new double[3];  // 为深拷贝分配内存
      //   // std::copy(para_bw_vec[1], para_bw_vec[1] + 3, omega_bias_j_copy);  // 复制数据
      //   // double* vel_bias_j_copy = new double[3];  // 为深拷贝分配内存
      //   // std::copy(para_bv_vec[1], para_bv_vec[1] + 3, vel_bias_j_copy);  // 复制数据

      //   // estimator->AddBiasFactor(omega_bias_i_copy, omega_bias_j_copy, 
      //   //                           vel_bias_i_copy, vel_bias_j_copy, 
      //   //                           1, sqrt_info);


      //   estimator->AddBiasFactor(para_bw_vec[0], para_bw_vec[1], 
      //                             para_bv_vec[0], para_bv_vec[1], 
      //                             1, sqrt_info);

      //   // std::map<int, double*> para_bw_vec;
      //   // std::map<int, double*> para_bv_vec;

      //   // estimator->AddBiasFactor(last_omega_bias, cur_omega_bias, 
      //   //                           last_vel_bias, cur_vel_bias, 
      //   //                           1, sqrt_info);

      //   // estimator->AddBiasFactor(all_twist_bias_[last_time].omega_bias.data() , all_twist_bias_[cur_time].omega_bias.data(), 
      //   //                   all_twist_bias_[last_time].vel_bias.data(), all_twist_bias_[cur_time].vel_bias.data(), 
      //   //                   1, sqrt_info);

      //   LOG(ERROR) << " Add Bias Factor " << std::endl;                          
      // }
      // LOG(ERROR) << " Add all factor into the solver " << std::endl;

      // [4] 时移
      // estimator->SetTimeoffsetState();
    }

    // Bias 只跟两次量测时间相关，而和具体量测数量无关
    {
      double delta_time = cur_time - last_time;
      // double dt = 1. / 200.; // opt_weight_.imu_noise.imu_frequency;
      double dt = 1. / 10.; // opt_weight_.radar_frequency;
      double cov = delta_time / dt * (dt * dt);
      Eigen::Matrix<double, 6, 1> sqrt_info;
      sqrt_info.setOnes();
      sqrt_info *=  (1. / std::sqrt(cov)) ; // * opt_weight_.bias_info_vec;

      estimator->AddBiasFactor(para_bw_vec[0], para_bw_vec[1], 
                                para_bv_vec[0], para_bv_vec[1], 
                                1, sqrt_info);

      LOG(ERROR) << " Add Bias Factor " << std::endl;                          
    }

    LOG(ERROR) << " Add all factor into the solver " << std::endl;
    LOG(ERROR) << "Estimator: Add Doppler Factors for " << doppler_factors_num << std::endl;
    LOG(ERROR) << "Estimator: Add EventFlow Factors for " << event_flow_factors_num << std::endl;
    LOG(ERROR) << "Estimator: Add Bias Factors for " << 1 << std::endl;
  }  

  /// 问题求解
  {
    LOG(ERROR) << " start to solve problem " << std::endl;

    ceres::Solver::Summary summary = estimator->Solve(50, false);
    // ceres::Solver::Summary summary = estimator->Solve(50, true, 4);
    // ceres::Solver::Summary summary = estimator->Solve(50, true, 4);
    // LOG(INFO) << summary.BriefReport();
    // LOG(INFO) << "Traj Update Successful/Unsuccessful steps: "
    //           << summary.num_successful_steps << "/"
    //           << summary.num_unsuccessful_steps;

    LOG(ERROR) << summary.BriefReport();
    LOG(ERROR) << "Traj Update Successful/Unsuccessful steps: "
              << summary.num_successful_steps << "/"
              << summary.num_unsuccessful_steps;
    LOG(ERROR) << "Traj Update details: ";
    LOG(ERROR) << summary.FullReport();



    LOG(ERROR) << "end to solve problem " << std::endl;
  }

  // LOG(ERROR) << "Estimator End Check: " 
  //            << "\npara_bw_vec[0] address at: " << para_bw_vec[0]
  //            << "\npara_bw_vec[1] address at: " << para_bw_vec[1]
  //            << "\npara_bv_vec[0] address at: " << para_bv_vec[0]
  //            << "\npara_bv_vec[1] address at: " << para_bv_vec[1]
  //            << std::endl;   

  /// 结果
  {
    // HAO TODO:
    // double opt_min_time = last_time;
    // size_t opt_start_idx = trajectory_->GetCtrlIndex(opt_min_time * S_TO_NS);
    // init_ctrl_point_.clear();
    // LOG(ERROR) << " init_ctrl_point_.size() = " << init_ctrl_point_.size() << std::endl;
    // for (size_t i = opt_start_idx; i < trajectory_->numKnots(); ++i) {
    //   const auto& p = trajectory_->getKnotPos(i);
    //   VPoint vp;
    //   vp.x = p[0];
    //   vp.y = p[1];
    //   vp.z = p[2];
    //   init_ctrl_point_.push_back(vp);
    // }
    // LOG(ERROR) << " init_ctrl_point_.size() = " << init_ctrl_point_.size() << std::endl;
    
    
    auto bias = GetLatestBias();
    // PublishLatestBias(bias.omega_bias, bias.vel_bias);
    auto pose = trajectory_->GetRadarPose(it->timestamp);
    auto vel = trajectory_->GetTransVelWorld(it->timestamp);

    auto linear_velocity = pose.rotationMatrix().transpose() * vel;    // radar velocity
    auto angular_velocity = trajectory_->GetRotVelBody(it->timestamp);  

    // Eigen::Quaterniond q_extract = pose.rotationQuaternion();
    Eigen::Quaterniond q_extract(pose.rotationMatrix());
    Eigen::Vector3d t_extract = pose.translation();
    
    // extrincs parameter event w.r.t to radar
    ExtrinsicParam Extrin_e_r = trajectory_->GetSensorEP(EventSensor);
    Eigen::Matrix4d T_e_r = Extrin_e_r.Get_T();
    double timeoffset_e_r = Extrin_e_r.Get_Timeoffset();

    LOG(ERROR) << "Estimate Result:\n" 
              << "timestamp = \t" << it->timestamp + trajectory_->GetDataStartTime() << "\n"
              << "position = \t" << t_extract.transpose() << "\n"
              << "quaternion = \t" << q_extract.coeffs().transpose() << "\n"
              << "linear velocity = \t" << linear_velocity.transpose() << "\n"
              << "angular velocity = \t" << angular_velocity.transpose() << "\n"
              << "omega_bias = \t" << Eigen::Vector3d(para_bw_vec[1]).transpose() << "\n"
              << "linear_bias = \t" << Eigen::Vector3d(para_bv_vec[1]).transpose() << "\n"
              << "T_e_r = \t" << T_e_r << "\n"
              << "time_offset = \t" << timeoffset_e_r << std::endl;

    // ceres_debug_path
    // for estimation residual
    estimator->GetResidualSummary().PrintSummary(trajectory_->minTimeNs(),
                                                 trajectory_->getDtNs());          

    double pose_time = it->timestamp + relative_start_time;
    Save_Result(pose_time, pose, bias, linear_velocity, angular_velocity);

    // 发布 Odometry 

    // 发布 Trajctory

  }


  // // 插值出速度
  // {
    


  // }

#ifdef VERBOSE:
 bool verbose = false;
  {
    TrajectoryEstimatorOptions option;
    option.lock_ab = false;
    option.lock_wb = false;
    option.show_residual_summary = verbose;
    TrajectoryEstimator::Ptr estimator(new TrajectoryEstimator(trajectory_, option, "Debug Traj"));
    
    LOG(ERROR) << " start evaluate " << std::endl;

    if (true && marg_info) {
      estimator->AddMarginalizationFactor(marg_info,
                                          marg_parameter_blocks);
      LOG(ERROR) << " Debug Marginalize " << std::endl;
    }

    Eigen::Matrix3d R_r_e = trajectory_->GetSensorEP(EventSensor).q.toRotationMatrix();
    LOG(ERROR) << " Doppler Size = " << (it_base - twist_vec_.begin() + 1) << std::endl;
    for(auto it_temp_ = twist_vec_.begin(); it_temp_ <= it_base; it_temp_++)
    {

      const double twist_timestamp = it_temp_->timestamp;
      {
        double linear_weight = 1.0 / it_temp_->linear_cov.norm();

        // all_twist_bias_[cur_time].vel_bias 的浅拷贝
        // double* vel_bias_ = all_twist_bias_[cur_time].vel_bias.data();
        double* vel_bias_ = para_bv_vec[1];

        sensor_msgs::PointCloud2ConstIterator<float> iter_x(it_temp_->point_cloud, "x");
        sensor_msgs::PointCloud2ConstIterator<float> iter_y(it_temp_->point_cloud, "y");
        sensor_msgs::PointCloud2ConstIterator<float> iter_z(it_temp_->point_cloud, "z");
        sensor_msgs::PointCloud2ConstIterator<float> iter_doppler(it_temp_->point_cloud, "doppler");
        
        // std::vector<Eigen::Vector3d> pt_vec;
        // std::vector<double> pt_doppler_vec;
        for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z, ++iter_doppler) {
            //
            Eigen::Vector3d pt;
            pt << *iter_x, *iter_y, *iter_z;
            // 提取当前点的 doppler 值
            float pt_doppler = *iter_doppler;

            // pt_vec.push_back(pt);
            // pt_doppler_vec.push_back(pt_doppler);

          estimator->AddDopplerMeasurementAnalytic(twist_timestamp, pt, vel_bias_,
            pt_doppler, R_r_e, linear_weight, false);    // 边缘化在先验处进行
        }
        LOG(ERROR) << " Debug Doppler " << std::endl;
      }

      {
        Eigen::Quaterniond q_e_r = trajectory_->GetSensorEP(EventSensor).q;
        Eigen::Vector3d t_e_r = trajectory_->GetSensorEP(EventSensor).p;
        double omega_weight = 1.0 / it_temp_->angular_cov.norm();
        LOG(ERROR) << " before angular_bias: " << std::endl;

        double* angular_bias_ = para_bw_vec[1];

        LOG(ERROR) << " Debug Flow Points: " << it_temp_->best_inliers.size() << std::endl;
        for(auto& in: it_temp_->best_inliers)
        {
          // estimator->AddEventFlowMeasurementAnalytic(twist_timestamp, in, twist.best_flow, twist.linear_vel_vec_,
          //     q_e_r, t_e_r, angular_bias, time_offset, omega_weight, false);
          // estimator->AddEventFlowMeasurementAnalytic(twist_timestamp, in, twist.best_flow, twist.linear_vel_vec_,
          //     q_e_r, t_e_r, para_bv_vec[1], angular_bias_, time_offset, omega_weight, false);
          Eigen::Vector3d pixel(in.x, in.y, 1.0);
          Eigen::Vector3d pixel_cord = K.inverse() * pixel;            
          estimator->AddEventFlowMeasurementAnalytic(twist_timestamp, pixel_cord, it_temp_->best_flow, it_temp_->linear_vel_vec_,
              q_e_r, t_e_r, para_bv_vec[1], para_bw_vec[1], time_offset, omega_weight, option.is_marg_state);
        }

        LOG(ERROR) << " Debug Event Flow " << std::endl;
      }

      {
        double delta_time = cur_time - last_time;
        // double dt = 1. / 200.; // opt_weight_.imu_noise.imu_frequency;
        double dt = 1. / 10.; // opt_weight_.radar_frequency;
        double cov = delta_time / dt * (dt * dt);
        Eigen::Matrix<double, 6, 1> sqrt_info;
        sqrt_info.setOnes();
        sqrt_info *=  (1. / std::sqrt(cov)) ; // * opt_weight_.bias_info_vec;

        estimator->AddBiasFactor(para_bw_vec[0], para_bw_vec[1], 
                                  para_bv_vec[0], para_bv_vec[1], 
                                  1, sqrt_info);

        LOG(ERROR) << " Debug Bias Factor " << std::endl;                          
      }
      LOG(ERROR) << " Debug all factor into the solver " << std::endl;
    }
  }
#endif

  // 边缘化
  {
    LOG(ERROR) << " Start to Update Prior " << std::endl;
    UpdatePrior();
    LOG(ERROR) << " Update Prior " << std::endl;
  }

  // 更新数据和时间 update_time
  {
    LOG(ERROR) << "\nShoule Update Time: " << std::setprecision(9) << traj_max_time << ", "
               << "\nActual Update Time: " <<  update_time << ", "  
               << "\nSetForce Time: " << it->timestamp << ", "  
               << "\nRadar Min Time: " << trajectory_->minTime(RadarSensor) << ", " 
               << "\nRadar Max Time: " << trajectory_->maxTime(RadarSensor) << ", " 
               << "\nTraj Min Time: " << trajectory_->minTime() << ", " 
               << "\nTraj Max Time: " << trajectory_->maxTime() << ", " 
               << std::endl;          
               

    // 更新轨迹时间
    // trajectory_->UpdateActiveTime(traj_max_time);
    trajectory_->UpdateActiveTime(update_time);
    LOG(ERROR) << " Update Active Time " << update_time << std::endl;

    // trajectory_->SetForcedFixedTime(it->timestamp);
    trajectory_->SetForcedFixedTime(it->timestamp);         // 毫米波雷达无畸变,只有一个更新时间
    LOG(ERROR) << " Set Forced Fixed Time " << update_time << std::endl;

    // LOG(ERROR) << " traj_max_time = " << traj_max_time << ", it->timestamp = " << it->timestamp << std::endl;

    // 时间更新
    // double last_2_time = last_time;

    // cur_time = it_base->timestamp + relative_start_time;  
    LOG(ERROR) << " Update Time " << std::setprecision(9) << last_time << "  |-->  " << cur_time << std::endl;
    LOG(ERROR) << " Duration Time " << std::setprecision(9) << cur_time - last_time << std::endl;
    last_time = cur_time;

    // 偏置更新
    all_twist_bias_[last_time].omega_bias = Eigen::Vector3d(para_bw_vec[0]);
    all_twist_bias_[last_time].vel_bias = Eigen::Vector3d(para_bv_vec[0]);
    all_twist_bias_[cur_time].omega_bias = Eigen::Vector3d(para_bw_vec[1]);
    all_twist_bias_[cur_time].vel_bias = Eigen::Vector3d(para_bv_vec[1]);

    // // 更新时间以后,顺推一下
    // all_twist_bias_[cur_time].omega_bias = Eigen::Vector3d(para_bw_vec[1]);
    // all_twist_bias_[cur_time].vel_bias = Eigen::Vector3d(para_bv_vec[1]);

    // LOG(ERROR) << " para_bw_vec.size() = " << para_bw_vec.size() << std::endl;
    // LOG(ERROR) << " para_bv_vec.size() = " << para_bv_vec.size() << std::endl;


    PublishTrajectoryAndMap(trajectory_, trajectory_->minTime(RadarSensor), 
                            trajectory_->maxTime(RadarSensor), trajectory_->getDt() * 2);


    // 插值状态
    // std::vector<SE3d> pose_vec;
    // 删除使用后的速度信息
    LOG(ERROR) << " twist_vec_.before_size() = " << twist_vec_.size() << std::endl;
    twist_vec_.erase(twist_vec_.begin(), it_base);
    // ((it == twist_vec_.end())? twist_vec_.end(): it + 1));

    LOG(ERROR) << " twist_vec_.size() = " << twist_vec_.size() << std::endl;
    LOG(ERROR) << " all_twist_bias_.size() = " << all_twist_bias_.size() << std::endl;
  }

  // for debug
  {
    std::cout << "Pause press any key to continue, 'q' or 'Q' for exit!" << std::endl;
    char ch = std::getchar();

    // 检查用户是否按下 'q' 键
    if (ch == 'q' || ch == 'Q') {
        std::cout << "Exiting TwistEstimator..." << std::endl;
        ros::shutdown();  // 关闭 ROS 系统
        exit(0);  // 退出程序
    }
  }

  // gt

}
}


void InitEstimation(TwistData2& twist)
{
  if (trajectory_->GetDataStartTime() < 0) {
    trajectory_->SetDataStartTime(twist.timestamp);
    relative_start_time = twist.timestamp;
    data_start_time = trajectory_->GetDataStartTime();
    std::cout << YELLOW << " trajectory set start time: " << std::setprecision(18) << twist.timestamp << std::endl;
    LOG(ERROR) << " trajectory set start time:  " << std::setprecision(18) << twist.timestamp << std::endl;

    LOG(ERROR) << "first traj_max_time = " << trajectory_->maxTime(RadarSensor) << std::endl;  
  }

  if (twist_trajctory->GetDataStartTime() < 0) {
    twist_trajctory->SetDataStartTime(twist.timestamp);
    relative_start_time = twist.timestamp;
    data_start_time = twist_trajctory->GetDataStartTime();
    std::cout << YELLOW << " trajectory set start time: " << std::setprecision(18) << twist.timestamp << std::endl;
    LOG(ERROR) << " trajectory set start time:  " << std::setprecision(18) << twist.timestamp << std::endl;

    LOG(ERROR) << "first traj_max_time = " << twist_trajctory->maxTime(RadarSensor) << std::endl;  
  }
  else
    initial_done = false;

  initial_done = true;
}

double last_end_time = 0;       // use for Local_Estimate2
void PushEstimation(TwistData2& twist)
{
  if(!initial_done)
  {
    InitEstimation(twist);
  }

  if(!twist.is_relative_time)
  {
    twist.timestamp -= (relative_start_time + last_end_time); //转换到相对时间
    twist.is_relative_time = true;
  }
  LOG(ERROR) << "twist.timestamp = " << twist.timestamp << std::endl;
  twist2_vec_.push_back(twist);
}

// void Estimate2(TwistData2& twist)
void Estimate2()
{
  LOG(ERROR) << "  ----------------- Estimate -------------------- " << std::endl;
  std::chrono::time_point<std::chrono::high_resolution_clock> start_time = std::chrono::high_resolution_clock::now();

  if(!initial_done)
    return ;

  // [step1] 设置系统初始时间
  // LOG(ERROR) << "trajectory_->GetDataStartTime() = " 
  //           << std::setprecision(20) << trajectory_->GetDataStartTime() << std::endl;
  // if (trajectory_->GetDataStartTime() < 0) {
  //   trajectory_->SetDataStartTime(twist.timestamp);
  //   relative_start_time = twist.timestamp;

  //   std::cout << YELLOW << " trajectory set start time: " << std::setprecision(18) << twist.timestamp << std::endl;
  //   LOG(ERROR) << " trajectory set start time:  " << std::setprecision(18) << twist.timestamp << std::endl;

  //   LOG(ERROR) << "first traj_max_time = " << trajectory_->maxTime(RadarSensor) << std::endl;
  // }

  // [step2] 转换数据时间
  // double data_start_time = trajectory_->GetDataStartTime();
  // if(!twist.is_relative_time)
  // {
  //   twist.timestamp -= relative_start_time; //转换到相对时间
  //   twist.is_relative_time = true;
  // }

  // HAO TODO: 这部分可以修改为 前后端的多线程
  // [step3] 缓冲区
  // twist2_vec_.push_back(twist);
  // assert(twist.is_relative_time);   // 确保是相对时间
  if(twist2_vec_.empty())
  {
    LOG(ERROR) << "twist2_vec_.empty !" << std::endl;
    return;
  }
  // LOG(ERROR) << "twist2_vec_.size = " << twist2_vec_.size() << std::endl;

  // [step4] 根据样条段，分割当前区间
  double last_max_time = trajectory_->maxTime(RadarSensor);
  double traj_max_time = last_max_time + t_add; // 单位: 秒
  // LOG(ERROR) << "last traj_max_time = " << last_max_time << std::endl;
  // LOG(ERROR) << "traj_max_time = " << traj_max_time << std::endl;
  // LOG(ERROR) << "twist2_vec_.back().timestamp = " << twist2_vec_.back().timestamp << std::endl;
  if(twist2_vec_.back().timestamp < traj_max_time)
  {
    LOG(ERROR) << "time is not enough" << std::endl;
    return;
  }

  // for(auto& twis : twist2_vec_)
  // {
  //   LOG(ERROR) << "loop timestamp = " << twis.timestamp << std::endl;
  // }

  auto it = std::find_if(twist2_vec_.rbegin(), twist2_vec_.rend(), 
                      [traj_max_time](const TwistData2& data) {
                          return data.timestamp < traj_max_time;
                      });
  it_base2 = ((it == twist2_vec_.rend())? twist2_vec_.begin() : it.base() - 1); // it 的正向跌代器

  SE3d last_knot = trajectory_->getLastKnot();
  if(it == twist2_vec_.rend())  // 没有合理的数据,直接返回,这种一般是存在缺失值
  {
    // LOG(ERROR) << " Skip " << std::endl;
    trajectory_->extendKnotsTo(traj_max_time * S_TO_NS, last_knot); // 扩展到最新的数据

    trajectory_->SetActiveTime(traj_max_time);
    // LOG(ERROR) << " Update Active Time " << traj_max_time << std::endl;

    trajectory_->SetForcedFixedTime(traj_max_time);         // 毫米波雷达无畸变,只有一个更新时间
    // LOG(ERROR) << " Set Forced Fixed Time " << traj_max_time << std::endl;

    // assert(it_base2 == twist2_vec_.begin());
    // {
    //   if(it_base2 == twist2_vec_.begin())
    //       twist2_vec_.erase(twist2_vec_.begin());
    //   else
    //       twist2_vec_.erase(twist2_vec_.begin(), it_base2 + 1);
    // }
    return;
  }

  // LOG(ERROR) << "it in " << it_base2 - twist2_vec_.begin() + 1 << std::endl;
  // LOG(ERROR) << "it->timestamp = " << it->timestamp << std::endl;
  // LOG(ERROR) << "it_base2->timestamp = " << it_base2->timestamp << std::endl;
  assert(it->timestamp == it_base2->timestamp && "Not Same Place");

  // 确定终点
  cur_time = it_base2->timestamp;
  all_twist_bias_[cur_time] = all_twist_bias_[last_time];
  double update_time = it->timestamp;  // 注意是相对时间的 秒
  
  // LOG(ERROR) << "CHECK max time 1 = " << trajectory_->maxTime(RadarSensor) << std::endl;
  // LOG(ERROR) << "it->timestamp * S_TO_NS = " << it->timestamp * S_TO_NS << std::endl;
  trajectory_->extendKnotsTo(it->timestamp * S_TO_NS, last_knot); // 扩展到最新的数据
  // LOG(ERROR) << "CHECK max time 2 = " << trajectory_->maxTime(RadarSensor) << std::endl;

  // Debug: 某端时间是合理的
  /*{
    LOG(ERROR) << " trajectory_ extend knots timestamp: " << std::setprecision(9) << it->timestamp << std::endl;
    LOG(ERROR) << " check update timestamp: " << std::setprecision(9) << it->timestamp - traj_max_time << std::endl;  // should be <
    // 如果下一个存在,检查一下下一个的时间戳
    if(it_base2 != twist2_vec_.end() && (it_base2 + 1) != twist2_vec_.end())
    {
      LOG(ERROR) << " trajectory_ extend knots timestamp: " << std::setprecision(9) << (it + 1)->timestamp << std::endl;
      LOG(ERROR) << " check update timestamp: " << std::setprecision(9) << (it - 1)->timestamp - traj_max_time << std::endl;
    }
  }*/

  std::map<int, double*> para_bw_vec;
  std::map<int, double*> para_bv_vec;
  // 分配内存并深拷贝 omega_bias 数据
  para_bw_vec[0] = new double[3];
  std::memcpy(para_bw_vec[0], all_twist_bias_.at(last_time).omega_bias.data(), 3 * sizeof(double));

  para_bw_vec[1] = new double[3];
  std::memcpy(para_bw_vec[1], all_twist_bias_.at(cur_time).omega_bias.data(), 3 * sizeof(double));

  // 分配内存并深拷贝 vel_bias 数据
  para_bv_vec[0] = new double[3];
  std::memcpy(para_bv_vec[0], all_twist_bias_.at(last_time).vel_bias.data(), 3 * sizeof(double));

  para_bv_vec[1] = new double[3];
  std::memcpy(para_bv_vec[1], all_twist_bias_.at(cur_time).vel_bias.data(), 3 * sizeof(double));

  std::chrono::time_point<std::chrono::high_resolution_clock> time1 = std::chrono::high_resolution_clock::now();

  // [step5] 创建优化器
  TrajectoryEstimatorOptions option;
  option.lock_EPs.at(EventSensor).Unlock();
  option.is_marg_state = false;
  option.show_residual_summary = true;
  TrajectoryEstimator::Ptr estimator(new TrajectoryEstimator(trajectory_, option, "Update Traj"));
  // LOG(ERROR) << " start estimate " << std::endl;

  // decide prior index
  double opt_min_time = twist2_vec_.begin()->timestamp;
  int opt_idx = trajectory_->GetCtrlIndex(opt_min_time * S_TO_NS);
  option.ctrl_to_be_opt_now = std::min(prior_ctrl_id.first, opt_idx);
  double opt_max_time = it_base2->timestamp;
  int scan_idx = trajectory_->GetCtrlIndex(opt_max_time * S_TO_NS);
  option.ctrl_to_be_opt_later = std::max(scan_idx, trajectory_->Get_N());
  // LOG(ERROR) << "check ctrl later idx = std::max [" << scan_idx 
  //            << ", " << trajectory_->Get_N() << "]" << std::endl;

  // LOG(ERROR) << "Radar max time = " << trajectory_->maxTime(RadarSensor)  << std::endl;
  // LOG(ERROR) << "activate time = " << trajectory_->GetActiveTime()  << std::endl;
  // LOG(ERROR) << "forcefixed time = " << trajectory_->GetForcedFixedTime()  << std::endl;
  // LOG(ERROR) << "optimization time between [" << opt_min_time << ", " << opt_max_time << "]" << std::endl;
  // LOG(ERROR) << "optimization ctrl idx between [" << option.ctrl_to_be_opt_now 
  //            << ", " << option.ctrl_to_be_opt_later << "]" << std::endl;
  
  // HAO TODO: 这部分暂时不需要
  // TODO: 确定 prior_ctrl_id.first 的数值
  if (LocatedInFirstSegment(opt_min_time)) {
    // estimator->SetFixedIndex(trajectory_->Get_N() - 1);
    // LOG(ERROR) << "estimator->SetFixedIndex = " << trajectory_->Get_N() - 1 << std::endl;
    // LOG(ERROR) << "estimator->SetFixedIndex = " << trajectory_->Get_N() - 1 << std::endl;
    // estimator->AddStartTimePose(original_pose_);
  } else {
    estimator->SetFixedIndex(prior_ctrl_id.first - 1);
    // LOG(ERROR) << "estimator->SetFixedIndex = " << prior_ctrl_id.first - 1 << std::endl;
    // LOG(ERROR) << "trajectory_.fixed_control_point_index_ = " << estimator->GetFixedControlIndex() << std::endl;

    // trajectory_->opt_min_init_time_tmp = opt_min_time;
    // trajectory_->opt_min_init_time_tmp = last_time;
    // trajectory_->opt_init_fixed_idx_tmp = estimator->GetFixedControlIndex();
  }

  // [step5] 因子图优化
  /// 因子图优化
  std::chrono::time_point<std::chrono::high_resolution_clock> time3;
  std::chrono::time_point<std::chrono::high_resolution_clock> time4;
  std::chrono::time_point<std::chrono::high_resolution_clock> time5;
  long int doppler_factors_num = 0;
  long int event_flow_factors_num = 0;

  // 联合优化
  //{
    // [step5-1] 优化问题建模
    /*{
      time3 = std::chrono::high_resolution_clock::now();
      // assert(marg_info == nullptr);
      LOG(ERROR) << "Estimation marg_info = " << ((marg_info)? "True" : "False") << std::endl;
      // marg_info 后面会一直存在
      // [step5-0] 先验因子
      if (true && marg_info) {
        // 确定下一次边缘化的部分
        prior_ctrl_id.first = option.ctrl_to_be_opt_later;
        prior_ctrl_id.second = trajectory_->numKnots() - 1;

        LOG(ERROR) << "[Prior Ctrl ID] = [" << prior_ctrl_id.first << ","
            << prior_ctrl_id.second << "] "
            << prior_ctrl_id.second - prior_ctrl_id.first;

        estimator->AddMarginalizationFactor(marg_info,
                                            marg_parameter_blocks);
        LOG(ERROR) << " Add Marginalize " << std::endl;
      }
      
      time4 = std::chrono::high_resolution_clock::now();
      // 根据量测构建因子图优化(Loop Optimization)
      Eigen::Matrix3d R_r_e = trajectory_->GetSensorEP(EventSensor).q.toRotationMatrix();
      LOG(ERROR) << " Twist Size = " << (it_base2 - twist2_vec_.begin() + 1) << std::endl;
      // LOG(ERROR) << " Aver Freq = " << 1.0 / (it_base2->timestamp - twist2_vec_.begin()->timestamp) / (it_base2 - twist2_vec_.begin() + 1) << " Hz" << std::endl;

      for(auto it_temp_ = twist2_vec_.begin(); it_temp_ <= it_base2; it_temp_++)
      {
        // [step5-1] 多普勒残差
        const double twist_timestamp = it_temp_->timestamp;
        LOG(ERROR) << "twist time = " << twist_timestamp << ", maxTime = " << trajectory_->maxTime(RadarSensor) << " dt = " << trajectory_->getDt() << std::endl;
        LOG(ERROR) << "twist in " << (int)((twist_timestamp - last_max_time) / trajectory_->getDt());
        {
          
          // if(it_temp_->linear_vel_vec_.norm() < 1e-10)
          LOG(ERROR) << "pcl num = " << it_temp_->point_cloud.width << std::endl;
          LOG(ERROR) << "vel norm = " << it_temp_->linear_vel_vec_.norm() << std::endl;

          if(it_temp_->point_cloud.width < 5)
            continue;

          // double linear_weight = 1.0 / it_temp_->linear_cov.norm();
          // double linear_weight = 1.0;

          // sensor_msgs::PointCloud2ConstIterator<float> iter_x(it_temp_->point_cloud, "x");
          // sensor_msgs::PointCloud2ConstIterator<float> iter_y(it_temp_->point_cloud, "y");
          // sensor_msgs::PointCloud2ConstIterator<float> iter_z(it_temp_->point_cloud, "z");
          // sensor_msgs::PointCloud2ConstIterator<float> iter_doppler(it_temp_->point_cloud, "doppler");

          sensor_msgs::PointCloud2ConstIterator<float> iter_x(it_temp_->point_cloud, "x");
          sensor_msgs::PointCloud2ConstIterator<float> iter_y(it_temp_->point_cloud, "y");
          sensor_msgs::PointCloud2ConstIterator<float> iter_z(it_temp_->point_cloud, "z");
          sensor_msgs::PointCloud2ConstIterator<float> iter_doppler(it_temp_->point_cloud, "doppler");

          // std::vector<Eigen::Vector3d> pt_vec;
          // std::vector<double> pt_doppler_vec;
          int radar_doppler_num = 0;
          for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z, ++iter_doppler) {
              //
              Eigen::Vector3d pt;
              pt << *iter_x, *iter_y, *iter_z;
              // 提取当前点的 doppler 值
              float pt_doppler = *iter_doppler;

            estimator->AddDopplerMeasurementAnalytic(twist_timestamp, pt, para_bv_vec[1],
              pt_doppler, R_r_e, linear_weight, false);    // 边缘化在先验处进行

            radar_doppler_num++;
          }
          LOG(ERROR) << " Add Doppler Once: " << radar_doppler_num << std::endl;
          doppler_factors_num += radar_doppler_num;
        }

        if(it_temp_->best_inliers.size() < 5)
          continue;

        // [2] 光流残差
        {
          Eigen::Quaterniond q_e_r = trajectory_->GetSensorEP(EventSensor).q;
          Eigen::Vector3d t_e_r = trajectory_->GetSensorEP(EventSensor).p;
          // double omega_weight = 1.0 / (it_temp_->angular_cov.norm() + 1e-2);
          // 1-11 修改
          // double omega_weight = 1.0;
          LOG(ERROR) << " before angular_bias: " << std::endl;

          // double* angular_bias_ = para_bw_vec[1];

          event_flow_factors_num += it_temp_->best_inliers.size();
          LOG(ERROR) << " Add Flow Points: " << it_temp_->best_inliers.size() << std::endl;
          for(int i = 0; i < it_temp_->best_inliers.size(); i++)
          {
          Eigen::Vector3d pixel(it_temp_->best_inliers[i].x, it_temp_->best_inliers[i].y, 1.0);
          Eigen::Vector3d pixel_cord = K.inverse() * pixel;            
          estimator->AddEventFlowMeasurementAnalytic(twist_timestamp, pixel_cord, it_temp_->best_flow[i], it_temp_->linear_vel_vec_, 
            q_e_r, t_e_r, para_bv_vec[1], para_bw_vec[1], time_offset, omega_weight, option.is_marg_state);
            
            // estimator->AddEventFlowMeasurementAnalytic(twist_timestamp, it_temp_->best_inliers[i], it_temp_->best_flow[i], it_temp_->linear_vel_vec_,
            //   q_e_r, t_e_r, para_bv_vec[1], para_bw_vec[1], time_offset, omega_weight, option.is_marg_state);
          }
          LOG(ERROR) << " Add Event Flow " << std::endl;
        }
      }

      // [3] Twist Bias 估计
      // Bias 只跟两次量测时间相关，而和具体量测数量无关
      {
        double delta_time = cur_time - last_time;
        // double dt = 1. / 200.; // opt_weight_.imu_noise.imu_frequency;
        // double dt = 1. / 10.; // opt_weight_.radar_frequency;
        double dt = 1. / 20.; // opt_weight_.radar_frequency;
        double cov = delta_time / dt * (dt * dt);
        Eigen::Matrix<double, 6, 1> sqrt_info;
        sqrt_info.setOnes();
        sqrt_info *=  (1. / std::sqrt(cov)); // * opt_weight_.bias_info_vec;

        LOG(ERROR) << "100 * sqrt_info = " << 100 * sqrt_info << std::endl;

        estimator->AddBiasFactor(para_bw_vec[0], para_bw_vec[1], 
                                  para_bv_vec[0], para_bv_vec[1], 
                                  1, 100 * sqrt_info); // 100 * sqrt_info);

        LOG(ERROR) << " Add Bias Factor " << std::endl;                          
      }

      LOG(ERROR) << " Add all factor into the solver " << std::endl;
      LOG(ERROR) << "Estimator: Add Doppler Factors for " << doppler_factors_num << std::endl;
      LOG(ERROR) << "Estimator: Add EventFlow Factors for " << event_flow_factors_num << std::endl;
      LOG(ERROR) << "Estimator: Add Bias Factors for " << 1 << std::endl;
    }  

    // 雷达线速度必须存在
    time5 = std::chrono::high_resolution_clock::now();
    if(doppler_factors_num > 0)
    {   
      // [step5-2] 优化问题求解
      {
        LOG(ERROR) << " start to solve problem " << std::endl;

        ceres::Solver::Summary summary = estimator->Solve(50, false);
        // ceres::Solver::Summary summary = estimator->Solve(50, true, 4);  // use multi-thread
        LOG(ERROR) << summary.BriefReport();
        LOG(ERROR) << "Traj Update Successful/Unsuccessful steps: "
                  << summary.num_successful_steps << "/"
                  << summary.num_unsuccessful_steps;
        LOG(ERROR) << "Traj Update details: ";
        LOG(ERROR) << summary.FullReport();



        LOG(ERROR) << "end to solve problem " << std::endl;
      }
    }
  //}/// 因子图优化*/
// */
/*
  // 分步优化
  //{
    // [step5-1] 优化问题建模
    // 1. 优化多普勒线速度
    {
      time3 = std::chrono::high_resolution_clock::now();
      LOG(ERROR) << "Estimation marg_info = " << ((marg_info)? "True" : "False") << std::endl;
      // [step5-0] 先验因子
      if (true && marg_info) {
        // 确定下一次边缘化的部分
        prior_ctrl_id.first = option.ctrl_to_be_opt_later;
        prior_ctrl_id.second = trajectory_->numKnots() - 1;

        LOG(ERROR) << "[Prior Ctrl ID] = [" << prior_ctrl_id.first << ","
            << prior_ctrl_id.second << "] "
            << prior_ctrl_id.second - prior_ctrl_id.first;

        estimator->AddMarginalizationFactor(marg_info,
                                            marg_parameter_blocks);
        LOG(ERROR) << " Add Marginalize " << std::endl;
      }
      time4 = std::chrono::high_resolution_clock::now();
      // 根据量测构建因子图优化(Loop Optimization)
      Eigen::Matrix3d R_r_e = trajectory_->GetSensorEP(EventSensor).q.toRotationMatrix();
      LOG(ERROR) << " Twist Size = " << (it_base2 - twist2_vec_.begin() + 1) << std::endl;

      for(auto it_temp_ = twist2_vec_.begin(); it_temp_ <= it_base2; it_temp_++)
      {
        // [step5-1] 多普勒残差
        const double twist_timestamp = it_temp_->timestamp;
        LOG(ERROR) << "twist time = " << twist_timestamp << ", maxTime = " << trajectory_->maxTime(RadarSensor) << " dt = " << trajectory_->getDt() << std::endl;
        LOG(ERROR) << "twist in " << (int)((twist_timestamp - last_max_time) / trajectory_->getDt());
        {
          
          // if(it_temp_->linear_vel_vec_.norm() < 1e-10)
          LOG(ERROR) << "pcl num = " << it_temp_->point_cloud.width << std::endl;
          LOG(ERROR) << "vel norm = " << it_temp_->linear_vel_vec_.norm() << std::endl;

          if(it_temp_->point_cloud.width < 5)
            continue;

          // double linear_weight = 1.0 / it_temp_->linear_cov.norm();
          double linear_weight = 1.0;

          sensor_msgs::PointCloud2ConstIterator<float> iter_x(it_temp_->point_cloud, "x");
          sensor_msgs::PointCloud2ConstIterator<float> iter_y(it_temp_->point_cloud, "y");
          sensor_msgs::PointCloud2ConstIterator<float> iter_z(it_temp_->point_cloud, "z");
          sensor_msgs::PointCloud2ConstIterator<float> iter_doppler(it_temp_->point_cloud, "doppler");

          // std::vector<Eigen::Vector3d> pt_vec;
          // std::vector<double> pt_doppler_vec;
          int radar_doppler_num = 0;
          for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z, ++iter_doppler) {
              //
              Eigen::Vector3d pt;
              pt << *iter_x, *iter_y, *iter_z;
              // 提取当前点的 doppler 值
              float pt_doppler = *iter_doppler;

            estimator->AddDopplerMeasurementAnalytic(twist_timestamp, pt, para_bv_vec[1],
              pt_doppler, R_r_e, linear_weight, false);    // 边缘化在先验处进行

              
            // SplineMeta<SplineOrder>& spline_meta
            // estimator->AddDopplerMeasurementAnalytic2(twist_timestamp, SplineMeta<SplineOrder> spline_meta, pt, para_bv_vec[1],
            //   pt_doppler, R_r_e, linear_weight, false);    // 边缘化在先验处进行

            radar_doppler_num++;
          }
          LOG(ERROR) << " Add Doppler Once: " << radar_doppler_num << std::endl;
          doppler_factors_num += radar_doppler_num;
        }
      }

      // [3] Twist Bias 估计
      // Bias 只跟两次量测时间相关，而和具体量测数量无关
      {
        double delta_time = cur_time - last_time;
        // double dt = 1. / 200.; // opt_weight_.imu_noise.imu_frequency;
        // double dt = 1. / 10.; // opt_weight_.radar_frequency;
        double dt = 1. / 20.; // opt_weight_.radar_frequency;
        double cov = delta_time / dt * (dt * dt);
        Eigen::Matrix<double, 6, 1> sqrt_info;
        sqrt_info.setOnes();
        sqrt_info *=  (1. / std::sqrt(cov)); // * opt_weight_.bias_info_vec;

        LOG(ERROR) << "100 * sqrt_info = " << 100 * sqrt_info << std::endl;

        estimator->AddBiasFactor(para_bw_vec[0], para_bw_vec[1], 
                                  para_bv_vec[0], para_bv_vec[1], 
                                  1, 100 * sqrt_info); // 100 * sqrt_info);

        LOG(ERROR) << " Add Bias Factor " << std::endl;                          
      }


      // [step5-2] 优化问题求解
      {
        LOG(ERROR) << " start to solve problem " << std::endl;

        ceres::Solver::Summary summary = estimator->Solve(50, false);
        // ceres::Solver::Summary summary = estimator->Solve(50, true, 4);  // use multi-thread
        LOG(ERROR) << summary.BriefReport();
        LOG(ERROR) << "Traj Update Successful/Unsuccessful steps: "
                  << summary.num_successful_steps << "/"
                  << summary.num_unsuccessful_steps;
        LOG(ERROR) << "Traj Update details: ";
        LOG(ERROR) << summary.FullReport();



        LOG(ERROR) << "end to solve problem " << std::endl;
      }
    }
    // 需要 Doppler + Bias 进行联合估计

    TrajectoryEstimatorOptions option2;
  {
    // [step5] 创建优化器

    option2.lock_EPs.at(EventSensor).Unlock();
    option2.is_marg_state = false;
    option2.show_residual_summary = true;
  }
    TrajectoryEstimator::Ptr estimator2(new TrajectoryEstimator(trajectory_, option2, "Update Traj"));
    LOG(ERROR) << " start estimate " << std::endl;
  {
    // decide prior index
    double opt_min_time = twist2_vec_.begin()->timestamp;
    int opt_idx = trajectory_->GetCtrlIndex(opt_min_time * S_TO_NS);
    option.ctrl_to_be_opt_now = std::min(prior_ctrl_id.first, opt_idx);
    double opt_max_time = it_base2->timestamp;
    int scan_idx = trajectory_->GetCtrlIndex(opt_max_time * S_TO_NS);
    option.ctrl_to_be_opt_later = std::max(scan_idx, trajectory_->Get_N());
    LOG(ERROR) << "check ctrl later idx = std::max [" << scan_idx 
              << ", " << trajectory_->Get_N() << "]" << std::endl;

    // HAO TODO: 这部分暂时不需要
    // TODO: 确定 prior_ctrl_id.first 的数值
    if (LocatedInFirstSegment(opt_min_time)) {
      LOG(ERROR) << "estimator->SetFixedIndex = " << trajectory_->Get_N() - 1 << std::endl;
    } else {
      estimator2->SetFixedIndex(prior_ctrl_id.first - 1);
      LOG(ERROR) << "estimator2->SetFixedIndex = " << prior_ctrl_id.first - 1 << std::endl;
      LOG(ERROR) << "trajectory_.fixed_control_point_index_ = " << estimator2->GetFixedControlIndex() << std::endl;
    }
  }

    // 2. 优化事件角速度
    {
       // 根据量测构建因子图优化(Loop Optimization)
      Eigen::Matrix3d R_r_e = trajectory_->GetSensorEP(EventSensor).q.toRotationMatrix();
      LOG(ERROR) << " Twist Size = " << (it_base2 - twist2_vec_.begin() + 1) << std::endl;
      for(auto it_temp_ = twist2_vec_.begin(); it_temp_ <= it_base2; it_temp_++)
      {
        // [2] 光流残差
        const double twist_timestamp = it_temp_->timestamp;
        {
          Eigen::Quaterniond q_e_r = trajectory_->GetSensorEP(EventSensor).q;
          Eigen::Vector3d t_e_r = trajectory_->GetSensorEP(EventSensor).p;
          // double omega_weight = 1.0 / (it_temp_->angular_cov.norm() + 1e-2);
          // 1-11 修改
          double omega_weight = 2.0;
          LOG(ERROR) << " before angular_bias: " << std::endl;

          // double* angular_bias_ = para_bw_vec[1];

          event_flow_factors_num += it_temp_->best_inliers.size();
          LOG(ERROR) << " Add Flow Points: " << it_temp_->best_inliers.size() << std::endl;
          for(int i = 0; i < it_temp_->best_inliers.size(); i++)
          {
            Eigen::Vector3d pixel(it_temp_->best_inliers[i].x, it_temp_->best_inliers[i].y, 1.0);
            Eigen::Vector3d pixel_cord = K.inverse() * pixel;            
            // estimator->AddEventFlowMeasurementAnalytic(twist_timestamp, pixel_cord, it_temp_->best_flow[i], it_temp_->linear_vel_vec_, 
            //   q_e_r, t_e_r, para_bv_vec[1], para_bw_vec[1], time_offset, omega_weight, option.is_marg_state);
            
            estimator2->AddEventFlowMeasurementAnalytic(twist_timestamp, pixel_cord, it_temp_->best_flow[i], it_temp_->linear_vel_vec_, 
              q_e_r, t_e_r, para_bv_vec[1], para_bw_vec[1], time_offset, omega_weight, option.is_marg_state);
                        
              // estimator->AddEventFlowMeasurementAnalytic(twist_timestamp, it_temp_->best_inliers[i], it_temp_->best_flow[i], it_temp_->linear_vel_vec_,
              //   q_e_r, t_e_r, para_bv_vec[1], para_bw_vec[1], time_offset, omega_weight, option.is_marg_state);
          }
          LOG(ERROR) << " Add Event Flow " << std::endl;
        }
      }

      // [3] Twist Bias 估计
      // Bias 只跟两次量测时间相关，而和具体量测数量无关
      {
        double delta_time = cur_time - last_time;
        // double dt = 1. / 200.; // opt_weight_.imu_noise.imu_frequency;
        // double dt = 1. / 10.; // opt_weight_.radar_frequency;
        double dt = 1. / 20.; // opt_weight_.radar_frequency;
        double cov = delta_time / dt * (dt * dt);
        Eigen::Matrix<double, 6, 1> sqrt_info;
        sqrt_info.setOnes();
        sqrt_info *=  (1. / std::sqrt(cov)); // * opt_weight_.bias_info_vec;

        LOG(ERROR) << "100 * sqrt_info = " << 100 * sqrt_info << std::endl;

        estimator2->AddBiasFactor(para_bw_vec[0], para_bw_vec[1], 
                                  para_bv_vec[0], para_bv_vec[1], 
                                  1, 100 * sqrt_info); // 100 * sqrt_info);

        LOG(ERROR) << " Add Bias Factor " << std::endl;                          
      }

      // [step5-2] 优化问题求解
      {
        LOG(ERROR) << " start to solve problem " << std::endl;

        ceres::Solver::Summary summary = estimator2->Solve(50, false);
        // ceres::Solver::Summary summary = estimator->Solve(50, true, 4);  // use multi-thread
        LOG(ERROR) << summary.BriefReport();
        LOG(ERROR) << "Traj Update Successful/Unsuccessful steps: "
                  << summary.num_successful_steps << "/"
                  << summary.num_unsuccessful_steps;
        LOG(ERROR) << "Traj Update details: ";
        LOG(ERROR) << summary.FullReport();



        LOG(ERROR) << "end to solve problem " << std::endl;
      }
    }
*/
    // 雷达线速度必须存在
    time5 = std::chrono::high_resolution_clock::now();

    // LOG(ERROR) << " Add all factor into the solver " << std::endl;
    // LOG(ERROR) << "Estimator: Add Doppler Factors for " << doppler_factors_num << std::endl;
    // LOG(ERROR) << "Estimator: Add EventFlow Factors for " << event_flow_factors_num << std::endl;
    // LOG(ERROR) << "Estimator: Add Bias Factors for " << 1 << std::endl;
  //}/// 因子图优化

  // 松耦合
  {
    // [step5-1] 优化问题建模
    {
      time3 = std::chrono::high_resolution_clock::now();
      // assert(marg_info == nullptr);
      LOG(ERROR) << "Estimation marg_info = " << ((marg_info)? "True" : "False") << std::endl;
      // marg_info 后面会一直存在
      // [step5-0] 先验因子
      prior_ctrl_id.first = option.ctrl_to_be_opt_later;
      prior_ctrl_id.second = trajectory_->numKnots() - 1;
      if (true && marg_info) {
        // 确定下一次边缘化的部分
        prior_ctrl_id.first = option.ctrl_to_be_opt_later;
        prior_ctrl_id.second = trajectory_->numKnots() - 1;

        LOG(ERROR) << "[Prior Ctrl ID] = [" << prior_ctrl_id.first << ","
            << prior_ctrl_id.second << "] "
            << prior_ctrl_id.second - prior_ctrl_id.first;

        estimator->AddMarginalizationFactor(marg_info,
                                            marg_parameter_blocks);
        LOG(ERROR) << " Add Marginalize " << std::endl;
      }
      
      time4 = std::chrono::high_resolution_clock::now();
      // 根据量测构建因子图优化(Loop Optimization)
      Eigen::Matrix3d R_r_e = trajectory_->GetSensorEP(EventSensor).q.toRotationMatrix();
      LOG(ERROR) << " Twist Size = " << (it_base2 - twist2_vec_.begin() + 1) << std::endl;
      // LOG(ERROR) << " Aver Freq = " << 1.0 / (it_base2->timestamp - twist2_vec_.begin()->timestamp) / (it_base2 - twist2_vec_.begin() + 1) << " Hz" << std::endl;

      for(auto it_temp_ = twist2_vec_.begin(); it_temp_ <= it_base2; it_temp_++)
      {
        // [step5-1] 多普勒残差
        const double twist_timestamp = it_temp_->timestamp;
        LOG(ERROR) << "twist time = " << twist_timestamp << ", maxTime = " << trajectory_->maxTime(RadarSensor) << " dt = " << trajectory_->getDt() << std::endl;
        LOG(ERROR) << "twist in " << (int)((twist_timestamp - last_max_time) / trajectory_->getDt());
        {
          
          estimator->AddBodyLocalVelocityMeasurementAnalytic(twist_timestamp, para_bv_vec[1], 
                                                          it_temp_->linear_vel_vec_, linear_weight, linear_w_weight, R_weight, false);

          estimator->AddBodyLocalAngularVelocityMeasurementAnalytic(twist_timestamp, para_bw_vec[1], 
                                                          it_temp_-> angular_vel_vec_, omega_weight, omega_w_weight, false);                                            
        }

      }

      // [3] Twist Bias 估计
      // Bias 只跟两次量测时间相关，而和具体量测数量无关
      {
        double delta_time = cur_time - last_time;
        delta_time = std::max(delta_time, 1e-3);

        // double dt = 1. / 200.; // opt_weight_.imu_noise.imu_frequency;
        // double dt = 1. / 10.; // opt_weight_.radar_frequency;
        double dt = 1. / 20.; // opt_weight_.radar_frequency;
        double cov = delta_time / dt * (dt * dt);
        LOG(ERROR) << "delta_time = " << delta_time << std::endl;
        LOG(ERROR) << "cov = " << cov << std::endl;
        Eigen::Matrix<double, 6, 1> sqrt_info;
        sqrt_info.setOnes();
        sqrt_info *=  (1. / std::sqrt(cov)); // * opt_weight_.bias_info_vec;

        LOG(ERROR) << "100 * sqrt_info = " << 100 * sqrt_info << std::endl;

        estimator->AddBiasFactor(para_bw_vec[0], para_bw_vec[1], 
                                  para_bv_vec[0], para_bv_vec[1], 
                                  1, 100 * sqrt_info); // 100 * sqrt_info);

        LOG(ERROR) << " Add Bias Factor " << std::endl;                          
      }

      LOG(ERROR) << " Add all factor into the solver " << std::endl;
      LOG(ERROR) << "Estimator: Add Doppler Factors for " << doppler_factors_num << std::endl;
      LOG(ERROR) << "Estimator: Add EventFlow Factors for " << event_flow_factors_num << std::endl;
      LOG(ERROR) << "Estimator: Add Bias Factors for " << 1 << std::endl;
    }  

    // 雷达线速度必须存在
    time5 = std::chrono::high_resolution_clock::now();
    // if(doppler_factors_num > 0)
    // {   
      // [step5-2] 优化问题求解
      {
        LOG(ERROR) << " start to solve problem " << std::endl;

        // ceres::Solver::Summary summary = estimator->Solve(50, false);
        ceres::Solver::Summary summary = estimator->Solve(50, false, 8);  // use multi-thread
        LOG(ERROR) << summary.BriefReport();
        LOG(ERROR) << "Traj Update Successful/Unsuccessful steps: "
                  << summary.num_successful_steps << "/"
                  << summary.num_unsuccessful_steps;
        LOG(ERROR) << "Traj Update details: ";
        LOG(ERROR) << summary.FullReport();

        LOG(ERROR) << "end to solve problem " << std::endl;
      }
    // }
  }/// 因子图优化*/




  /// 发布结果
  std::chrono::time_point<std::chrono::high_resolution_clock> time6 = std::chrono::high_resolution_clock::now();
  /*{
    auto bias = GetLatestBias();
    // PublishLatestBias(bias.omega_bias, bias.vel_bias);
    auto pose = trajectory_->GetRadarPose(it->timestamp);

    // Eigen::Vector3d linear_velocity = pose.rotationMatrix().transpose() * trajectory_->GetTransVelWorld(it->timestamp);    // radar velocity
    Eigen::Vector3d linear_velocity = trajectory_->GetTransVelWorld(it->timestamp);    // radar velocity
    Eigen::Vector3d angular_velocity = trajectory_->GetRotVelBody(it->timestamp);  

    // Eigen::Quaterniond q_extract = pose.rotationQuaternion();
    Eigen::Quaterniond q_extract(pose.rotationMatrix());
    Eigen::Vector3d t_extract = pose.rotationMatrix() * pose.translation();
    

    // extrincs parameter event w.r.t to radar
    ExtrinsicParam Extrin_e_r = trajectory_->GetSensorEP(EventSensor);
    Eigen::Matrix4d T_e_r = Extrin_e_r.Get_T();
    double timeoffset_e_r = Extrin_e_r.Get_Timeoffset();

    LOG(ERROR) << "Estimate Result:\n" 
              << "timestamp = \t" << std::setprecision(18) << it->timestamp + trajectory_->GetDataStartTime() << "\n"
              << "position = \t" << t_extract.transpose() << "\n"
              << "quaternion = \t" << q_extract.coeffs().transpose() << "\n"
              << "linear velocity = \t" << linear_velocity(0) << ", " << linear_velocity(1) << ", " << linear_velocity(2) << "\n"
              << "angular velocity = \t" << angular_velocity(0) << ", " << angular_velocity(1) << ", " << angular_velocity(2) << "\n"
              << "omega_bias = \t" << Eigen::Vector3d(para_bw_vec[1]).transpose() << "\n"
              << "linear_bias = \t" << Eigen::Vector3d(para_bv_vec[1]).transpose() << "\n"
              << "T_e_r = \t" << T_e_r << "\n"
              << "time_offset = \t" << timeoffset_e_r << std::endl;

    // ceres_debug_path
    // for estimation residual
    estimator->GetResidualSummary().PrintSummary(trajectory_->minTimeNs(),
                                                 trajectory_->getDtNs());          

    double pose_time = it->timestamp + relative_start_time;
    // Save_Result(pose_time, pose, bias);
    Save_Result(pose_time, pose, bias, linear_velocity, angular_velocity);
  }  /// 发布结果 */

  // check velocity estimation
  /* std::fstream vel_output_file;
  vel_output_file.open("/home/hao/Desktop/radar-event-new/src/TwistEstimator/output/estimate.tum", std::ios::out | std::ios::app);
  for(auto it_temp_ = twist2_vec_.begin(); it_temp_ <= it_base2; it_temp_++)
  {
    auto pose = trajectory_->GetRadarPose(it_temp_->timestamp);
    Eigen::Vector3d linear_velocity = pose.rotationMatrix().transpose() * trajectory_->GetTransVelWorld(it_temp_->timestamp);    // radar velocity
    Eigen::Vector3d angular_velocity = trajectory_->GetRotVelBody(it_temp_->timestamp);  
    LOG(ERROR) << "Estimate Velocity:\n"
               << "origin linear velocity = \t" << it_temp_->linear_vel_vec_(0) << ", " << it_temp_->linear_vel_vec_(1) << ", " << it_temp_->linear_vel_vec_(2) << "\n"
               << "estimate linear velocity = \t" << linear_velocity(0) << ", " << linear_velocity(1) << ", " << linear_velocity(2) << "\n"
               << "origin angular velocity = \t" << it_temp_->angular_vel_vec_(0) << ", " << it_temp_->angular_vel_vec_(1) << ", " << it_temp_->angular_vel_vec_(2) << "\n"
               << "estimate angular velocity = \t" << angular_velocity(0) << ", " << angular_velocity(1) << ", " << angular_velocity(2) << "\n";        
  
    vel_output_file << std::setprecision(20) << it_temp_->timestamp + data_start_time << " ";
    vel_output_file << linear_velocity(0) << " " << linear_velocity(1) << " " << linear_velocity(2) << " ";
    vel_output_file << angular_velocity(0) << " " << angular_velocity(1) << " " << angular_velocity(2) << " ";
    vel_output_file << std::endl;
  } 
  vel_output_file.close();*/
  
  for(auto it_temp_ = twist2_vec_.begin() + 1; it_temp_ <= it_base2; it_temp_++)
  {
    double ref_max_time = it_temp_->timestamp;
    for(auto it_ref = (it_temp_ - 1)->timestamp; it_ref <= ref_max_time; it_ref += output_dt)
    {

      auto pose = trajectory_->GetRadarPose(it_ref);
      Eigen::Vector3d linear_velocity = trajectory_->GetTransVelWorld(it_ref);
      // Eigen::Vector3d linear_velocity = pose.rotationMatrix().transpose() * trajectory_->GetTransVelWorld(it_temp_->timestamp);    // radar velocity
      Eigen::Vector3d angular_velocity = trajectory_->GetRotVelBody(it_ref);

      // 积分

      LOG(ERROR) << "Estimate Velocity:\n"
                << "it_ref = \t" << it_ref <<"\n"
                << "origin linear velocity = \t" << it_temp_->linear_vel_vec_(0) << ", " << it_temp_->linear_vel_vec_(1) << ", " << it_temp_->linear_vel_vec_(2) << "\n"
                << "estimate linear velocity = \t" << linear_velocity(0) << ", " << linear_velocity(1) << ", " << linear_velocity(2) << "\n"
                << "origin angular velocity = \t" << it_temp_->angular_vel_vec_(0) << ", " << it_temp_->angular_vel_vec_(1) << ", " << it_temp_->angular_vel_vec_(2) << "\n"
                << "estimate angular velocity = \t" << angular_velocity(0) << ", " << angular_velocity(1) << ", " << angular_velocity(2) << "\n" 
                << "point cloud size = " << it_temp_->point_cloud.width << "\n"
                << "inliers size = " << it_temp_->best_inliers.size() << "\n"
                << "flow size = " << it_temp_->best_flow.size() << "\n"
                << "origin linear norm = " << it_temp_->linear_vel_vec_.norm() << "\n"
                << "linear norm = " << linear_velocity.norm() << "\n";


      auto bias = GetLatestBias();
      // auto pose = trajectory_->GetRadarPose(it->timestamp);
      // Eigen::Vector3d linear_velocity = pose.rotationMatrix().transpose() * trajectory_->GetTransVelWorld(it->timestamp);    // radar velocity
      // Eigen::Vector3d angular_velocity = trajectory_->GetRotVelBody(it->timestamp);  
      Eigen::Quaterniond q_extract(pose.rotationMatrix());
      // Eigen::Vector3d t_extract = pose.translation();
      Eigen::Vector3d t_extract = pose.rotationMatrix() * pose.translation();
      pose.translation() = t_extract;
      ExtrinsicParam Extrin_e_r = trajectory_->GetSensorEP(EventSensor);
      Eigen::Matrix4d T_e_r = Extrin_e_r.Get_T();
      double timeoffset_e_r = Extrin_e_r.Get_Timeoffset();
      double pose_time = it_ref + relative_start_time;

      Save_Result(pose_time, pose, bias, linear_velocity, angular_velocity);
    
      // debug: 是否能从多普勒点云恢复速度估计
      /*{
        const double twist_timestamp = it_temp_->timestamp;
        LOG(ERROR) << "twist time = " << twist_timestamp << ", maxTime = " << trajectory_->maxTime(RadarSensor) << " dt = " << trajectory_->getDt() << std::endl;
        LOG(ERROR) << "twist in " << (int)((twist_timestamp - last_max_time) / trajectory_->getDt());

        LOG(ERROR) << "Estimate Velocity INFO:" << std::endl;
        sensor_msgs::PointCloud2ConstIterator<float> iter_x(it_temp_->point_cloud, "x");
            sensor_msgs::PointCloud2ConstIterator<float> iter_y(it_temp_->point_cloud, "y");
            sensor_msgs::PointCloud2ConstIterator<float> iter_z(it_temp_->point_cloud, "z");
            sensor_msgs::PointCloud2ConstIterator<float> iter_doppler(it_temp_->point_cloud, "doppler");

            // std::vector<Eigen::Vector3d> pt_vec;
            // std::vector<double> pt_doppler_vec;
            Eigen::MatrixXd M_A(it_temp_->point_cloud.width,3);
            Eigen::VectorXd M_b(it_temp_->point_cloud.width);
            for (int i = 0; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z, ++iter_doppler, i++) {
                //
                Eigen::Vector3d pt;
                pt << *iter_x, *iter_y, *iter_z;
                // 提取当前点的 doppler 值
                float pt_doppler = *iter_doppler;

                M_A.block<1,3>(i, 0) = pt.normalized().transpose();
                M_b(i) = pt_doppler;
            }
            LOG(ERROR) << "M_A = " << M_A << std::endl;
            LOG(ERROR) << "M_b = " << M_b << std::endl;

            Eigen::Vector3d lsq_vel;
            Eigen::HouseholderQR<Eigen::MatrixXd> qr(M_A);
            lsq_vel = qr.solve(- M_b);

            LOG(ERROR) << "lsq_vel = " << lsq_vel.transpose() << std::endl;
      }*/

      // std::fstream norm_vel("/home/hao/Desktop/radar-event-new/src/TwistEstimator/output/norm.csv", std::ios::out | std::ios::app); 
      // norm_vel << linear_velocity.norm() << std::endl;
      // norm_vel.close();

      LOG(ERROR) << "Publish Twist at: " << pose_time << std::endl;
      // 机体系下的速度发布
      geometry_msgs::TwistWithCovarianceStamped twist_esti_;
      twist_esti_.header.frame_id = "estimate";
      double twist_timestamp = pose_time;
      twist_esti_.header.stamp.fromSec(twist_timestamp);

      twist_esti_.twist.twist.linear.x = linear_velocity(0);
      twist_esti_.twist.twist.linear.y = linear_velocity(1);
      twist_esti_.twist.twist.linear.z = linear_velocity(2);

      twist_esti_.twist.twist.angular.x = angular_velocity(0);
      twist_esti_.twist.twist.angular.y = angular_velocity(1);
      twist_esti_.twist.twist.angular.z = angular_velocity(2);

      pub_spline_twist_.publish(twist_esti_);
    }
  }

 // 1-24 修改  
 /*
 double dt = trajectory_->getDt();
  {
    auto bias = GetLatestBias();
    // PublishLatestBias(bias.omega_bias, bias.vel_bias);
    LOG(ERROR) << "check output time = " << cur_time - last_time << "knot time = " << dt << std::endl;
    LOG(ERROR) << "check output condition = " << last_time + 0.5 * dt << " util = " << cur_time << std::endl;
    for(double time = last_time + 0.5 * dt; time < cur_time; time += (0.5 * dt))
    {
      auto pose = trajectory_->GetRadarPose(time);

      auto linear_velocity = pose.rotationMatrix().transpose() * trajectory_->GetTransVelWorld(time);    // radar velocity
      auto angular_velocity = trajectory_->GetRotVelBody(time);  

      // Eigen::Quaterniond q_extract = pose.rotationQuaternion();
      Eigen::Quaterniond q_extract(pose.rotationMatrix());
      Eigen::Vector3d t_extract = pose.translation();
      
      // extrincs parameter event w.r.t to radar
      ExtrinsicParam Extrin_e_r = trajectory_->GetSensorEP(EventSensor);
      Eigen::Matrix4d T_e_r = Extrin_e_r.Get_T();
      double timeoffset_e_r = Extrin_e_r.Get_Timeoffset();

      LOG(ERROR) << "Estimate Result:\n" 
                << "timestamp = \t" << time + trajectory_->GetDataStartTime() << "\n"
                << "position = \t" << t_extract.transpose() << "\n"
                << "quaternion = \t" << q_extract.coeffs().transpose() << "\n"
                << "linear velocity = \t" << linear_velocity.transpose() << "\n"
                << "angular velocity = \t" << angular_velocity.transpose() << "\n"
                << "omega_bias = \t" << Eigen::Vector3d(para_bw_vec[1]).transpose() << "\n"
                << "linear_bias = \t" << Eigen::Vector3d(para_bv_vec[1]).transpose() << "\n"
                << "T_e_r = \t" << T_e_r << "\n"
                << "time_offset = \t" << timeoffset_e_r << std::endl;

      // ceres_debug_path
      // for estimation residual
      estimator->GetResidualSummary().PrintSummary(trajectory_->minTimeNs(),
                                                  trajectory_->getDtNs());          

      double pose_time = time + relative_start_time;
      Save_Result(pose_time, pose, bias, linear_velocity, angular_velocity);
    }

  }
  */
  
  // /// 边缘化
  std::chrono::time_point<std::chrono::high_resolution_clock> time7 = std::chrono::high_resolution_clock::now();
  
  // if(doppler_factors_num > 0)
  /*{
    LOG(ERROR) << " Start to Update Prior " << std::endl;
    // UpdatePrior2();
    UpdatePriorLoose();
    LOG(ERROR) << " Update Prior " << std::endl;
  }*/
  // }   /// 边缘化

  std::chrono::time_point<std::chrono::high_resolution_clock> time8 = std::chrono::high_resolution_clock::now();
  /// 更新下一个窗口
  {
    LOG(ERROR) << "\nShoule Update Time: " << std::setprecision(9) << traj_max_time << ", "
               << "\nActual Update Time: " <<  update_time << ", "  
               << "\nSetForce Time: " << update_time << ", " // it->timestamp << ", "  
               << "\nRadar Min Time: " << trajectory_->minTime(RadarSensor) << ", " 
               << "\nRadar Max Time: " << trajectory_->maxTime(RadarSensor) << ", " 
               << "\nTraj Min Time: " << trajectory_->minTime() << ", " 
               << "\nTraj Max Time: " << trajectory_->maxTime() << ", " 
               << std::endl;           

    // 更新轨迹时间
    // trajectory_->UpdateActiveTime(update_time);
    trajectory_->SetActiveTime(update_time);
    LOG(ERROR) << " Update Active Time " << update_time << std::endl;

    trajectory_->SetForcedFixedTime(it->timestamp);         // 毫米波雷达无畸变,只有一个更新时间
    LOG(ERROR) << " Set Forced Fixed Time " << update_time << std::endl;

    // HAO Debug: 检查更新和固定时间
    LOG(ERROR) << "activate time = " << trajectory_->GetActiveTime()  << std::endl;
    LOG(ERROR) << "forcefixed time = " << trajectory_->GetForcedFixedTime()  << std::endl;

    // 时间更新
    LOG(ERROR) << " Update Time " << std::setprecision(9) << last_time << "  |-->  " << cur_time << std::endl;
    LOG(ERROR) << " Duration Time " << std::setprecision(9) << cur_time - last_time << std::endl;
    last_time = cur_time;

    // 偏置更新
    all_twist_bias_[last_time].omega_bias = Eigen::Vector3d(para_bw_vec[0]);
    all_twist_bias_[last_time].vel_bias = Eigen::Vector3d(para_bv_vec[0]);
    all_twist_bias_[cur_time].omega_bias = Eigen::Vector3d(para_bw_vec[1]);
    all_twist_bias_[cur_time].vel_bias = Eigen::Vector3d(para_bv_vec[1]);

    PublishTrajectoryAndMap(trajectory_, trajectory_->minTime(RadarSensor), 
                            trajectory_->maxTime(RadarSensor), trajectory_->getDt() * 2);


    // 插值状态
    // 删除使用后的速度信息
    LOG(ERROR) << " twist2_vec_.before_size() = " << twist2_vec_.size() << std::endl;
    LOG(ERROR) << " it_base2 index = " << it_base2 - twist2_vec_.begin() + 1 << std::endl;
    if(it_base2 == twist2_vec_.begin())
      twist2_vec_.erase(twist2_vec_.begin());
    else
      twist2_vec_.erase(twist2_vec_.begin(), it_base2 + 1);
    // ((it == twist2_vec_.end())? twist2_vec_.end(): it + 1));

    LOG(ERROR) << " twist2_vec_.size() = " << twist2_vec_.size() << std::endl;
    LOG(ERROR) << " all_twist_bias_.size() = " << all_twist_bias_.size() << std::endl;
  }
    
  std::chrono::time_point<std::chrono::high_resolution_clock> end_time = std::chrono::high_resolution_clock::now();
  {
      std::chrono::duration<double, std::milli> elapsed;
      elapsed = end_time - start_time;
      LOG(ERROR) << "Total Time: " << elapsed.count() << std::endl;
      elapsed = time1 - start_time;
      LOG(ERROR) << "Segment Data: " << elapsed.count() << std::endl;
      elapsed = time3 - time1;
      LOG(ERROR) << "Optimize: " << elapsed.count() << std::endl;
      elapsed = time4 - time3;
      LOG(ERROR) << "Construct Marginize: " << elapsed.count() << std::endl;
      elapsed = time5 - time4;
      LOG(ERROR) << "Construct: " << elapsed.count() << std::endl;
      elapsed = time6 - time5;
      LOG(ERROR) << "Solve: " << elapsed.count() << std::endl;
      elapsed = time7 - time6;
      LOG(ERROR) << "Publish Result: " << elapsed.count() << std::endl;
      elapsed = time8 - time7;
      LOG(ERROR) << "Marginize: " << elapsed.count() << std::endl;
      elapsed = end_time - time8;
      LOG(ERROR) << "Update Time: " << elapsed.count() << std::endl;

      elapsed = end_time - start_time;
      std::fstream estimate_time("/home/hao/Desktop/twist_ws/src/TwistEstimator/output/estimate_T.csv", std::ios::out | std::ios::app);
      estimate_time << elapsed.count() << std::endl;
      estimate_time.close();
  }

  // /// for debug
  // {
  //   std::cout << "Pause press any key to continue, 'q' or 'Q' for exit!" << std::endl;
  //   char ch = std::getchar();
  //   // 检查用户是否按下 'q' 键
  //   if (ch == 'q' || ch == 'Q') {
  //       std::cout << "Exiting TwistEstimator..." << std::endl;
  //       ros::shutdown();  // 关闭 ROS 系统
  //       exit(0);  // 退出程序
  //   }
  // }/// debug

  /// compare for gt

}


// void Estimate2(TwistData2& twist)
// 每次估计 一定时间内的 轨迹, 轨迹会重置
Trajectory::Ptr local_trajectory_;
bool local_initial = false;
void Local_Estimate2()
{
  LOG(ERROR) << "  ----------------- Estimate -------------------- " << std::endl;
  std::chrono::time_point<std::chrono::high_resolution_clock> start_time = std::chrono::high_resolution_clock::now();

  if(!initial_done)
  {
    // local_trajectory_ = std::make_shared<Trajectory>(knot_distance, 0);
    // local_trajectory_->SetSensorExtrinsics(SensorType::RadarSensor, EP_RtoI);
    // local_trajectory_->SetSensorExtrinsics(SensorType::EventSensor, EP_EtoI);
    return ;
  }

  if(!local_initial)
  {
    local_trajectory_->SetDataStartTime(trajectory_->GetDataStartTime());
    local_initial = true;
  }
  // [step1] 设置系统初始时间
  // LOG(ERROR) << "trajectory_->GetDataStartTime() = " 
  //           << std::setprecision(20) << trajectory_->GetDataStartTime() << std::endl;
  // if (trajectory_->GetDataStartTime() < 0) {
  //   trajectory_->SetDataStartTime(twist.timestamp);
  //   relative_start_time = twist.timestamp;

  //   std::cout << YELLOW << " trajectory set start time: " << std::setprecision(18) << twist.timestamp << std::endl;
  //   LOG(ERROR) << " trajectory set start time:  " << std::setprecision(18) << twist.timestamp << std::endl;

  //   LOG(ERROR) << "first traj_max_time = " << trajectory_->maxTime(RadarSensor) << std::endl;
  // }

  // [step2] 转换数据时间
  // double data_start_time = trajectory_->GetDataStartTime();
  // if(!twist.is_relative_time)
  // {
  //   twist.timestamp -= relative_start_time; //转换到相对时间
  //   twist.is_relative_time = true;
  // }

  // HAO TODO: 这部分可以修改为 前后端的多线程
  // [step3] 缓冲区
  // twist2_vec_.push_back(twist);
  // assert(twist.is_relative_time);   // 确保是相对时间
  if(twist2_vec_.empty())
  {
    LOG(ERROR) << "twist2_vec_.empty !" << std::endl;
    return;
  }
  LOG(ERROR) << "twist2_vec_.size = " << twist2_vec_.size() << std::endl;

  // [step4] 根据样条段，分割当前区间
  // double last_min_time = trajectory_->minTime(RadarSensor);  
  // double last_max_time = trajectory_->maxTime(RadarSensor);

  
  double last_min_time = local_trajectory_->minTime(RadarSensor);  
  double last_max_time = local_trajectory_->maxTime(RadarSensor);
  static long int spline_counts = 0;
  LOG(ERROR) << "local_trajectory_ time in [" << last_min_time << ", " << last_max_time << "] " << std::endl;

  if(last_max_time - last_min_time > local_dt)  // 超过局部范围,需要重置
  {
    LOG(ERROR) << "last_max_time - last_min_time" << last_max_time - last_min_time 
                << ", local_dt = " << local_dt << std::endl;  
    /*{
      std::fstream local_pose("/home/hao/Desktop/radar-event-new/src/TwistEstimator/output/local_pose.csv", std::ios::out | std::ios::app);
      local_pose << "Spline" << "," << std::setprecision(18) <<relative_start_time + last_end_time << std::endl;
      // 保存上次的信息(相对启始时刻的)
      for(auto it_ref = last_min_time; it_ref < last_max_time; it_ref += 0.02)
      {
        auto bias = GetLatestBias();
        auto pose = local_trajectory_->GetRadarPose(it_ref);
        Eigen::Vector3d linear_velocity = pose.rotationMatrix().transpose() * local_trajectory_->GetTransVelWorld(it_ref);    // radar velocity
        Eigen::Vector3d angular_velocity = local_trajectory_->GetRotVelBody(it_ref);  
        // Eigen::Vector3d linear_velocity = pose.rotationMatrix().transpose() * trajectory_->GetTransVelWorld(it->timestamp);    // radar velocity
        // Eigen::Vector3d angular_velocity = trajectory_->GetRotVelBody(it->timestamp);  
        Eigen::Quaterniond q(pose.rotationMatrix());
        // Eigen::Vector3d t_extract = pose.translation();
        Eigen::Vector3d p = pose.rotationMatrix() * pose.translation();   // body0
        // pose.translation() = t_extract;
        ExtrinsicParam Extrin_e_r = local_trajectory_->GetSensorEP(EventSensor);
        Eigen::Matrix4d T_e_r = Extrin_e_r.Get_T();
        double timeoffset_e_r = Extrin_e_r.Get_Timeoffset();
        double pose_time = it_ref + relative_start_time + last_end_time;

        local_pose << std::setprecision(18) << pose_time << ", ";  // 设置高精度输出时间戳
        local_pose << p(0) << ", " << p(1) << ", " << p(2) << ", ";  // 平移向量
        local_pose << q.x() << ", " << q.y() << ", " << q.z() << ", " << q.w() << ", ";  // 四元数
        local_pose << bias.omega_bias(0) << ", " << bias.omega_bias(1) << ", " << bias.omega_bias(2) << ", ";  // 角速度偏置
        local_pose << bias.vel_bias(0) << ", " << bias.vel_bias(1) << ", " << bias.vel_bias(2) << ", ";
        local_pose << linear_velocity(0) << ", " << linear_velocity(1) << ", " << linear_velocity(2) << ", ";
        local_pose << angular_velocity(0) << ", " << angular_velocity(1) << ", " << angular_velocity(2) << std::endl;  // 速度偏置    
      }
      local_pose.close();
    }*/

    {
      std::string path_dir = "/home/hao/Desktop/twist_ws/src/TwistEstimator/output/local_pose";
      std::fstream local_pose(path_dir + std::to_string(spline_counts++) + ".tum", std::ios::out | std::ios::app);
      // local_pose << "Spline" << "," << std::setprecision(18) <<relative_start_time + last_end_time << std::endl;
      // 保存上次的信息(相对启始时刻的)
      for(auto it_ref = last_min_time; it_ref < last_max_time; it_ref += 0.02)
      {
        auto bias = GetLatestBias();
        auto pose = local_trajectory_->GetRadarPose(it_ref);
        Eigen::Vector3d linear_velocity = pose.rotationMatrix().transpose() * local_trajectory_->GetTransVelWorld(it_ref);    // radar velocity
        Eigen::Vector3d angular_velocity = local_trajectory_->GetRotVelBody(it_ref);  
        // Eigen::Vector3d linear_velocity = pose.rotationMatrix().transpose() * trajectory_->GetTransVelWorld(it->timestamp);    // radar velocity
        // Eigen::Vector3d angular_velocity = trajectory_->GetRotVelBody(it->timestamp);  
        Eigen::Quaterniond q(pose.rotationMatrix());
        // Eigen::Vector3d t_extract = pose.translation();
        Eigen::Vector3d p = pose.rotationMatrix() * pose.translation();   // body0
        // pose.translation() = t_extract;
        ExtrinsicParam Extrin_e_r = local_trajectory_->GetSensorEP(EventSensor);
        Eigen::Matrix4d T_e_r = Extrin_e_r.Get_T();
        double timeoffset_e_r = Extrin_e_r.Get_Timeoffset();
        double pose_time = it_ref + relative_start_time + last_end_time - 1716196036.30958; // dji的起始时间

        local_pose << std::setprecision(18) << pose_time << " ";  // 设置高精度输出时间戳
        local_pose << p(0) << " " << p(1) << " " << p(2) << " ";  // 平移向量
        local_pose << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << std::endl;  // 四元数
      }
      local_pose.close();
    }

    // 初始化
    // local_trajectory_ = std::make_shared<Trajectory>(trajectory_->getDt(), last_max_time);   // last_max_time 是否定义到0时间内
    // local_trajectory_->SetSensorExtrinsics(RadarSensor, trajectory_->GetSensorEP(RadarSensor));
    // local_trajectory_->SetSensorExtrinsics(EventSensor, trajectory_->GetSensorEP(EventSensor));

    local_trajectory_ = std::make_shared<Trajectory>(trajectory_->getDt(), 0);
    local_trajectory_->SetSensorExtrinsics(SensorType::RadarSensor, trajectory_->GetSensorEP(RadarSensor));
    local_trajectory_->SetSensorExtrinsics(SensorType::EventSensor, trajectory_->GetSensorEP(EventSensor));  
    for (auto& twist: twist2_vec_)
    {
      twist.timestamp = twist.timestamp - last_max_time; // 重新做时间戳
    }

    double last_update_end_time = last_end_time;
    last_end_time += last_max_time;
    local_trajectory_->SetDataStartTime(last_end_time);

    LOG(ERROR) << "Update last_end_time = " << std::setprecision(6) << "[ " 
              << last_update_end_time << ", " << last_end_time <<  " ]" << last_end_time << std::endl;

    // LOG(ERROR) << "Update last_end_time = " << std::setprecision(6) << last_end_time << std::endl;

    // 全局是否更新
  }

  LOG(ERROR) << "twist2_vec_.size = " << twist2_vec_.size() << std::endl;
  double traj_max_time = last_max_time + t_add; // 单位: 秒
  LOG(ERROR) << "last traj_max_time = " << last_max_time << std::endl;
  LOG(ERROR) << "traj_max_time = " << traj_max_time << std::endl;
  LOG(ERROR) << "twist2_vec_.back().timestamp = " << twist2_vec_.back().timestamp << std::endl;
  if(twist2_vec_.back().timestamp < traj_max_time)
  {
    LOG(ERROR) << "time is not enough" << std::endl;
    return;
  }

  for(auto& twis : twist2_vec_)
  {
    LOG(ERROR) << "loop timestamp = " << twis.timestamp << std::endl;
  }

  auto it = std::find_if(twist2_vec_.rbegin(), twist2_vec_.rend(), 
                      [traj_max_time](const TwistData2& data) {
                          return data.timestamp < traj_max_time;
                      });
  it_base2 = ((it == twist2_vec_.rend())? twist2_vec_.begin() : it.base() - 1); // it 的正向跌代器

  SE3d last_knot = local_trajectory_->getLastKnot();
  if(it == twist2_vec_.rend())  // 没有合理的数据,直接返回,这种一般是存在缺失值
  {
    LOG(ERROR) << " Skip " << std::endl;
    local_trajectory_->extendKnotsTo(traj_max_time * S_TO_NS, last_knot); // 扩展到最新的数据

    local_trajectory_->SetActiveTime(traj_max_time);
    LOG(ERROR) << " Update Active Time " << traj_max_time << std::endl;

    local_trajectory_->SetForcedFixedTime(traj_max_time);         // 毫米波雷达无畸变,只有一个更新时间
    LOG(ERROR) << " Set Forced Fixed Time " << traj_max_time << std::endl;

    // assert(it_base2 == twist2_vec_.begin());
    // {
    //   if(it_base2 == twist2_vec_.begin())
    //       twist2_vec_.erase(twist2_vec_.begin());
    //   else
    //       twist2_vec_.erase(twist2_vec_.begin(), it_base2 + 1);
    // }
    return;
  }

  LOG(ERROR) << "it in " << it_base2 - twist2_vec_.begin() + 1 << std::endl;
  LOG(ERROR) << "it->timestamp = " << it->timestamp << std::endl;
  LOG(ERROR) << "it_base2->timestamp = " << it_base2->timestamp << std::endl;
  assert(it->timestamp == it_base2->timestamp && "Not Same Place");

  // 确定终点
  cur_time = it_base2->timestamp;
  all_twist_bias_[cur_time] = all_twist_bias_[last_time];
  double update_time = it->timestamp;  // 注意是相对时间的 秒
  
  // LOG(ERROR) << "CHECK max time 1 = " << trajectory_->maxTime(RadarSensor) << std::endl;
  LOG(ERROR) << "it->timestamp * S_TO_NS = " << it->timestamp * S_TO_NS << std::endl;
  local_trajectory_->extendKnotsTo(it->timestamp * S_TO_NS, last_knot); // 扩展到最新的数据
  // LOG(ERROR) << "CHECK max time 2 = " << trajectory_->maxTime(RadarSensor) << std::endl;

  // Debug: 某端时间是合理的
  /*{
    LOG(ERROR) << " trajectory_ extend knots timestamp: " << std::setprecision(9) << it->timestamp << std::endl;
    LOG(ERROR) << " check update timestamp: " << std::setprecision(9) << it->timestamp - traj_max_time << std::endl;  // should be <
    // 如果下一个存在,检查一下下一个的时间戳
    if(it_base2 != twist2_vec_.end() && (it_base2 + 1) != twist2_vec_.end())
    {
      LOG(ERROR) << " trajectory_ extend knots timestamp: " << std::setprecision(9) << (it + 1)->timestamp << std::endl;
      LOG(ERROR) << " check update timestamp: " << std::setprecision(9) << (it - 1)->timestamp - traj_max_time << std::endl;
    }
  }*/

  std::map<int, double*> para_bw_vec;
  std::map<int, double*> para_bv_vec;
  // 分配内存并深拷贝 omega_bias 数据
  para_bw_vec[0] = new double[3];
  std::memcpy(para_bw_vec[0], all_twist_bias_.at(last_time).omega_bias.data(), 3 * sizeof(double));

  para_bw_vec[1] = new double[3];
  std::memcpy(para_bw_vec[1], all_twist_bias_.at(cur_time).omega_bias.data(), 3 * sizeof(double));

  // 分配内存并深拷贝 vel_bias 数据
  para_bv_vec[0] = new double[3];
  std::memcpy(para_bv_vec[0], all_twist_bias_.at(last_time).vel_bias.data(), 3 * sizeof(double));

  para_bv_vec[1] = new double[3];
  std::memcpy(para_bv_vec[1], all_twist_bias_.at(cur_time).vel_bias.data(), 3 * sizeof(double));

  std::chrono::time_point<std::chrono::high_resolution_clock> time1 = std::chrono::high_resolution_clock::now();

  // [step5] 创建优化器
  TrajectoryEstimatorOptions option;
  option.lock_EPs.at(EventSensor).Unlock();
  option.is_marg_state = false;
  option.show_residual_summary = true;
  TrajectoryEstimator::Ptr estimator(new TrajectoryEstimator(local_trajectory_, option, "Update Traj"));
  LOG(ERROR) << " start estimate " << std::endl;

  

  // decide prior index
  double opt_min_time = twist2_vec_.begin()->timestamp;
  int opt_idx = local_trajectory_->GetCtrlIndex(opt_min_time * S_TO_NS);
  option.ctrl_to_be_opt_now = std::min(prior_ctrl_id.first, opt_idx);
  double opt_max_time = it_base2->timestamp;
  LOG(ERROR) << "opt_min_time = " << opt_min_time << ", opt_max_time = " << opt_max_time << std::endl;
  int scan_idx = local_trajectory_->GetCtrlIndex(opt_max_time * S_TO_NS);
  option.ctrl_to_be_opt_later = std::max(scan_idx, local_trajectory_->Get_N());
  LOG(ERROR) << "check ctrl later idx = std::max [" << scan_idx 
             << ", " << local_trajectory_->Get_N() << "]" << std::endl;
  
  // LOG(ERROR) << "Radar max time = " << trajectory_->maxTime(RadarSensor)  << std::endl;
  // LOG(ERROR) << "activate time = " << trajectory_->GetActiveTime()  << std::endl;
  // LOG(ERROR) << "forcefixed time = " << trajectory_->GetForcedFixedTime()  << std::endl;
  // LOG(ERROR) << "optimization time between [" << opt_min_time << ", " << opt_max_time << "]" << std::endl;
  // LOG(ERROR) << "optimization ctrl idx between [" << option.ctrl_to_be_opt_now 
  //            << ", " << option.ctrl_to_be_opt_later << "]" << std::endl;
  
  // HAO TODO: 这部分暂时不需要
  // TODO: 确定 prior_ctrl_id.first 的数值
  if (LocatedLocalInFirstSegment(opt_min_time)) {
    // estimator->SetFixedIndex(trajectory_->Get_N() - 1);
    LOG(ERROR) << "estimator->SetFixedIndex = " << local_trajectory_->Get_N() - 1 << std::endl;
    // LOG(ERROR) << "estimator->SetFixedIndex = " << trajectory_->Get_N() - 1 << std::endl;
    // estimator->AddStartTimePose(original_pose_);
  } else {
    estimator->SetFixedIndex(prior_ctrl_id.first - 1);
    LOG(ERROR) << "estimator->SetFixedIndex = " << prior_ctrl_id.first - 1 << std::endl;
    LOG(ERROR) << "local_trajectory_.fixed_control_point_index_ = " << estimator->GetFixedControlIndex() << std::endl;

    // trajectory_->opt_min_init_time_tmp = opt_min_time;
    // trajectory_->opt_min_init_time_tmp = last_time;
    // trajectory_->opt_init_fixed_idx_tmp = estimator->GetFixedControlIndex();
  }

  // [step5] 因子图优化
  /// 因子图优化
  std::chrono::time_point<std::chrono::high_resolution_clock> time3;
  std::chrono::time_point<std::chrono::high_resolution_clock> time4;
  std::chrono::time_point<std::chrono::high_resolution_clock> time5;
  long int doppler_factors_num = 0;
  long int event_flow_factors_num = 0;

    // 雷达线速度必须存在
    time5 = std::chrono::high_resolution_clock::now();

    // LOG(ERROR) << " Add all factor into the solver " << std::endl;
    // LOG(ERROR) << "Estimator: Add Doppler Factors for " << doppler_factors_num << std::endl;
    // LOG(ERROR) << "Estimator: Add EventFlow Factors for " << event_flow_factors_num << std::endl;
    // LOG(ERROR) << "Estimator: Add Bias Factors for " << 1 << std::endl;
  //}/// 因子图优化

  // 松耦合
  {
    // [step5-1] 优化问题建模
    {
      time3 = std::chrono::high_resolution_clock::now();
      // assert(marg_info == nullptr);
      LOG(ERROR) << "Estimation marg_info = " << ((marg_info)? "True" : "False") << std::endl;
      // marg_info 后面会一直存在
      // [step5-0] 先验因子
      prior_ctrl_id.first = option.ctrl_to_be_opt_later;
      prior_ctrl_id.second = local_trajectory_->numKnots() - 1;
      if (true && marg_info) {
        // 确定下一次边缘化的部分
        prior_ctrl_id.first = option.ctrl_to_be_opt_later;
        prior_ctrl_id.second = local_trajectory_->numKnots() - 1;

        LOG(ERROR) << "[Prior Ctrl ID] = [" << prior_ctrl_id.first << ","
            << prior_ctrl_id.second << "] "
            << prior_ctrl_id.second - prior_ctrl_id.first;

        estimator->AddMarginalizationFactor(marg_info,
                                            marg_parameter_blocks);
        LOG(ERROR) << " Add Marginalize " << std::endl;
      }
      
      time4 = std::chrono::high_resolution_clock::now();
      // 根据量测构建因子图优化(Loop Optimization)
      Eigen::Matrix3d R_r_e = local_trajectory_->GetSensorEP(EventSensor).q.toRotationMatrix();
      LOG(ERROR) << " Twist Size = " << (it_base2 - twist2_vec_.begin() + 1) << std::endl;
      // LOG(ERROR) << " Aver Freq = " << 1.0 / (it_base2->timestamp - twist2_vec_.begin()->timestamp) / (it_base2 - twist2_vec_.begin() + 1) << " Hz" << std::endl;

      for(auto it_temp_ = twist2_vec_.begin(); it_temp_ <= it_base2; it_temp_++)
      {
        // [step5-1] 多普勒残差
        const double twist_timestamp = it_temp_->timestamp;
        LOG(ERROR) << "twist time = " << twist_timestamp << ", maxTime = " << local_trajectory_->maxTime(RadarSensor) << " dt = " << trajectory_->getDt() << std::endl;
        LOG(ERROR) << "twist in " << (int)((twist_timestamp - last_max_time) / local_trajectory_->getDt());
        {
          
          estimator->AddBodyLocalVelocityMeasurementAnalytic(twist_timestamp, para_bv_vec[1], 
                                                          it_temp_->linear_vel_vec_, linear_weight, linear_w_weight, R_weight, false);

          estimator->AddBodyLocalAngularVelocityMeasurementAnalytic(twist_timestamp, para_bw_vec[1], 
                                                          it_temp_-> angular_vel_vec_, omega_weight, omega_w_weight, false);                                            
        }

      }

      // [3] Twist Bias 估计
      // Bias 只跟两次量测时间相关，而和具体量测数量无关
      {
        double delta_time = cur_time - last_time;
        delta_time = std::max(delta_time, 1e-3);

        // double dt = 1. / 200.; // opt_weight_.imu_noise.imu_frequency;
        // double dt = 1. / 10.; // opt_weight_.radar_frequency;
        double dt = 1. / 20.; // opt_weight_.radar_frequency;
        double cov = delta_time / dt * (dt * dt);
        LOG(ERROR) << "delta_time = " << delta_time << std::endl;
        LOG(ERROR) << "cov = " << cov << std::endl;
        Eigen::Matrix<double, 6, 1> sqrt_info;
        sqrt_info.setOnes();
        sqrt_info *=  (1. / std::sqrt(cov)); // * opt_weight_.bias_info_vec;

        LOG(ERROR) << "100 * sqrt_info = " << 100 * sqrt_info << std::endl;

        estimator->AddBiasFactor(para_bw_vec[0], para_bw_vec[1], 
                                  para_bv_vec[0], para_bv_vec[1], 
                                  1, 100 * sqrt_info); // 100 * sqrt_info);

        LOG(ERROR) << " Add Bias Factor " << std::endl;                          
      }

      LOG(ERROR) << " Add all factor into the solver " << std::endl;
      LOG(ERROR) << "Estimator: Add Doppler Factors for " << doppler_factors_num << std::endl;
      LOG(ERROR) << "Estimator: Add EventFlow Factors for " << event_flow_factors_num << std::endl;
      LOG(ERROR) << "Estimator: Add Bias Factors for " << 1 << std::endl;
    }  

    // 雷达线速度必须存在
    time5 = std::chrono::high_resolution_clock::now();
    // if(doppler_factors_num > 0)
    // {   
      // [step5-2] 优化问题求解
      {
        LOG(ERROR) << " start to solve problem " << std::endl;

        // ceres::Solver::Summary summary = estimator->Solve(50, false);
        ceres::Solver::Summary summary = estimator->Solve(50, false, 8);  // use multi-thread
        LOG(ERROR) << summary.BriefReport();
        LOG(ERROR) << "Traj Update Successful/Unsuccessful steps: "
                  << summary.num_successful_steps << "/"
                  << summary.num_unsuccessful_steps;
        LOG(ERROR) << "Traj Update details: ";
        LOG(ERROR) << summary.FullReport();

        LOG(ERROR) << "end to solve problem " << std::endl;
      }
    // }
  }/// 因子图优化*/




  /// 发布结果
  std::chrono::time_point<std::chrono::high_resolution_clock> time6 = std::chrono::high_resolution_clock::now();
  /*
  for(auto it_temp_ = twist2_vec_.begin() + 1; it_temp_ <= it_base2; it_temp_++)
  {
    double ref_max_time = it_temp_->timestamp;
    for(auto it_ref = (it_temp_ - 1)->timestamp; it_ref <= ref_max_time; it_ref += output_dt)
    {

      auto pose = local_trajectory_->GetRadarPose(it_ref);
      Eigen::Vector3d linear_velocity = local_trajectory_->GetTransVelWorld(it_ref);
      // Eigen::Vector3d linear_velocity = pose.rotationMatrix().transpose() * trajectory_->GetTransVelWorld(it_temp_->timestamp);    // radar velocity
      Eigen::Vector3d angular_velocity = local_trajectory_->GetRotVelBody(it_ref);

      // 积分

      LOG(ERROR) << "Estimate Velocity:\n"
                << "it_ref = \t" << it_ref <<"\n"
                << "origin linear velocity = \t" << it_temp_->linear_vel_vec_(0) << ", " << it_temp_->linear_vel_vec_(1) << ", " << it_temp_->linear_vel_vec_(2) << "\n"
                << "estimate linear velocity = \t" << linear_velocity(0) << ", " << linear_velocity(1) << ", " << linear_velocity(2) << "\n"
                << "origin angular velocity = \t" << it_temp_->angular_vel_vec_(0) << ", " << it_temp_->angular_vel_vec_(1) << ", " << it_temp_->angular_vel_vec_(2) << "\n"
                << "estimate angular velocity = \t" << angular_velocity(0) << ", " << angular_velocity(1) << ", " << angular_velocity(2) << "\n" 
                << "point cloud size = " << it_temp_->point_cloud.width << "\n"
                << "inliers size = " << it_temp_->best_inliers.size() << "\n"
                << "flow size = " << it_temp_->best_flow.size() << "\n"
                << "origin linear norm = " << it_temp_->linear_vel_vec_.norm() << "\n"
                << "linear norm = " << linear_velocity.norm() << "\n";


      auto bias = GetLatestBias();
      // auto pose = trajectory_->GetRadarPose(it->timestamp);
      // Eigen::Vector3d linear_velocity = pose.rotationMatrix().transpose() * trajectory_->GetTransVelWorld(it->timestamp);    // radar velocity
      // Eigen::Vector3d angular_velocity = trajectory_->GetRotVelBody(it->timestamp);  
      Eigen::Quaterniond q_extract(pose.rotationMatrix());
      // Eigen::Vector3d t_extract = pose.translation();
      Eigen::Vector3d t_extract = pose.rotationMatrix() * pose.translation();
      pose.translation() = t_extract;
      ExtrinsicParam Extrin_e_r = local_trajectory_->GetSensorEP(EventSensor);
      Eigen::Matrix4d T_e_r = Extrin_e_r.Get_T();
      double timeoffset_e_r = Extrin_e_r.Get_Timeoffset();
      double pose_time = it_ref + relative_start_time;

      Save_Result(pose_time, pose, bias, linear_velocity, angular_velocity);
    
      // debug: 是否能从多普勒点云恢复速度估计
      /*{
        const double twist_timestamp = it_temp_->timestamp;
        LOG(ERROR) << "twist time = " << twist_timestamp << ", maxTime = " << trajectory_->maxTime(RadarSensor) << " dt = " << trajectory_->getDt() << std::endl;
        LOG(ERROR) << "twist in " << (int)((twist_timestamp - last_max_time) / trajectory_->getDt());

        LOG(ERROR) << "Estimate Velocity INFO:" << std::endl;
        sensor_msgs::PointCloud2ConstIterator<float> iter_x(it_temp_->point_cloud, "x");
            sensor_msgs::PointCloud2ConstIterator<float> iter_y(it_temp_->point_cloud, "y");
            sensor_msgs::PointCloud2ConstIterator<float> iter_z(it_temp_->point_cloud, "z");
            sensor_msgs::PointCloud2ConstIterator<float> iter_doppler(it_temp_->point_cloud, "doppler");

            // std::vector<Eigen::Vector3d> pt_vec;
            // std::vector<double> pt_doppler_vec;
            Eigen::MatrixXd M_A(it_temp_->point_cloud.width,3);
            Eigen::VectorXd M_b(it_temp_->point_cloud.width);
            for (int i = 0; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z, ++iter_doppler, i++) {
                //
                Eigen::Vector3d pt;
                pt << *iter_x, *iter_y, *iter_z;
                // 提取当前点的 doppler 值
                float pt_doppler = *iter_doppler;

                M_A.block<1,3>(i, 0) = pt.normalized().transpose();
                M_b(i) = pt_doppler;
            }
            LOG(ERROR) << "M_A = " << M_A << std::endl;
            LOG(ERROR) << "M_b = " << M_b << std::endl;

            Eigen::Vector3d lsq_vel;
            Eigen::HouseholderQR<Eigen::MatrixXd> qr(M_A);
            lsq_vel = qr.solve(- M_b);

            LOG(ERROR) << "lsq_vel = " << lsq_vel.transpose() << std::endl;
      } // * /

      // std::fstream norm_vel("/home/hao/Desktop/radar-event-new/src/TwistEstimator/output/norm.csv", std::ios::out | std::ios::app); 
      // norm_vel << linear_velocity.norm() << std::endl;
      // norm_vel.close();

      LOG(ERROR) << "Publish Twist at: " << pose_time << std::endl;
      // 机体系下的速度发布
      geometry_msgs::TwistWithCovarianceStamped twist_esti_;
      twist_esti_.header.frame_id = "estimate";
      double twist_timestamp = pose_time;
      twist_esti_.header.stamp.fromSec(twist_timestamp);

      twist_esti_.twist.twist.linear.x = linear_velocity(0);
      twist_esti_.twist.twist.linear.y = linear_velocity(1);
      twist_esti_.twist.twist.linear.z = linear_velocity(2);

      twist_esti_.twist.twist.angular.x = angular_velocity(0);
      twist_esti_.twist.twist.angular.y = angular_velocity(1);
      twist_esti_.twist.twist.angular.z = angular_velocity(2);

      pub_spline_twist_.publish(twist_esti_);
    }
  } */ 

  // /// 边缘化
  std::chrono::time_point<std::chrono::high_resolution_clock> time7 = std::chrono::high_resolution_clock::now();
  
  // if(doppler_factors_num > 0)
  /*{
    LOG(ERROR) << " Start to Update Prior " << std::endl;
    // UpdatePrior2();
    UpdatePriorLoose();
    LOG(ERROR) << " Update Prior " << std::endl;
  }*/
  // }   /// 边缘化
  
  std::chrono::time_point<std::chrono::high_resolution_clock> time8 = std::chrono::high_resolution_clock::now();
  /// 更新下一个窗口
  {
    LOG(ERROR) << "\nShoule Update Time: " << std::setprecision(9) << traj_max_time << ", "
               << "\nActual Update Time: " <<  update_time << ", "  
               << "\nSetForce Time: " << update_time << ", " // it->timestamp << ", "  
               << "\nRadar Min Time: " << local_trajectory_->minTime(RadarSensor) << ", " 
               << "\nRadar Max Time: " << local_trajectory_->maxTime(RadarSensor) << ", " 
               << "\nTraj Min Time: " << local_trajectory_->minTime() << ", " 
               << "\nTraj Max Time: " << local_trajectory_->maxTime() << ", " 
               << std::endl;           

    // 更新轨迹时间
    // trajectory_->UpdateActiveTime(update_time);
    local_trajectory_->SetActiveTime(update_time);
    LOG(ERROR) << " Update Active Time " << update_time << std::endl;

    local_trajectory_->SetForcedFixedTime(it->timestamp);         // 毫米波雷达无畸变,只有一个更新时间
    LOG(ERROR) << " Set Forced Fixed Time " << update_time << std::endl;

    // HAO Debug: 检查更新和固定时间
    LOG(ERROR) << "activate time = " << local_trajectory_->GetActiveTime()  << std::endl;
    LOG(ERROR) << "forcefixed time = " << local_trajectory_->GetForcedFixedTime()  << std::endl;

    // 时间更新
    LOG(ERROR) << " Update Time " << std::setprecision(9) << last_time << "  |-->  " << cur_time << std::endl;
    LOG(ERROR) << " Duration Time " << std::setprecision(9) << cur_time - last_time << std::endl;
    last_time = cur_time;

    // 偏置更新
    all_twist_bias_[last_time].omega_bias = Eigen::Vector3d(para_bw_vec[0]);
    all_twist_bias_[last_time].vel_bias = Eigen::Vector3d(para_bv_vec[0]);
    all_twist_bias_[cur_time].omega_bias = Eigen::Vector3d(para_bw_vec[1]);
    all_twist_bias_[cur_time].vel_bias = Eigen::Vector3d(para_bv_vec[1]);

    /*PublishTrajectoryAndMap(local_trajectory_, local_trajectory_->minTime(RadarSensor), 
                            local_trajectory_->maxTime(RadarSensor), local_trajectory_->getDt() * 2);*/


    // 插值状态
    // 删除使用后的速度信息
    LOG(ERROR) << " twist2_vec_.before_size() = " << twist2_vec_.size() << std::endl;
    LOG(ERROR) << " it_base2 index = " << it_base2 - twist2_vec_.begin() + 1 << std::endl;
    if(it_base2 == twist2_vec_.begin())
      twist2_vec_.erase(twist2_vec_.begin());
    else
      twist2_vec_.erase(twist2_vec_.begin(), it_base2 + 1);
    // ((it == twist2_vec_.end())? twist2_vec_.end(): it + 1));

    LOG(ERROR) << " twist2_vec_.size() = " << twist2_vec_.size() << std::endl;
    LOG(ERROR) << " all_twist_bias_.size() = " << all_twist_bias_.size() << std::endl;
  }
    
  std::chrono::time_point<std::chrono::high_resolution_clock> end_time = std::chrono::high_resolution_clock::now();
  {
      std::chrono::duration<double, std::milli> elapsed;
      elapsed = end_time - start_time;
      LOG(ERROR) << "Total Time: " << elapsed.count() << std::endl;
      elapsed = time1 - start_time;
      LOG(ERROR) << "Segment Data: " << elapsed.count() << std::endl;
      elapsed = time3 - time1;
      LOG(ERROR) << "Optimize: " << elapsed.count() << std::endl;
      elapsed = time4 - time3;
      LOG(ERROR) << "Construct Marginize: " << elapsed.count() << std::endl;
      elapsed = time5 - time4;
      LOG(ERROR) << "Construct: " << elapsed.count() << std::endl;
      elapsed = time6 - time5;
      LOG(ERROR) << "Solve: " << elapsed.count() << std::endl;
      elapsed = time7 - time6;
      LOG(ERROR) << "Publish Result: " << elapsed.count() << std::endl;
      elapsed = time8 - time7;
      LOG(ERROR) << "Marginize: " << elapsed.count() << std::endl;
      elapsed = end_time - time8;
      LOG(ERROR) << "Update Time: " << elapsed.count() << std::endl;
  }

  // /// for debug
  // {
  //   std::cout << "Pause press any key to continue, 'q' or 'Q' for exit!" << std::endl;
  //   char ch = std::getchar();
  //   // 检查用户是否按下 'q' 键
  //   if (ch == 'q' || ch == 'Q') {
  //       std::cout << "Exiting TwistEstimator..." << std::endl;
  //       ros::shutdown();  // 关闭 ROS 系统
  //       exit(0);  // 退出程序
  //   }
  // }/// debug

  /// compare for gt

}

/*
// Estimate3
void UpdatePrior4()
{
  // twist2_margin_vec_
  LOG(ERROR) << "Marginize: TEST [" << prior_ctrl_id.first << ", " << prior_ctrl_id.second 
            << "], Knots = " << twist_trajctory->numKnots()<< std::endl;

  // return;

  if(prior_ctrl_id.first < 0 || prior_ctrl_id.second < prior_ctrl_id.first + 3)
  {
    LOG(ERROR) << "Initialze " << std::endl;
    return;
  }
  TrajectoryEstimatorOptions option;
  option.is_marg_state = true;
  option.marg_bias_param = true;  
  // option.marg_gravity_param = true;
  if (opt_time_offset_ && start_opt_time_offset_) {
    option.marg_t_offset_param = false;
  } else {
    option.marg_t_offset_param = true;
  }
  LOG(ERROR) << "Marginize: SET Option" << std::endl;

  LOG(ERROR) << "twist2_margin_vec_.size = " << twist2_margin_vec_.size() << std::endl;
  if(twist2_margin_vec_.empty())
    return;

  option.ctrl_to_be_opt_now = prior_ctrl_id.first;
  option.ctrl_to_be_opt_later = prior_ctrl_id.second;

  LOG(ERROR) << "spline knots = " << twist_trajctory->getKnotSize() << std::endl;
  LOG(ERROR) << "minTime = " << twist_trajctory->minTime() << std::endl;
  LOG(ERROR) << "maxTime = " << twist_trajctory->maxTime() << std::endl;
  for (auto it = twist2_margin_vec_.begin(); it != twist2_margin_vec_.end(); ) {
    int data_idx = twist_trajctory->GetCtrlIndex(it->timestamp * S_TO_NS);
    LOG(ERROR) << "data_idx = " << data_idx << std::endl; // 测试滑窗内的控制点标签
    LOG(ERROR) << "data timestamp = " << it->timestamp << std::endl; // 测试滑窗内的控制点标签
    if (data_idx < prior_ctrl_id.first) {
        it = twist2_margin_vec_.erase(it); // 正确方式：erase 返回下一个合法迭代器
    } else {
        ++it;
    }
  }


  std::map<int, double*> para_bw_vec;
  std::map<int, double*> para_bv_vec;
  {
    // 分配内存并深拷贝 omega_bias 数据
    para_bw_vec[0] = new double[3];
    std::memcpy(para_bw_vec[0], all_twist_bias_.at(last_time).omega_bias.data(), 3 * sizeof(double));

    para_bw_vec[1] = new double[3];
    std::memcpy(para_bw_vec[1], all_twist_bias_.at(cur_time).omega_bias.data(), 3 * sizeof(double));

    // 分配内存并深拷贝 vel_bias 数据
    para_bv_vec[0] = new double[3];
    std::memcpy(para_bv_vec[0], all_twist_bias_.at(last_time).vel_bias.data(), 3 * sizeof(double));

    para_bv_vec[1] = new double[3];
    std::memcpy(para_bv_vec[1], all_twist_bias_.at(cur_time).vel_bias.data(), 3 * sizeof(double));
  }

  // 构建问题
  TrajectoryEstimator2::Ptr estimator(
      new TrajectoryEstimator2(twist_trajctory, option, "Update Prior4"));

  if (marg_info) {
    std::vector<double*> drop_param_set;
    for (int i = prior_ctrl_id.first; i < prior_ctrl_id.second;   // 只边缘化范围内的点,之后的点只是产生联系
         ++i) {
      drop_param_set.emplace_back(twist_trajctory->GetAngularSpline().getKnot(i).data());
      // drop_param_set.emplace_back(trajectory_->getKnotPos(i).data());

      LOG(ERROR) << "marg spline in " << i << std::endl;
    }
    // // last bias
    // drop_param_set.emplace_back(para_bv_vec[0]);  // in the prior bias
    // drop_param_set.emplace_back(para_bw_vec[0]);

    std::vector<int> drop_set;
    for (int j = 0; j < (int)marg_parameter_blocks.size(); j++) {
      for (auto const& drop_param : drop_param_set) {
        if (marg_parameter_blocks[j] == drop_param) {
          // LOG(ERROR) << "drop set " << std::endl;
          drop_set.emplace_back(j);
          break;
        }
      }
    }
    LOG(ERROR) << "drop_set.size = " << drop_set.size() << std::endl;

    if (!drop_set.empty()) {
      MarginalizationFactor* marginalization_factor =
          new MarginalizationFactor(marg_info);

      estimator->PrepareMarginalizationInfo(RType_Prior, marginalization_factor,
                                            NULL, marg_parameter_blocks,
                                            drop_set);
      LOG(ERROR) << "Marginasize: Prepare Marginalization Info" << std::endl;
      LOG(ERROR) << "Marginasize: Add Marginalization Size " << drop_set.size() << std::endl;
    }
  }
  else
    LOG(ERROR) << "Marginasize: No Marginalization Info" << std::endl;

  // 构建量测的边缘化因子
  /// [step1] add radar doppler features 这里和优化部分是基本一致的,除了加入先验信息
  // 评估本次优化的误差,加入下一次先验
  bool marg_this_factor = true;
  // [1] 多普勒残差
  Eigen::Matrix3d R_r_e = twist_trajctory->GetSensorEP(EventSensor).q.toRotationMatrix();
  for(auto it_temp_ = twist2_margin_vec_.begin(); it_temp_ != twist2_margin_vec_.end(); it_temp_++)
  {
    // [step5-1] 多普勒残差
    const double twist_timestamp = it_temp_->timestamp;
    // LOG(ERROR) << "twist time = " << twist_timestamp << ", maxTime = " << trajectory_->maxTime(RadarSensor) << " dt = " << trajectory_->getDt() << std::endl;
    // LOG(ERROR) << "twist in " << (int)((twist_timestamp - last_max_time) / trajectory_->getDt());
    // if(last_max_time > twist_timestamp)
    //   continue;

    LOG(ERROR) << "twist time = " << twist_timestamp << ", maxTime = " << twist_trajctory->maxTime(RadarSensor) << " dt = " << trajectory_->getDt() << std::endl;
    // LOG(ERROR) << "twist in " << (int)((twist_timestamp - last_max_time) / twist_trajctory->getDt());

    Eigen::Quaterniond q_e_r = twist_trajctory->GetSensorEP(EventSensor).q;
    Eigen::Vector3d t_e_r = twist_trajctory->GetSensorEP(EventSensor).p;
    // double omega_weight = 1.0 / (it_temp_->angular_cov.norm() + 1e-2);
    // LOG(ERROR) << " before angular_bias: " << std::endl;

    // LOG(ERROR) << "it_temp_->linear_vel_vec_ = " << it_temp_->linear_vel_vec_.transpose() << std::endl;
    // estimator->AddBodyLocalVelocityMeasurementAnalytic(twist_timestamp, para_bv_vec[1], 
    //             it_temp_->linear_vel_vec_, linear_weight, linear_w_weight, R_weight, false);

    /*{
      // double linear_weight = 1.0 / it_temp_->linear_cov.norm();
      // double* vel_bias_ = para_bv_vec[1];

      sensor_msgs::PointCloud2ConstIterator<float> iter_x(it_temp_->point_cloud, "x");
      sensor_msgs::PointCloud2ConstIterator<float> iter_y(it_temp_->point_cloud, "y");
      sensor_msgs::PointCloud2ConstIterator<float> iter_z(it_temp_->point_cloud, "z");
      sensor_msgs::PointCloud2ConstIterator<float> iter_doppler(it_temp_->point_cloud, "doppler");
      
      // LOG(ERROR) << "set pre radar pcl size: " << it_temp_->point_cloud.width << std::endl;

      // std::vector<Eigen::Vector3d> pt_vec;
      // std::vector<double> pt_doppler_vec;
      for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z, ++iter_doppler) {
          //
          Eigen::Vector3d pt;
          pt << *iter_x, *iter_y, *iter_z;
          // 提取当前点的 doppler 值
          float pt_doppler = *iter_doppler;

          // pt_vec.push_back(pt);
          // pt_doppler_vec.push_back(pt_doppler);

        // estimator->AddDopplerMeasurementAnalytic(twist_timestamp, pt, vel_bias_,
        //   pt_doppler, R_r_e, linear_weight, false);    // 边缘化在先验处进行
        estimator->AddDopplerMeasurementAnalytic2(twist_timestamp, pt, para_bv_vec[1],
          pt_doppler, R_r_e, linear_weight, linear_w_weight, marg_this_factor);    // 边缘化在先验处进行

      }
    } // * /

    // 降采样
    {
      int total_points = it_temp_->point_cloud.width;
      int sample_size = 25;
      sample_size = std::min(sample_size, total_points); // 防止点数不足25

      // 构建索引数组
      std::vector<int> indices(total_points);
      std::iota(indices.begin(), indices.end(), 0);

      // 随机打乱索引
      std::random_device rd;
      std::mt19937 g(rd());
      std::shuffle(indices.begin(), indices.end(), g);

      // 随机采样的索引前25个
      std::vector<int> sample_indices(indices.begin(), indices.begin() + sample_size);

      // 准备访问迭代器
      sensor_msgs::PointCloud2ConstIterator<float> iter_x(it_temp_->point_cloud, "x");
      sensor_msgs::PointCloud2ConstIterator<float> iter_y(it_temp_->point_cloud, "y");
      sensor_msgs::PointCloud2ConstIterator<float> iter_z(it_temp_->point_cloud, "z");
      sensor_msgs::PointCloud2ConstIterator<float> iter_doppler(it_temp_->point_cloud, "doppler");

      // 为方便随机访问，把点先读到vector里（可优化为直接随机访问，但ros pointcloud2迭代器不支持直接索引）
      std::vector<Eigen::Vector3d> pts(total_points);
      std::vector<float> dopplers(total_points);
      int idx = 0;
      for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z, ++iter_doppler, ++idx) {
          pts[idx] << *iter_x, *iter_y, *iter_z;
          dopplers[idx] = *iter_doppler;
      }

      // 只对随机采样的25个点进行计算
      for (int i : sample_indices) {
          Eigen::Vector3d pt = pts[i];
          float pt_doppler = dopplers[i];
          // estimator->AddDopplerMeasurementAnalytic2(twist_timestamp, pt, para_bv_vec[1],
          //     pt_doppler, R_r_e, linear_weight, linear_w_weight, true);
          estimator->AddDopplerMeasurementAnalytic2(twist_timestamp, pt, para_bv_vec[1],
            pt_doppler, R_r_e, use_order_opti, linear_weight, linear_w_weight, true);
      }
    }

    // double* angular_bias_ = para_bw_vec[1];
    // event_flow_factors_num += it_temp_->best_inliers.size();
    LOG(ERROR) << " Add Flow Points: " << it_temp_->best_inliers.size() << std::endl;
    for(int i = 0; i < it_temp_->best_inliers.size(); i++)
    {
      Eigen::Vector3d pixel(it_temp_->best_inliers[i].x, it_temp_->best_inliers[i].y, 1.0);
      Eigen::Vector3d pixel_cord = K.inverse() * pixel;         
      // estimator->AddEventFlowMeasurementAnalytic2(twist_timestamp, pixel_cord, it_temp_->best_flow[i], it_temp_->linear_vel_vec_,
      //     q_e_r, t_e_r, para_bv_vec[1], para_bw_vec[1], time_offset, omega_weight, omega_w_weight, false);
      // estimator->AddEventFlowMeasurementAnalytic3(twist_timestamp, pixel_cord, it_temp_->best_flow[i], it_temp_->linear_vel_vec_,
      //     q_e_r, t_e_r, para_bv_vec[1], para_bw_vec[1], time_offset, global_fej_state_, use_fej, omega_weight, 
      //     omega_w_weight, marg_this_factor);

      estimator->AddEventFlowMeasurementAnalytic3(twist_timestamp, pixel_cord, it_temp_->best_flow[i], it_temp_->linear_vel_vec_,
        q_e_r, t_e_r, para_bv_vec[1], para_bw_vec[1], time_offset, global_fej_state_, use_fej, 
        use_order_opti, omega_weight, omega_w_weight, marg_this_factor);
    }
    LOG(ERROR) << "Marginasize: Add Event Flow " << std::endl;
  }

  // LOG(ERROR) << "Marginasize: Add Event Flow Measurement" << std::endl;

  estimator->SaveMarginalizationInfo(marg_info,
                                     marg_parameter_blocks);
  if(marg_info != nullptr)
    LOG(ERROR) << "[After Prior]  marg/left: " << marg_info->m << "/"
            << marg_info->n;
  else
    LOG(ERROR) << "Failed Margin info" << std::endl;

  LOG(ERROR) << "Save Marginalization Info " << std::endl;
}*/

// TightEstimate
void UpdatePrior4()
{
  // twist2_margin_vec_
  LOG(ERROR) << "Marginize: TEST [" << prior_ctrl_id.first << ", " << prior_ctrl_id.second 
            << "], Knots = " << twist_trajctory->numKnots()<< std::endl;

  // return;

  if(prior_ctrl_id.first < 0 || prior_ctrl_id.second < prior_ctrl_id.first + 3)
  {
    LOG(ERROR) << "Initialze " << std::endl;
    return;
  }
  TrajectoryEstimatorOptions option;
  option.is_marg_state = true;
  option.marg_bias_param = true;  
  // option.marg_gravity_param = true;
  if (opt_time_offset_ && start_opt_time_offset_) {
    option.marg_t_offset_param = false;
  } else {
    option.marg_t_offset_param = true;
  }
  option.ctrl_to_be_opt_now = prior_ctrl_id.first;
  option.ctrl_to_be_opt_later = prior_ctrl_id.second;

  LOG(ERROR) << "Marginize: SET Option" << std::endl;

  LOG(ERROR) << "twist2_margin_vec_.size = " << twist2_margin_vec_.size() << std::endl;
  if(twist2_margin_vec_.empty())
    return;

  LOG(ERROR) << "spline knots = " << twist_trajctory->getKnotSize() << std::endl;
  LOG(ERROR) << "minTime = " << twist_trajctory->minTime() << std::endl;
  LOG(ERROR) << "maxTime = " << twist_trajctory->maxTime() << std::endl;
  for (auto it = twist2_margin_vec_.begin(); it != twist2_margin_vec_.end(); ) {
    int data_idx = twist_trajctory->GetCtrlIndex(it->timestamp * S_TO_NS);
    LOG(ERROR) << "data_idx = " << data_idx << std::endl; // 测试滑窗内的控制点标签
    LOG(ERROR) << "data timestamp = " << it->timestamp << std::endl; // 测试滑窗内的控制点标签
    if (data_idx < prior_ctrl_id.first) {
        it = twist2_margin_vec_.erase(it); // 正确方式：erase 返回下一个合法迭代器
    } else {
        ++it;
    }
  }


  std::map<int, double*> para_bw_vec;
  std::map<int, double*> para_bv_vec;
  {
    // 分配内存并深拷贝 omega_bias 数据
    para_bw_vec[0] = new double[3];
    std::memcpy(para_bw_vec[0], all_twist_bias_.at(last_time).omega_bias.data(), 3 * sizeof(double));

    para_bw_vec[1] = new double[3];
    std::memcpy(para_bw_vec[1], all_twist_bias_.at(cur_time).omega_bias.data(), 3 * sizeof(double));

    // 分配内存并深拷贝 vel_bias 数据
    para_bv_vec[0] = new double[3];
    std::memcpy(para_bv_vec[0], all_twist_bias_.at(last_time).vel_bias.data(), 3 * sizeof(double));

    para_bv_vec[1] = new double[3];
    std::memcpy(para_bv_vec[1], all_twist_bias_.at(cur_time).vel_bias.data(), 3 * sizeof(double));
  }

  // 构建问题
  TrajectoryEstimator2::Ptr estimator(
      new TrajectoryEstimator2(twist_trajctory, option, "Update Prior4"));

  if (marg_info) {
    std::vector<double*> drop_param_set;
    for (int i = prior_ctrl_id.first; i < prior_ctrl_id.second;   // 只边缘化范围内的点,之后的点只是产生联系
         ++i) {
      drop_param_set.emplace_back(twist_trajctory->GetAngularSpline().getKnot(i).data());
      drop_param_set.emplace_back(twist_trajctory->GetLinearSpline().getKnot(i).data());
      // drop_param_set.emplace_back(trajectory_->getKnotPos(i).data());

      LOG(ERROR) << "marg spline in " << i << std::endl;
    }
    // // last bias
    // drop_param_set.emplace_back(para_bv_vec[0]);  // in the prior bias
    // drop_param_set.emplace_back(para_bw_vec[0]);

    // marg_parameter_blocks 里面没有 prior_ctrl_id 中的点 (样条移动)
    std::vector<int> drop_set;
    for (int j = 0; j < (int)marg_parameter_blocks.size(); j++) {
      for (auto const& drop_param : drop_param_set) {
        if (marg_parameter_blocks[j] == drop_param) {
          // LOG(ERROR) << "drop set " << std::endl;
          drop_set.emplace_back(j);
          break;
        }
      }
    }
    LOG(ERROR) << "drop_set.size = " << drop_set.size() << std::endl;

    if (!drop_set.empty()) {
      MarginalizationFactor* marginalization_factor =
          new MarginalizationFactor(marg_info);

      estimator->PrepareMarginalizationInfo(RType_Prior, marginalization_factor,
                                            NULL, marg_parameter_blocks,
                                            drop_set);
      LOG(ERROR) << "Marginasize: Prepare Marginalization Info" << std::endl;
      LOG(ERROR) << "Marginasize: Add Marginalization Size " << drop_set.size() << std::endl;
    }
  }
  else
    LOG(ERROR) << "Marginasize: No Marginalization Info" << std::endl;

  // 构建量测的边缘化因子
  /// [step1] add radar doppler features 这里和优化部分是基本一致的,除了加入先验信息
  // 评估本次优化的误差,加入下一次先验
  bool marg_this_factor = true;
  // [1] 多普勒残差
  Eigen::Matrix3d R_r_e = twist_trajctory->GetSensorEP(EventSensor).q.toRotationMatrix();
  LOG(ERROR) << "Marginize Size: " << twist2_margin_vec_.end() - twist2_margin_vec_.begin() + 1 << std::endl;
  // LOG(ERROR) << "it_base2 is in " << (it_base2 - twist2_margin_vec_.begin() + 1) << std::endl;
  for(auto it_temp_ = twist2_margin_vec_.begin(); it_temp_ != twist2_margin_vec_.end(); it_temp_++)
  {
    // [step5-1] 多普勒残差
    const double twist_timestamp = it_temp_->timestamp;

    LOG(ERROR) << "twist time = " << twist_timestamp << ", maxTime = " << twist_trajctory->maxTime(RadarSensor) << " dt = " << trajectory_->getDt() << std::endl;
    // LOG(ERROR) << "twist in " << (int)((twist_timestamp - last_max_time) / twist_trajctory->getDt());

    Eigen::Quaterniond q_e_r = twist_trajctory->GetSensorEP(EventSensor).q;
    Eigen::Vector3d t_e_r = twist_trajctory->GetSensorEP(EventSensor).p;

    
    // 降采样
    {
      int total_points = it_temp_->point_cloud.width;
      int sample_size = 25;
      sample_size = std::min(sample_size, total_points); // 防止点数不足25
      LOG(ERROR) << "total_points = " << total_points << std::endl;
      LOG(ERROR) << "sample_size = " << sample_size << std::endl;

      // 构建索引数组
      std::vector<int> indices(total_points);
      std::iota(indices.begin(), indices.end(), 0);

      LOG(ERROR) << "A" << std::endl;

      // 随机打乱索引
      std::random_device rd;
      std::mt19937 g(rd());
      std::shuffle(indices.begin(), indices.end(), g);

      // 随机采样的索引前25个
      std::vector<int> sample_indices(indices.begin(), indices.begin() + sample_size);
      LOG(ERROR) << "sample_indices.size = " << sample_indices.size() << std::endl;

      // if(it_temp_ == twis)
      //   LOG(ERROR) << "it_temp_ is not valid" << std::endl;

      // for (const auto& field : it_temp_->point_cloud.fields) {
      //     // ROS_INFO("Field: %s, datatype: %d", field.name.c_str(), field.datatype);
      //     LOG(ERROR) << "Field: " << field.name.c_str() << ", datatype: " << field.datatype << std::endl;
      // }

      // 准备访问迭代器
      sensor_msgs::PointCloud2ConstIterator<float> iter_x(it_temp_->point_cloud, "x");
      sensor_msgs::PointCloud2ConstIterator<float> iter_y(it_temp_->point_cloud, "y");
      sensor_msgs::PointCloud2ConstIterator<float> iter_z(it_temp_->point_cloud, "z");
      sensor_msgs::PointCloud2ConstIterator<float> iter_doppler(it_temp_->point_cloud, "doppler");
      LOG(ERROR) << "B" << std::endl;
      // 为方便随机访问，把点先读到vector里（可优化为直接随机访问，但ros pointcloud2迭代器不支持直接索引）
      std::vector<Eigen::Vector3d> pts(total_points);
      std::vector<float> dopplers(total_points);
      int idx = 0;
      for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z, ++iter_doppler, ++idx) {
          pts[idx] << *iter_x, *iter_y, *iter_z;
          dopplers[idx] = *iter_doppler;
          LOG(ERROR) << "idx = " << idx << ", total_points = " << total_points << std::endl;
      }
      LOG(ERROR) << "Get enough points" << std::endl;

      // 只对随机采样的25个点进行计算
      for (int i : sample_indices) {
          Eigen::Vector3d pt = pts[i];
          float pt_doppler = dopplers[i];
          estimator->AddDopplerMeasurementAnalytic2(twist_timestamp, pt, para_bv_vec[1],
              pt_doppler, R_r_e, use_order_opti, linear_weight, linear_w_weight, true);
      }
    }
    LOG(ERROR) << "Marginasize: Add Doppler Markers " << std::endl;

    // double* angular_bias_ = para_bw_vec[1];
    // event_flow_factors_num += it_temp_->best_inliers.size();
    LOG(ERROR) << " Add Flow Points: " << it_temp_->best_inliers.size() << std::endl;
    for(int i = 0; i < it_temp_->best_inliers.size(); i++)
    {
      Eigen::Vector3d pixel(it_temp_->best_inliers[i].x, it_temp_->best_inliers[i].y, 1.0);
      Eigen::Vector3d pixel_cord = K.inverse() * pixel;         
      // estimator->AddEventFlowMeasurementAnalytic2(twist_timestamp, pixel_cord, it_temp_->best_flow[i], it_temp_->linear_vel_vec_,
      //     q_e_r, t_e_r, para_bv_vec[1], para_bw_vec[1], time_offset, omega_weight, omega_w_weight, false);
      estimator->AddEventFlowMeasurementAnalytic3(twist_timestamp, pixel_cord, it_temp_->best_flow[i], it_temp_->linear_vel_vec_,
          q_e_r, t_e_r, para_bv_vec[1], para_bw_vec[1], time_offset, global_fej_state_, use_fej, 
          use_order_opti, omega_weight, omega_w_weight, true);
    }
    LOG(ERROR) << "Marginasize: Add Event Flow " << std::endl;
  }

  // LOG(ERROR) << "Marginasize: Add Event Flow Measurement" << std::endl;
  LOG(ERROR) << " Try to Save Marginalization Info " << std::endl;
  estimator->SaveMarginalizationInfo(marg_info,
                                     marg_parameter_blocks);
  LOG(ERROR) << " Save Marginalization Info Done " << std::endl;

  if(marg_info != nullptr)
    LOG(ERROR) << "[After Prior]  marg/left: " << marg_info->m << "/"
            << marg_info->n;
  else
    LOG(ERROR) << "Failed Margin info" << std::endl;

  LOG(ERROR) << "Save Marginalization Info " << std::endl;
}



// 使用速度空间进行优化， 状态为 v w, 其他参数 ba bg, T, t
// void Estimate2(TwistData2& twist)
// UpdatePrior4
Twist_Trajectory::Ptr twist_trajctory;
void Estimate3()
{
  LOG(ERROR) << "  ----------------- Estimate3 -------------------- " << std::endl;
  std::chrono::time_point<std::chrono::high_resolution_clock> start_time = std::chrono::high_resolution_clock::now();

  if(!initial_done)
    return ;

  if(twist2_vec_.empty())
    return;
  LOG(ERROR) << "twist2_vec_.size = " << twist2_vec_.size() << std::endl;

  // [step4] 根据样条段，分割当前区间
 
  // double last_max_time = (twist_spline.maxTimeNs() - EP_RtoI.t_offset_ns) * NS_TO_S;
  double last_max_time = twist_trajctory->maxTime(RadarSensor);
  double traj_max_time = last_max_time + t_add; // 单位: 秒
  LOG(ERROR) << "last traj_max_time = " << last_max_time << std::endl;
  LOG(ERROR) << "traj_max_time = " << traj_max_time << std::endl;
  LOG(ERROR) << "twist2_vec_.back().timestamp = " << twist2_vec_.back().timestamp << std::endl;
  if(twist2_vec_.back().timestamp < traj_max_time)
    return;

  auto it = std::find_if(twist2_vec_.rbegin(), twist2_vec_.rend(), 
                      [traj_max_time](const TwistData2& data) {
                          return data.timestamp < traj_max_time;
                      });

  for(auto& twis : twist2_vec_)
  {
    LOG(ERROR) << "loop timestamp = " << twis.timestamp << std::endl;
  }

  // SE3d last_knot = trajectory_->getLastKnot();
  // Eigen::Matrix<double,6, 1> last_knot = twist_trajctory->getLastKnot();
  LOG(ERROR) << "Get Last Knot" << std::endl;
  // Eigen::Vector3d last_linear_knot, last_angular_knot;
  // EIGEN_ALIGN16 Eigen::Vector3d last_linear_knot;
  // EIGEN_ALIGN16 Eigen::Vector3d last_angular_knot;

  Eigen::Vector3d last_linear_knot;
  Eigen::Vector3d last_angular_knot;

  if(twist_trajctory->getKnotSize() < 1)  // 为空
  {
    last_linear_knot.setZero();
    last_angular_knot.setZero();
  }
  // else
  // {
  //   Eigen::VectorXd last_knots(6);
  //   // last_knots.resize(6);
  //   last_knots = twist_trajctory->getLastKnot();
  //   last_linear_knot = last_knots.block<3,1>(0,0);
  //   last_angular_knot = last_knots.block<3,1>(3,0);
  // }
  else 
  {
    auto [lin_knot, ang_knot] = twist_trajctory->getLastKnots();
    last_linear_knot = lin_knot;
    last_angular_knot = ang_knot;
  }
  LOG(ERROR) << "Get Last Knot" << std::endl;

  // discard
  /*{
    // Eigen::Matrix<double, 6, 1>last_knot = twist_trajctory->getLastKnot();
    // Eigen::Matrix<double, 6, 1>last_knot;
    // last_knot.setZero();        // 速度空间清零
    // Eigen::Vector3d last_linear_knot = twist_spline.getLinearKnots().back();
    // Eigen::Vector3d last_angular_knot = twist_spline.getAngularKnots().back();
  }*/
  
  size_t knot_num = twist_trajctory->getKnotSize();
  LOG(ERROR) << "knot_num = " << knot_num << std::endl;
  if(it == twist2_vec_.rend())  // 没有合理的数据,直接返回,这种一般是存在缺失值
  {
    LOG(ERROR) << " Not Suitable Data " << std::endl;

    // twist_spline.extendKnotsTo(traj_max_time * S_TO_NS, last_linear_knot, last_angular_knot);

    // trajectory_->extendKnotsTo(traj_max_time * S_TO_NS, last_knot); // 扩展到最新的数据
    twist_trajctory->extendKnotsTo(traj_max_time * S_TO_NS, last_linear_knot, last_angular_knot); // 扩展到最新的数据

    // twist_spline.SetActiveTime(traj_max_time);
    // trajectory_->SetActiveTime(traj_max_time);
    twist_trajctory->SetActiveTime(traj_max_time);
    LOG(ERROR) << " Update Active Time " << traj_max_time << std::endl;

    // twist_spline.SetForcedFixedTime(traj_max_time);
    // trajectory_->SetForcedFixedTime(traj_max_time);         // 毫米波雷达无畸变,只有一个更新时间
    twist_trajctory->SetForcedFixedTime(traj_max_time);
    LOG(ERROR) << " Set Forced Fixed Time " << traj_max_time << std::endl;

    LOG(ERROR) << "Piror_id.first = " << prior_ctrl_id.first << ", " << prior_ctrl_id.second << std::endl;
    
    /*{
      LOG(ERROR) << "twist2_vec_.size()" << twist2_vec_.size() << std::endl;
      LOG(ERROR) << "it_base2 - twist2_vec_.begin()" << it_base2 - twist2_vec_.begin() << std::endl;
      if(it_base2 == twist2_vec_.begin())
          twist2_vec_.erase(twist2_vec_.begin());
      else
          if(it_base2 + 1 == twist2_vec_.end())
            twist2_vec_.clear();
          else
          twist2_vec_.erase(twist2_vec_.begin(), it_base2 + 1);

      // if (it_base2 != twist2_vec_.end()) {
      //     twist2_vec_.erase(twist2_vec_.begin(), std::next(it_base2));

    }*/

    // 清空边缘化信息
    ClearPrior();
  //   marg_info = nullptr;
  //  for (double* parameters : marg_parameter_blocks) {
  //       delete[] parameters;
  //   }
  //   marg_parameter_blocks.clear();
    LOG(ERROR) << "Missing Data" << std::endl;
    return;
  }

  it_base2 = ((it == twist2_vec_.rend())? twist2_vec_.begin() : it.base() - 1); // it 的正向跌代器
  LOG(ERROR) << "it->timestamp = " << it->timestamp << std::endl;
  LOG(ERROR) << "it_base2->timestamp = " << it_base2->timestamp << std::endl;
  assert(it->timestamp == it_base2->timestamp && "Not Same Place");
  // 确定终点
  cur_time = it_base2->timestamp;
  all_twist_bias_[cur_time] = all_twist_bias_[last_time];
  double update_time = cur_time;  // 注意是相对时间的 秒
  
  // LOG(ERROR) << "CHECK max time 1 = " << trajectory_->maxTime(RadarSensor) << std::endl;
  LOG(ERROR) << "it->timestamp * S_TO_NS = " << it->timestamp * S_TO_NS << std::endl;
  // trajectory_->extendKnotsTo(it->timestamp * S_TO_NS, last_knot); // 扩展到最新的数据

  LOG(ERROR) << "check knot size = " << twist_trajctory->getKnotSize() << std::endl;
  twist_trajctory->extendKnotsTo(traj_max_time * S_TO_NS, last_linear_knot, last_angular_knot); // 扩展到最新的数据
  // twist_spline.extendKnotsTo(it->timestamp * S_TO_NS, last_linear_knot, last_angular_knot);
  // LOG(ERROR) << "CHECK max time 2 = " << trajectory_->maxTime(RadarSensor) << std::endl;

  InitPrior();

  // Debug: 某端时间是合理的
  /*{
    LOG(ERROR) << " trajectory_ extend knots timestamp: " << std::setprecision(9) << it->timestamp << std::endl;
    LOG(ERROR) << " check update timestamp: " << std::setprecision(9) << it->timestamp - traj_max_time << std::endl;  // should be <
    // 如果下一个存在,检查一下下一个的时间戳
    if(it_base2 != twist2_vec_.end() && (it_base2 + 1) != twist2_vec_.end())
    {
      LOG(ERROR) << " trajectory_ extend knots timestamp: " << std::setprecision(9) << (it + 1)->timestamp << std::endl;
      LOG(ERROR) << " check update timestamp: " << std::setprecision(9) << (it - 1)->timestamp - traj_max_time << std::endl;
    }
  }*/

  std::map<int, double*> para_bw_vec;
  std::map<int, double*> para_bv_vec;
  {
    // 分配内存并深拷贝 omega_bias 数据
    para_bw_vec[0] = new double[3];
    std::memcpy(para_bw_vec[0], all_twist_bias_.at(last_time).omega_bias.data(), 3 * sizeof(double));

    para_bw_vec[1] = new double[3];
    std::memcpy(para_bw_vec[1], all_twist_bias_.at(cur_time).omega_bias.data(), 3 * sizeof(double));

    // 分配内存并深拷贝 vel_bias 数据
    para_bv_vec[0] = new double[3];
    std::memcpy(para_bv_vec[0], all_twist_bias_.at(last_time).vel_bias.data(), 3 * sizeof(double));

    para_bv_vec[1] = new double[3];
    std::memcpy(para_bv_vec[1], all_twist_bias_.at(cur_time).vel_bias.data(), 3 * sizeof(double));
  }
  std::chrono::time_point<std::chrono::high_resolution_clock> time1 = std::chrono::high_resolution_clock::now();


  // [step5] 创建优化器
  TrajectoryEstimatorOptions option;
  option.lock_EPs.at(EventSensor).Unlock();
  option.is_marg_state = false;
  option.show_residual_summary = true;
  // TrajectoryEstimator::Ptr estimator(new TrajectoryEstimator(trajectory_, option, "Update Traj"));
  TrajectoryEstimator2::Ptr estimator(new TrajectoryEstimator2(twist_trajctory, option, "Update Traj"));
  
  
  LOG(ERROR) << " start estimate " << std::endl;

  // decide prior index
  double opt_min_time = twist2_vec_.begin()->timestamp;
  LOG(ERROR) << " opt_min_time = " << opt_min_time << std::endl;
  int opt_idx = twist_trajctory->GetCtrlIndex(opt_min_time * S_TO_NS);
  // option.ctrl_to_be_opt_now = std::min(prior_ctrl_id.first, opt_idx);
  LOG(ERROR) << "opt_idx = " << opt_idx << std::endl;
  option.ctrl_to_be_opt_now = opt_idx;
  double opt_max_time = it_base2->timestamp;
  int scan_idx = twist_trajctory->GetCtrlIndex(opt_max_time * S_TO_NS);
  option.ctrl_to_be_opt_later = std::max(scan_idx, twist_trajctory->Get_N());
  LOG(ERROR) << "check ctrl later idx = std::max [" << scan_idx 
             << ", " << twist_trajctory->Get_N() << "]" << std::endl;

  size_t knots_num = twist_trajctory->getKnotSize();
  LOG(ERROR) << "spline knots_num = " << knots_num << std::endl;

  // LOG(ERROR) << "Radar max time = " << trajectory_->maxTime(RadarSensor)  << std::endl;
  // LOG(ERROR) << "activate time = " << trajectory_->GetActiveTime()  << std::endl;
  // LOG(ERROR) << "forcefixed time = " << trajectory_->GetForcedFixedTime()  << std::endl;
  // LOG(ERROR) << "optimization time between [" << opt_min_time << ", " << opt_max_time << "]" << std::endl;
  // LOG(ERROR) << "optimization ctrl idx between [" << option.ctrl_to_be_opt_now 
  //            << ", " << option.ctrl_to_be_opt_later << "]" << std::endl;

  LOG(ERROR) << "last max time = " << last_max_time << std::endl;
  LOG(ERROR) << "activate time = " << twist_trajctory->GetActiveTime()  << std::endl;
  LOG(ERROR) << "forcefixed time = " << twist_trajctory->GetForcedFixedTime()  << std::endl;
  LOG(ERROR) << "optimization time between [" << opt_min_time << ", " << opt_max_time << "]" << std::endl;
  LOG(ERROR) << "optimization ctrl idx between [" << option.ctrl_to_be_opt_now 
             << ", " << option.ctrl_to_be_opt_later << "]" << std::endl;

  // HAO TODO: 这部分暂时不需要
  // TODO: 确定 prior_ctrl_id.first 的数值
  /*if (LocatedInFirstSegment2(opt_min_time)) {
    // estimator->SetFixedIndex(trajectory_->Get_N() - 1);
    estimator->SetFixedIndex(twist_trajctory->Get_N() - 1);
    LOG(ERROR) << "estimator->SetFixedIndex = " << twist_trajctory->Get_N() - 1 << std::endl;
    // LOG(ERROR) << "estimator->SetFixedIndex = " << trajectory_->Get_N() - 1 << std::endl;
    // estimator->AddStartTimePose(original_pose_);
  } else {
    estimator->SetFixedIndex(prior_ctrl_id.first - 1);
    LOG(ERROR) << "estimator->SetFixedIndex = " << prior_ctrl_id.first - 1 << std::endl;
    LOG(ERROR) << "trajectory_.fixed_control_point_index_ = " << estimator->GetFixedControlIndex() << std::endl;

    // trajectory_->opt_min_init_time_tmp = opt_min_time;
    // trajectory_->opt_min_init_time_tmp = last_time;
    // trajectory_->opt_init_fixed_idx_tmp = estimator->GetFixedControlIndex();
  }*/

  estimator->SetFixedIndex(std::max(opt_idx - 1, 0));
  // estimator->SetFixedIndex(prior_ctrl_id.first - 1);
  // LOG(ERROR) << "estimator->SetFixedIndex = " << prior_ctrl_id.first - 1 << std::endl;
  LOG(ERROR) << "estimator->SetFixedIndex = " << std::max(opt_idx - 1, 0) << std::endl;
  LOG(ERROR) << "trajectory_.fixed_control_point_index_ = " << estimator->GetFixedControlIndex() << std::endl;

  // [step5] 因子图优化
  /// 因子图优化
  std::chrono::time_point<std::chrono::high_resolution_clock> time3;
  std::chrono::time_point<std::chrono::high_resolution_clock> time4;
  std::chrono::time_point<std::chrono::high_resolution_clock> time5;
  long int doppler_factors_num = 0;
  long int event_flow_factors_num = 0;

    // 雷达线速度必须存在
    time5 = std::chrono::high_resolution_clock::now();

    // LOG(ERROR) << " Add all factor into the solver " << std::endl;
    // LOG(ERROR) << "Estimator: Add Doppler Factors for " << doppler_factors_num << std::endl;
    // LOG(ERROR) << "Estimator: Add EventFlow Factors for " << event_flow_factors_num << std::endl;
    // LOG(ERROR) << "Estimator: Add Bias Factors for " << 1 << std::endl;
  //}/// 因子图优化

  LOG(ERROR) << "First [Prior Ctrl ID] = [" << prior_ctrl_id.first << ","
              << prior_ctrl_id.second << "] "
              << prior_ctrl_id.second - prior_ctrl_id.first + 1 ;

  // 松耦合
  {
    // [step5-1] 优化问题建模
    {
      time3 = std::chrono::high_resolution_clock::now();
      // assert(marg_info == nullptr);
      LOG(ERROR) << "Estimation marg_info = " << ((marg_info)? "True" : "False") << std::endl;
      // marg_info 后面会一直存在
      // [step5-0] 先验因子

        // LOG(ERROR) << "[Prior Ctrl ID] = [" << prior_ctrl_id.first << ","
        //     << prior_ctrl_id.second << "] " << std::endl;
      if (marg_info) {
        // 确定下一次边缘化的部分
        // prior_ctrl_id.first = option.ctrl_to_be_opt_later;
        // prior_ctrl_id.second = twist_trajctory->numKnots() - 1;

        // 边缘化示例子
        // [0 1 2 3]  不边缘化
        // [ [0 1 2 3] 4 5 6 7] 边缘化 [0 1 2 3]
        // [[4 5 6 7] 8 9 10 11] 边缘化 [4 5 6 7]
        // prior_ctrl_id.first = prior_ctrl_id.second + 1;
        // prior_ctrl_id.second = twist_trajctory->numKnots() - update_every_k_knot_;

        LOG(ERROR) << "[Prior Ctrl ID] = [" << prior_ctrl_id.first << ","
            << prior_ctrl_id.second << "] "
            << prior_ctrl_id.second - prior_ctrl_id.first + 1 ;

        estimator->AddMarginalizationFactor(marg_info,
                                            marg_parameter_blocks);
        LOG(ERROR) << " Add Marginalize " << std::endl;
      }
      
      time4 = std::chrono::high_resolution_clock::now();
      // 根据量测构建因子图优化(Loop Optimization)
      Eigen::Matrix3d R_r_e = twist_trajctory->GetSensorEP(EventSensor).q.toRotationMatrix();
      LOG(ERROR) << " Twist Size = " << (it_base2 - twist2_vec_.begin() + 1) << std::endl;
      // LOG(ERROR) << " Aver Freq = " << 1.0 / (it_base2->timestamp - twist2_vec_.begin()->timestamp) / (it_base2 - twist2_vec_.begin() + 1) << " Hz" << std::endl;

      for(auto it_temp_ = twist2_vec_.begin(); it_temp_ <= it_base2; it_temp_++)
      {
        // [step5-1] 多普勒残差
        const double twist_timestamp = it_temp_->timestamp;
        // LOG(ERROR) << "twist time = " << twist_timestamp << ", maxTime = " << trajectory_->maxTime(RadarSensor) << " dt = " << trajectory_->getDt() << std::endl;
        // LOG(ERROR) << "twist in " << (int)((twist_timestamp - last_max_time) / trajectory_->getDt());
        // if(last_max_time > twist_timestamp)
        //   continue;

        LOG(ERROR) << "twist time = " << twist_timestamp << ", maxTime = " << twist_trajctory->maxTime(RadarSensor) << " dt = " << trajectory_->getDt() << std::endl;
        // LOG(ERROR) << "twist in " << (int)((twist_timestamp - last_max_time) / twist_trajctory->getDt());

        Eigen::Quaterniond q_e_r = twist_trajctory->GetSensorEP(EventSensor).q;
        Eigen::Vector3d t_e_r = twist_trajctory->GetSensorEP(EventSensor).p;
        // double omega_weight = 1.0 / (it_temp_->angular_cov.norm() + 1e-2);
        // LOG(ERROR) << " before angular_bias: " << std::endl;

        // LOG(ERROR) << "it_temp_->linear_vel_vec_ = " << it_temp_->linear_vel_vec_.transpose() << std::endl;
        // estimator->AddBodyLocalVelocityMeasurementAnalytic(twist_timestamp, para_bv_vec[1], 
        //             it_temp_->linear_vel_vec_, linear_weight, linear_w_weight, R_weight, false);

        /*{
          // double linear_weight = 1.0 / it_temp_->linear_cov.norm();
          // double* vel_bias_ = para_bv_vec[1];

          sensor_msgs::PointCloud2ConstIterator<float> iter_x(it_temp_->point_cloud, "x");
          sensor_msgs::PointCloud2ConstIterator<float> iter_y(it_temp_->point_cloud, "y");
          sensor_msgs::PointCloud2ConstIterator<float> iter_z(it_temp_->point_cloud, "z");
          sensor_msgs::PointCloud2ConstIterator<float> iter_doppler(it_temp_->point_cloud, "doppler");
          
          LOG(ERROR) << "set pre radar pcl size: " << it_temp_->point_cloud.width << std::endl;

          // std::vector<Eigen::Vector3d> pt_vec;
          // std::vector<double> pt_doppler_vec;
          for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z, ++iter_doppler) {
              //
              Eigen::Vector3d pt;
              pt << *iter_x, *iter_y, *iter_z;
              // 提取当前点的 doppler 值
              float pt_doppler = *iter_doppler;

              // pt_vec.push_back(pt);
              // pt_doppler_vec.push_back(pt_doppler);

            // estimator->AddDopplerMeasurementAnalytic(twist_timestamp, pt, vel_bias_,
            //   pt_doppler, R_r_e, linear_weight, false);    // 边缘化在先验处进行
            estimator->AddDopplerMeasurementAnalytic2(twist_timestamp, pt, para_bv_vec[1],
              pt_doppler, R_r_e, linear_weight, linear_w_weight, option.is_marg_state);    // 边缘化在先验处进行

          }
        }*/

        // 降采样
        {
          int total_points = it_temp_->point_cloud.width;
          int sample_size = 25;
          sample_size = std::min(sample_size, total_points); // 防止点数不足25

          // 构建索引数组
          std::vector<int> indices(total_points);
          std::iota(indices.begin(), indices.end(), 0);

          // 随机打乱索引
          std::random_device rd;
          std::mt19937 g(rd());
          std::shuffle(indices.begin(), indices.end(), g);

          // 随机采样的索引前25个
          std::vector<int> sample_indices(indices.begin(), indices.begin() + sample_size);

          // 准备访问迭代器
          sensor_msgs::PointCloud2ConstIterator<float> iter_x(it_temp_->point_cloud, "x");
          sensor_msgs::PointCloud2ConstIterator<float> iter_y(it_temp_->point_cloud, "y");
          sensor_msgs::PointCloud2ConstIterator<float> iter_z(it_temp_->point_cloud, "z");
          sensor_msgs::PointCloud2ConstIterator<float> iter_doppler(it_temp_->point_cloud, "doppler");

          // 为方便随机访问，把点先读到vector里（可优化为直接随机访问，但ros pointcloud2迭代器不支持直接索引）
          std::vector<Eigen::Vector3d> pts(total_points);
          std::vector<float> dopplers(total_points);
          int idx = 0;
          for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z, ++iter_doppler, ++idx) {
              pts[idx] << *iter_x, *iter_y, *iter_z;
              dopplers[idx] = *iter_doppler;
          }

          LOG(ERROR) << "sample_indices.size = " << sample_indices.size() << std::endl;

          // 只对随机采样的25个点进行计算
          for (int i : sample_indices) {
              Eigen::Vector3d pt = pts[i];
              float pt_doppler = dopplers[i];
              estimator->AddDopplerMeasurementAnalytic2(twist_timestamp, pt, para_bv_vec[1],
                  pt_doppler, R_r_e, use_order_opti, linear_weight, linear_w_weight, false);
          }
        }

        // double* angular_bias_ = para_bw_vec[1];
        event_flow_factors_num += it_temp_->best_inliers.size();
        LOG(ERROR) << " Add Flow Points: " << it_temp_->best_inliers.size() << std::endl;
        for(int i = 0; i < it_temp_->best_inliers.size(); i++)
        {
          Eigen::Vector3d pixel(it_temp_->best_inliers[i].x, it_temp_->best_inliers[i].y, 1.0);
          Eigen::Vector3d pixel_cord = K.inverse() * pixel;         
          // estimator->AddEventFlowMeasurementAnalytic2(twist_timestamp, pixel_cord, it_temp_->best_flow[i], it_temp_->linear_vel_vec_,
          //     q_e_r, t_e_r, para_bv_vec[1], para_bw_vec[1], time_offset, omega_weight, omega_w_weight, false);
          estimator->AddEventFlowMeasurementAnalytic3(twist_timestamp, pixel_cord, it_temp_->best_flow[i], it_temp_->linear_vel_vec_,
              q_e_r, t_e_r, para_bv_vec[1], para_bw_vec[1], time_offset, global_fej_state_, use_fej, 
              use_order_opti, omega_weight, omega_w_weight, false);
        }
        LOG(ERROR) << "Marginasize: Add Event Flow " << std::endl;

        // TODO: twist.best_flow 是统一的光流,可以修改为每个事件的光流
        // LOG(INFO) << " Add Event Flow " << std::endl;
        LOG(ERROR) << " Add Event Flow " << std::endl;
      }
      // std::ofstream residual_file("/home/hao/Desktop/radar-event-new/src/TwistEstimator/output/res.txt", std::ios::out | std::ios::app);
      // residual_file << "done" << std::endl;
      // residual_file.close();

      // std::ofstream jac_knot_file("/home/hao/Desktop/radar-event-new/src/TwistEstimator/output/jac.txt", std::ios::out | std::ios::app);
      // jac_knot_file << "done" << std::endl;
      // jac_knot_file.close();
    }
      
      
      // [3] Twist Bias 估计
      // Bias 只跟两次量测时间相关，而和具体量测数量无关
      {
        double delta_time = cur_time - last_time;
        delta_time = std::max(delta_time, 1e-3);

        // double dt = 1. / 200.; // opt_weight_.imu_noise.imu_frequency;
        // double dt = 1. / 10.; // opt_weight_.radar_frequency;
        double dt = 1. / 20.; // opt_weight_.radar_frequency;
        double cov = delta_time / dt * (dt * dt);
        LOG(ERROR) << "delta_time = " << delta_time << std::endl;
        LOG(ERROR) << "cov = " << cov << std::endl;
        Eigen::Matrix<double, 6, 1> sqrt_info;
        sqrt_info.setOnes();
        sqrt_info *=  (1. / std::sqrt(cov)); // * opt_weight_.bias_info_vec;

        LOG(ERROR) << "100 * sqrt_info = " << 100 * sqrt_info << std::endl;

        LOG(ERROR) << " Add Bias Factor " << std::endl;

        estimator->AddBiasFactor(para_bw_vec[0], para_bw_vec[1], 
                                  para_bv_vec[0], para_bv_vec[1], 
                                  1, 100 * sqrt_info); // 100 * sqrt_info);

        LOG(ERROR) << " Add Bias: " << std::endl;
        LOG(ERROR) << "last time omega_bias = " << all_twist_bias_.at(last_time).omega_bias << std::endl;
        LOG(ERROR) << "cur time omega_bias = " << all_twist_bias_.at(cur_time).omega_bias << std::endl; 
        LOG(ERROR) << "last time vel_bias = " << all_twist_bias_.at(last_time).vel_bias << std::endl; 
        LOG(ERROR) << "cur time vel_bias = " << all_twist_bias_.at(cur_time).vel_bias << std::endl; 

      }

      LOG(ERROR) << " Add all factor into the solver " << std::endl;
      LOG(ERROR) << "Estimator: Add Doppler Factors for " << doppler_factors_num << std::endl;
      LOG(ERROR) << "Estimator: Add EventFlow Factors for " << event_flow_factors_num << std::endl;
      LOG(ERROR) << "Estimator: Add Bias Factors for " << 1 << std::endl;
    

    // 雷达线速度必须存在
    time5 = std::chrono::high_resolution_clock::now();

    // 分组优化
    /*{
      // 设置分组优化
      estimator->SetParameterBlockOrdering();
    }*/


    // if(doppler_factors_num > 0)
    // {   
      // [step5-2] 优化问题求解
      {
        LOG(ERROR) << " start to solve problem " << std::endl;

        // ceres::Solver::Summary summary = estimator->Solve(50, false);
        ceres::Solver::Summary summary = estimator->Solve(50, false, 8);  // use multi-thread
        LOG(ERROR) << summary.BriefReport();
        LOG(ERROR) << "Traj Update Successful/Unsuccessful steps: "
                  << summary.num_successful_steps << "/"
                  << summary.num_unsuccessful_steps;
        LOG(ERROR) << "Traj Update details: ";
        LOG(ERROR) << summary.FullReport();

        LOG(ERROR) << "end to solve problem " << std::endl;
      }
    // }
  }/// 因子图优化*/




  /// 发布结果
  std::chrono::time_point<std::chrono::high_resolution_clock> time6 = std::chrono::high_resolution_clock::now();
  {
    auto bias = GetLatestBias();
    // PublishLatestBias(bias.omega_bias, bias.vel_bias);
    // auto pose = twist_trajctory->GetRadarPose(it->timestamp);
    // HAO TODO: 修改
    Eigen::Quaterniond q(Eigen::Quaterniond::Identity());
    Eigen::Vector3d t(0.0, 0.0, 0.0);
    Sophus::SE3d pose(q, t);
        
    // Eigen::Vector3d linear_velocity = pose.rotationMatrix().transpose() * trajectory_->GetTransVelWorld(it->timestamp);    // radar velocity
    Eigen::Vector3d linear_velocity = twist_trajctory->GetLiearVelWorld(it->timestamp);    // radar velocity
    Eigen::Vector3d angular_velocity = twist_trajctory->GetAngularVelWorld(it->timestamp);  

    if(global_fej_state_ == nullptr && use_fej)
    {
      // global_fej_state_ = new FEJ_STATE();
      global_fej_state_ = std::make_shared<FEJ_STATE>();
      global_fej_state_->linear_velocity_ = linear_velocity;
      global_fej_state_->angular_velocity_ = angular_velocity;
      global_fej_state_->linear_bias_ = bias.vel_bias;
      global_fej_state_->angular_bias_ = bias.omega_bias;

      LOG(ERROR) << "FEJ STATE: "
                <<  "\nvel_ = " << global_fej_state_->linear_velocity_
                <<  "\nomega_ = " << global_fej_state_->angular_velocity_
                <<  "\nlinear_bias_ = " << global_fej_state_->linear_bias_
                <<  "\nangular_bias_ = " << global_fej_state_->angular_bias_
                << std::endl;
    }

    LOG(ERROR) << "Estimate Velocity:\n"
               << "origin linear velocity = \t" << it->linear_vel_vec_(0) << ", " << it->linear_vel_vec_(1) << ", " << it->linear_vel_vec_(2) << "\n"
               << "estimate linear velocity = \t" << linear_velocity(0) << ", " << linear_velocity(1) << ", " << linear_velocity(2) << "\n"
               << "origin angular velocity = \t" << it->angular_vel_vec_(0) << ", " << it->angular_vel_vec_(1) << ", " << it->angular_vel_vec_(2) << "\n"
               << "estimate angular velocity = \t" << angular_velocity(0) << ", " << angular_velocity(1) << ", " << angular_velocity(2) << "\n";        
  

    // Eigen::Quaterniond q_extract = pose.rotationQuaternion();
    /*Eigen::Quaterniond q_extract(pose.rotationMatrix());
    Eigen::Vector3d t_extract = pose.rotationMatrix() * pose.translation();
    

    // extrincs parameter event w.r.t to radar
    ExtrinsicParam Extrin_e_r = twist_trajctory->GetSensorEP(EventSensor);
    Eigen::Matrix4d T_e_r = Extrin_e_r.Get_T();
    double timeoffset_e_r = Extrin_e_r.Get_Timeoffset();

    LOG(ERROR) << "Estimate Result:\n" 
              << "timestamp = \t" << std::setprecision(18) << it->timestamp + trajectory_->GetDataStartTime() << "\n"
              << "position = \t" << t_extract.transpose() << "\n"
              << "quaternion = \t" << q_extract.coeffs().transpose() << "\n"
              << "linear velocity = \t" << linear_velocity(0) << ", " << linear_velocity(1) << ", " << linear_velocity(2) << "\n"
              << "angular velocity = \t" << angular_velocity(0) << ", " << angular_velocity(1) << ", " << angular_velocity(2) << "\n"
              << "omega_bias = \t" << Eigen::Vector3d(para_bw_vec[1]).transpose() << "\n"
              << "linear_bias = \t" << Eigen::Vector3d(para_bv_vec[1]).transpose() << "\n"
              << "T_e_r = \t" << T_e_r << "\n"
              << "time_offset = \t" << timeoffset_e_r << std::endl;
  
    // ceres_debug_path
    // for estimation residual
    estimator->GetResidualSummary().PrintSummary(twist_trajctory->minTimeNs(),
                                                 twist_trajctory->getDtNs());          
  */
    // double pose_time = it->timestamp + relative_start_time;
    // Save_Result(pose_time, pose, bias);0
    // Save_Result(pose_time, pose, bias, linear_velocity, angular_velocity);

    // Save_Result(pose_time, pose, bias, linear_velocity, angular_velocity);
  
  }  /// 发布结果

  // check velocity estimation
  /* std::fstream vel_output_file;
  vel_output_file.open("/home/hao/Desktop/radar-event-new/src/TwistEstimator/output/estimate.tum", std::ios::out | std::ios::app);
  for(auto it_temp_ = twist2_vec_.begin(); it_temp_ <= it_base2; it_temp_++)
  {
    auto pose = trajectory_->GetRadarPose(it_temp_->timestamp);
    Eigen::Vector3d linear_velocity = pose.rotationMatrix().transpose() * trajectory_->GetTransVelWorld(it_temp_->timestamp);    // radar velocity
    Eigen::Vector3d angular_velocity = trajectory_->GetRotVelBody(it_temp_->timestamp);  
    LOG(ERROR) << "Estimate Velocity:\n"
               << "origin linear velocity = \t" << it_temp_->linear_vel_vec_(0) << ", " << it_temp_->linear_vel_vec_(1) << ", " << it_temp_->linear_vel_vec_(2) << "\n"
               << "estimate linear velocity = \t" << linear_velocity(0) << ", " << linear_velocity(1) << ", " << linear_velocity(2) << "\n"
               << "origin angular velocity = \t" << it_temp_->angular_vel_vec_(0) << ", " << it_temp_->angular_vel_vec_(1) << ", " << it_temp_->angular_vel_vec_(2) << "\n"
               << "estimate angular velocity = \t" << angular_velocity(0) << ", " << angular_velocity(1) << ", " << angular_velocity(2) << "\n";        
  
    vel_output_file << std::setprecision(20) << it_temp_->timestamp + data_start_time << " ";
    vel_output_file << linear_velocity(0) << " " << linear_velocity(1) << " " << linear_velocity(2) << " ";
    vel_output_file << angular_velocity(0) << " " << angular_velocity(1) << " " << angular_velocity(2) << " ";
    vel_output_file << std::endl;
  } 
  vel_output_file.close();*/

  for(auto it_temp_ = twist2_vec_.begin() + 1; it_temp_ <= it_base2; it_temp_++)
  {
    double ref_max_time = it_temp_->timestamp;
    // for(auto it_ref = (it_temp_ - 1)->timestamp; it_ref <= ref_max_time; it_ref += 0.01)

    // Eigen::Quaterniond q0(Eigen::Quaterniond::Identity());
    // Eigen::Vector3d t0(0.0, 0.0, 0.0);
    for(auto it_ref = (it_temp_ - 1)->timestamp; it_ref <= ref_max_time; it_ref += output_dt)
    {

      // auto pose = twist_trajctory->GetRadarPose(it_ref);
      Eigen::Vector3d linear_velocity = twist_trajctory->GetLiearVelWorld(it_ref);
      // Eigen::Vector3d linear_velocity = pose.rotationMatrix().transpose() * trajectory_->GetTransVelWorld(it_temp_->timestamp);    // radar velocity
      Eigen::Vector3d angular_velocity = twist_trajctory->GetAngularVelWorld(it_ref); 

      // Eigen::Vector3d linear_velocity = twist_trajctory->GetLiearVelWorld(it->timestamp);    // radar velocity
      // Eigen::Vector3d angular_velocity = twist_trajctory->GetAngularVelWorld(it->timestamp);  
      if(ref_max_time == it_ref)
        LOG(ERROR) << "Estimate Velocity:\n"
                  << "origin linear velocity = \t" << it_temp_->linear_vel_vec_(0) << ", " << it_temp_->linear_vel_vec_(1) << ", " << it_temp_->linear_vel_vec_(2) << "\n"
                  << "estimate linear velocity = \t" << linear_velocity(0) << ", " << linear_velocity(1) << ", " << linear_velocity(2) << "\n"
                  << "origin angular velocity = \t" << it_temp_->angular_vel_vec_(0) << ", " << it_temp_->angular_vel_vec_(1) << ", " << it_temp_->angular_vel_vec_(2) << "\n"
                  << "estimate angular velocity = \t" << angular_velocity(0) << ", " << angular_velocity(1) << ", " << angular_velocity(2) << "\n" 
                  << "point cloud size = " << it_temp_->point_cloud.width << "\n"
                  << "inliers size = " << it_temp_->best_inliers.size() << "\n"
                  << "flow size = " << it_temp_->best_flow.size() << "\n"
                  << "origin linear norm = " << it_temp_->linear_vel_vec_.norm() << "\n"
                  << "linear norm = " << linear_velocity.norm() << "\n";


      auto bias = GetLatestBias();
      // auto pose = trajectory_->GetRadarPose(it->timestamp);
      // Eigen::Vector3d linear_velocity = pose.rotationMatrix().transpose() * trajectory_->GetTransVelWorld(it->timestamp);    // radar velocity
      // Eigen::Vector3d angular_velocity = trajectory_->GetRotVelBody(it->timestamp);  
      /*Eigen::Quaterniond q_extract(pose.rotationMatrix());
      // Eigen::Vector3d t_extract = pose.translation();
      Eigen::Vector3d t_extract = pose.rotationMatrix() * pose.translation();
      ExtrinsicParam Extrin_e_r = twist_trajctory->GetSensorEP(EventSensor);
      Eigen::Matrix4d T_e_r = Extrin_e_r.Get_T();
      double timeoffset_e_r = Extrin_e_r.Get_Timeoffset();*/
      double pose_time = it_ref + relative_start_time;

      // Eigen::Quaterniond q = angular_velocity 
      // Sophus::SE3d pose(q0, t0);
      // 时间步长
      double dt = ref_max_time;

      // 平均速度
      Eigen::Vector3d next_linear_velocity = twist_trajctory->GetLiearVelWorld(std::min(it_ref + output_dt, ref_max_time));
      Eigen::Vector3d next_angular_velocity = twist_trajctory->GetAngularVelWorld(std::min(it_ref + output_dt, ref_max_time)); 
      Eigen::Vector3d linear_avg = 0.5 * (linear_velocity + next_linear_velocity);
      Eigen::Vector3d angular_avg = 0.5 * (angular_velocity + next_angular_velocity);

      // 角速度积分为增量旋转（使用小角度近似）
      double angle = angular_avg.norm() * output_dt;
      Eigen::Vector3d axis = angular_avg.normalized();

      // 防止除以零
      Eigen::Quaterniond dq;
      if (angle < 1e-8) {
          dq = Eigen::Quaterniond::Identity();  // 近似无旋转
      } else {
          dq = Eigen::AngleAxisd(angle, axis);  // 转为四元数
      }

      // 积分更新
      q0 = q0 * dq;
      t0 = t0 + q0.toRotationMatrix() * (linear_avg * output_dt);

      // 构造位姿
      Sophus::SE3d pose(q0, t0);
      Save_Result(pose_time, pose, bias, linear_velocity, angular_velocity);
    
      // debug: 是否能从多普勒点云恢复速度估计
      /*{
        const double twist_timestamp = it_temp_->timestamp;
        LOG(ERROR) << "twist time = " << twist_timestamp << ", maxTime = " << trajectory_->maxTime(RadarSensor) << " dt = " << trajectory_->getDt() << std::endl;
        LOG(ERROR) << "twist in " << (int)((twist_timestamp - last_max_time) / trajectory_->getDt());

        LOG(ERROR) << "Estimate Velocity INFO:" << std::endl;
        sensor_msgs::PointCloud2ConstIterator<float> iter_x(it_temp_->point_cloud, "x");
            sensor_msgs::PointCloud2ConstIterator<float> iter_y(it_temp_->point_cloud, "y");
            sensor_msgs::PointCloud2ConstIterator<float> iter_z(it_temp_->point_cloud, "z");
            sensor_msgs::PointCloud2ConstIterator<float> iter_doppler(it_temp_->point_cloud, "doppler");

            // std::vector<Eigen::Vector3d> pt_vec;
            // std::vector<double> pt_doppler_vec;
            Eigen::MatrixXd M_A(it_temp_->point_cloud.width,3);
            Eigen::VectorXd M_b(it_temp_->point_cloud.width);
            for (int i = 0; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z, ++iter_doppler, i++) {
                //
                Eigen::Vector3d pt;
                pt << *iter_x, *iter_y, *iter_z;
                // 提取当前点的 doppler 值
                float pt_doppler = *iter_doppler;

                M_A.block<1,3>(i, 0) = pt.normalized().transpose();
                M_b(i) = pt_doppler;
            }
            LOG(ERROR) << "M_A = " << M_A << std::endl;
            LOG(ERROR) << "M_b = " << M_b << std::endl;

            Eigen::Vector3d lsq_vel;
            Eigen::HouseholderQR<Eigen::MatrixXd> qr(M_A);
            lsq_vel = qr.solve(- M_b);

            LOG(ERROR) << "lsq_vel = " << lsq_vel.transpose() << std::endl;
      }*/

      // std::fstream norm_vel("/home/hao/Desktop/radar-event-new/src/TwistEstimator/output/norm.csv", std::ios::out | std::ios::app); 
      // norm_vel << linear_velocity.norm() << std::endl;
      // norm_vel.close();

      // 机体系下的速度发布
      geometry_msgs::TwistWithCovarianceStamped twist_esti_;
      twist_esti_.header.frame_id = "estimate";
      double twist_timestamp = it_ref + relative_start_time;
      twist_esti_.header.stamp.fromSec(twist_timestamp);

      twist_esti_.twist.twist.linear.x = linear_velocity(0);
      twist_esti_.twist.twist.linear.y = linear_velocity(1);
      twist_esti_.twist.twist.linear.z = linear_velocity(2);

      twist_esti_.twist.twist.angular.x = angular_velocity(0);
      twist_esti_.twist.twist.angular.y = angular_velocity(1);
      twist_esti_.twist.twist.angular.z = angular_velocity(2);

      pub_spline_twist_.publish(twist_esti_);
    }
  }

 // 1-24 修改  
 /*
 double dt = trajectory_->getDt();
  {
    auto bias = GetLatestBias();
    // PublishLatestBias(bias.omega_bias, bias.vel_bias);
    LOG(ERROR) << "check output time = " << cur_time - last_time << "knot time = " << dt << std::endl;
    LOG(ERROR) << "check output condition = " << last_time + 0.5 * dt << " util = " << cur_time << std::endl;
    for(double time = last_time + 0.5 * dt; time < cur_time; time += (0.5 * dt))
    {
      auto pose = trajectory_->GetRadarPose(time);

      auto linear_velocity = pose.rotationMatrix().transpose() * trajectory_->GetTransVelWorld(time);    // radar velocity
      auto angular_velocity = trajectory_->GetRotVelBody(time);  

      // Eigen::Quaterniond q_extract = pose.rotationQuaternion();
      Eigen::Quaterniond q_extract(pose.rotationMatrix());
      Eigen::Vector3d t_extract = pose.translation();
      
      // extrincs parameter event w.r.t to radar
      ExtrinsicParam Extrin_e_r = trajectory_->GetSensorEP(EventSensor);
      Eigen::Matrix4d T_e_r = Extrin_e_r.Get_T();
      double timeoffset_e_r = Extrin_e_r.Get_Timeoffset();

      LOG(ERROR) << "Estimate Result:\n" 
                << "timestamp = \t" << time + trajectory_->GetDataStartTime() << "\n"
                << "position = \t" << t_extract.transpose() << "\n"
                << "quaternion = \t" << q_extract.coeffs().transpose() << "\n"
                << "linear velocity = \t" << linear_velocity.transpose() << "\n"
                << "angular velocity = \t" << angular_velocity.transpose() << "\n"
                << "omega_bias = \t" << Eigen::Vector3d(para_bw_vec[1]).transpose() << "\n"
                << "linear_bias = \t" << Eigen::Vector3d(para_bv_vec[1]).transpose() << "\n"
                << "T_e_r = \t" << T_e_r << "\n"
                << "time_offset = \t" << timeoffset_e_r << std::endl;

      // ceres_debug_path
      // for estimation residual
      estimator->GetResidualSummary().PrintSummary(trajectory_->minTimeNs(),
                                                  trajectory_->getDtNs());          

      double pose_time = time + relative_start_time;
      Save_Result(pose_time, pose, bias, linear_velocity, angular_velocity);
    }
  
  }
  */
  
  // /// 边缘化
  std::chrono::time_point<std::chrono::high_resolution_clock> time7 = std::chrono::high_resolution_clock::now();
  
  // if(doppler_factors_num > 0)
  /*{
    LOG(ERROR) << " Start to Update Prior " << std::endl;
    // UpdatePrior2();
    UpdatePriorLoose();
    LOG(ERROR) << " Update Prior " << std::endl;
  }*/
  // }   /// 边缘化s

  // 前置数据更新
  {
      if (it_base2 == twist2_vec_.begin()) {
          // 添加第一个元素到边缘化列表
          // if (!twist2_vec_.empty())
          twist2_margin_vec_.push_back(twist2_vec_.front());
          twist2_vec_.erase(twist2_vec_.begin());
      } else if (it_base2 + 1 == twist2_vec_.end()) {
          // 全部追加进边缘化列表
          twist2_margin_vec_.insert(twist2_margin_vec_.end(), twist2_vec_.begin(), twist2_vec_.end());
          twist2_vec_.clear();
      } else {
          // 追加部分 [begin, it_base2 + 1)
          LOG(ERROR) << " no twist end " << std::endl;
          twist2_margin_vec_.insert(twist2_margin_vec_.end(), twist2_vec_.begin(), it_base2 + 1);
          twist2_vec_.erase(twist2_vec_.begin(), it_base2 + 1);
      }
  }


  // HAO TODO: 加入 边缘化
  {
    // TrajectoryEstimator2::Ptr estimator
    // UpdatePrior4();
    if(use_prior)
      UpdatePrior4();

    // std::vector<double*> parameter_blocks_mutable;
    // for (const double* ptr : parameter_blocks) {
    //     parameter_blocks_mutable.push_back(const_cast<double*>(ptr));
    // }
  }
  
  std::chrono::time_point<std::chrono::high_resolution_clock> time8 = std::chrono::high_resolution_clock::now();
  /// 更新下一个窗口
  {
    LOG(ERROR) << "\nShoule Update Time: " << std::setprecision(9) << traj_max_time << ", "
               << "\nActual Update Time: " <<  update_time << ", "  
               << "\nSetForce Time: " << update_time << ", " // it->timestamp << ", "  
               << "\nRadar Min Time: " << twist_trajctory->minTime(RadarSensor) << ", " 
               << "\nRadar Max Time: " << twist_trajctory->maxTime(RadarSensor) << ", " 
               << "\nTraj Min Time: " << twist_trajctory->minTime() << ", " 
               << "\nTraj Max Time: " << twist_trajctory->maxTime() << ", " 
               << std::endl;           

    // 更新轨迹时间
    // trajectory_->UpdateActiveTime(update_time);
    twist_trajctory->UpdateActiveTime(update_time);
    LOG(ERROR) << " Update Active Time " << update_time << std::endl;

    twist_trajctory->SetForcedFixedTime(it->timestamp);         // 毫米波雷达无畸变,只有一个更新时间
    LOG(ERROR) << " Set Forced Fixed Time " << update_time << std::endl;

    // HAO Debug: 检查更新和固定时间
    LOG(ERROR) << "activate time = " << twist_trajctory->GetActiveTime()  << std::endl;
    LOG(ERROR) << "forcefixed time = " << twist_trajctory->GetForcedFixedTime()  << std::endl;

    // 时间更新
    LOG(ERROR) << " Update Time " << std::setprecision(9) << last_time << "  |-->  " << cur_time << std::endl;
    LOG(ERROR) << " Duration Time " << std::setprecision(9) << cur_time - last_time << std::endl;
    
    // 偏置更新
    all_twist_bias_[last_time].omega_bias = Eigen::Vector3d(para_bw_vec[0]);
    all_twist_bias_[last_time].vel_bias = Eigen::Vector3d(para_bv_vec[0]);
    all_twist_bias_[cur_time].omega_bias = Eigen::Vector3d(para_bw_vec[1]);
    all_twist_bias_[cur_time].vel_bias = Eigen::Vector3d(para_bv_vec[1]);
    last_time = cur_time;

    /*PublishTrajectoryAndMap(twist_trajctory, twist_trajctory->minTime(RadarSensor), 
                            twist_trajctory->maxTime(RadarSensor), twist_trajctory->getDt() * 2);*/

    // if (it_base2 == twist2_vec_.begin()) {
    //     // 添加第一个元素到边缘化列表
    //     // if (!twist2_vec_.empty())
    //     twist2_vec_.erase(twist2_vec_.begin());
    // } else if (it_base2 + 1 == twist2_vec_.end()) {
    //     // 全部追加进边缘化列表
    //     twist2_vec_.clear();
    // } else {
    //     // 追加部分 [begin, it_base2 + 1)
    //     LOG(ERROR) << " no twist end " << std::endl;
    //     twist2_vec_.erase(twist2_vec_.begin(), it_base2 + 1);
    // }

    // 插值状态
    // 删除使用后的速度信息
    LOG(ERROR) << " twist2_vec_.before_size() = " << twist2_vec_.size() << std::endl;
    LOG(ERROR) << " it_base2 index = " << it_base2 - twist2_vec_.begin() + 1 << std::endl;
    /*if(it_base2 == twist2_vec_.begin())
      twist2_vec_.erase(twist2_vec_.begin());
    else
      if(it_base2 + 1 == twist2_vec_.end())
        twist2_vec_.clear();
      else
      {
        LOG(ERROR) << " no twist end " << std::endl;
        twist2_vec_.erase(twist2_vec_.begin(), it_base2 + 1); 
      }  */

      // twist2_margin_vec_

        
      // twist2_vec_.erase(twist2_vec_.begin(), it_base2);
    // ((it == twist2_vec_.end())? twist2_vec_.end(): it + 1));

    LOG(ERROR) << " twist2_vec_.size() = " << twist2_vec_.size() << std::endl;
    LOG(ERROR) << " all_twist_bias_.size() = " << all_twist_bias_.size() << std::endl;
  }
    
  std::chrono::time_point<std::chrono::high_resolution_clock> end_time = std::chrono::high_resolution_clock::now();
  {
      std::chrono::duration<double, std::milli> elapsed;
      elapsed = end_time - start_time;
      LOG(ERROR) << "Total Time: " << elapsed.count() << std::endl;
      elapsed = time1 - start_time;
      LOG(ERROR) << "Segment Data: " << elapsed.count() << std::endl;
      elapsed = time3 - time1;
      LOG(ERROR) << "Optimize: " << elapsed.count() << std::endl;
      elapsed = time4 - time3;
      LOG(ERROR) << "Construct Marginize: " << elapsed.count() << std::endl;
      elapsed = time5 - time4;
      LOG(ERROR) << "Construct: " << elapsed.count() << std::endl;
      elapsed = time6 - time5;
      LOG(ERROR) << "Solve: " << elapsed.count() << std::endl;
      elapsed = time7 - time6;
      LOG(ERROR) << "Publish Result: " << elapsed.count() << std::endl;
      elapsed = time8 - time7;
      LOG(ERROR) << "Marginize: " << elapsed.count() << std::endl;
      elapsed = end_time - time8;
      LOG(ERROR) << "Update Time: " << elapsed.count() << std::endl;
  }

  /// for debug
  /*
  {
    std::cout << "Pause press any key to continue, 'q' or 'Q' for exit!" << std::endl;
    char ch = std::getchar();
    // char ch = 'q';
    // 检查用户是否按下 'q' 键
    if (ch == 'q' || ch == 'Q') {
        std::cout << "Exiting TwistEstimator..." << std::endl;
        ros::shutdown();  // 关闭 ROS 系统
        exit(0);  // 退出程序
    }
  }/// debug 
  */

  /// compare for gt

}

// LooseEstimate
void UpdatePrior5()
{
  // twist2_margin_vec_
  LOG(ERROR) << "Marginize: TEST [" << prior_ctrl_id.first << ", " << prior_ctrl_id.second 
            << "], Knots = " << twist_trajctory->numKnots()<< std::endl;

  // return;

  if(prior_ctrl_id.first < 0 || prior_ctrl_id.second < prior_ctrl_id.first + 3)
  {
    LOG(ERROR) << "Initialze " << std::endl;
    return;
  }
  TrajectoryEstimatorOptions option;
  option.is_marg_state = true;
  option.marg_bias_param = true;  
  // option.marg_gravity_param = true;
  if (opt_time_offset_ && start_opt_time_offset_) {
    option.marg_t_offset_param = false;
  } else {
    option.marg_t_offset_param = true;
  }
  option.ctrl_to_be_opt_now = prior_ctrl_id.first;
  option.ctrl_to_be_opt_later = prior_ctrl_id.second;

  LOG(ERROR) << "Marginize: SET Option" << std::endl;

  LOG(ERROR) << "twist2_margin_vec_.size = " << twist2_margin_vec_.size() << std::endl;
  if(twist2_margin_vec_.empty())
    return;

  LOG(ERROR) << "spline knots = " << twist_trajctory->getKnotSize() << std::endl;
  LOG(ERROR) << "minTime = " << twist_trajctory->minTime() << std::endl;
  LOG(ERROR) << "maxTime = " << twist_trajctory->maxTime() << std::endl;
  for (auto it = twist2_margin_vec_.begin(); it != twist2_margin_vec_.end(); ) {
    int data_idx = twist_trajctory->GetCtrlIndex(it->timestamp * S_TO_NS);
    LOG(ERROR) << "data_idx = " << data_idx << std::endl; // 测试滑窗内的控制点标签
    LOG(ERROR) << "data timestamp = " << it->timestamp << std::endl; // 测试滑窗内的控制点标签
    if (data_idx < prior_ctrl_id.first) {
        it = twist2_margin_vec_.erase(it); // 正确方式：erase 返回下一个合法迭代器
    } else {
        ++it;
    }
  }


  std::map<int, double*> para_bw_vec;
  std::map<int, double*> para_bv_vec;
  {
    // 分配内存并深拷贝 omega_bias 数据
    para_bw_vec[0] = new double[3];
    std::memcpy(para_bw_vec[0], all_twist_bias_.at(last_time).omega_bias.data(), 3 * sizeof(double));

    para_bw_vec[1] = new double[3];
    std::memcpy(para_bw_vec[1], all_twist_bias_.at(cur_time).omega_bias.data(), 3 * sizeof(double));

    // 分配内存并深拷贝 vel_bias 数据
    para_bv_vec[0] = new double[3];
    std::memcpy(para_bv_vec[0], all_twist_bias_.at(last_time).vel_bias.data(), 3 * sizeof(double));

    para_bv_vec[1] = new double[3];
    std::memcpy(para_bv_vec[1], all_twist_bias_.at(cur_time).vel_bias.data(), 3 * sizeof(double));
  }

  // 构建问题
  TrajectoryEstimator2::Ptr estimator(
      new TrajectoryEstimator2(twist_trajctory, option, "Update Prior4"));

  if (marg_info) {
    std::vector<double*> drop_param_set;
    for (int i = prior_ctrl_id.first; i < prior_ctrl_id.second;   // 只边缘化范围内的点,之后的点只是产生联系
         ++i) {
      drop_param_set.emplace_back(twist_trajctory->GetAngularSpline().getKnot(i).data());
      // drop_param_set.emplace_back(trajectory_->getKnotPos(i).data());

      LOG(ERROR) << "marg spline in " << i << std::endl;
    }
    // // last bias
    // drop_param_set.emplace_back(para_bv_vec[0]);  // in the prior bias
    // drop_param_set.emplace_back(para_bw_vec[0]);

    // marg_parameter_blocks 里面没有 prior_ctrl_id 中的点 (样条移动)
    std::vector<int> drop_set;
    for (int j = 0; j < (int)marg_parameter_blocks.size(); j++) {
      for (auto const& drop_param : drop_param_set) {
        if (marg_parameter_blocks[j] == drop_param) {
          // LOG(ERROR) << "drop set " << std::endl;
          drop_set.emplace_back(j);
          break;
        }
      }
    }
    LOG(ERROR) << "drop_set.size = " << drop_set.size() << std::endl;

    if (!drop_set.empty()) {
      MarginalizationFactor* marginalization_factor =
          new MarginalizationFactor(marg_info);

      estimator->PrepareMarginalizationInfo(RType_Prior, marginalization_factor,
                                            NULL, marg_parameter_blocks,
                                            drop_set);
      LOG(ERROR) << "Marginasize: Prepare Marginalization Info" << std::endl;
      LOG(ERROR) << "Marginasize: Add Marginalization Size " << drop_set.size() << std::endl;
    }
  }
  else
    LOG(ERROR) << "Marginasize: No Marginalization Info" << std::endl;

  // 构建量测的边缘化因子
  /// [step1] add radar doppler features 这里和优化部分是基本一致的,除了加入先验信息
  // 评估本次优化的误差,加入下一次先验
  bool marg_this_factor = true;
  // [1] 多普勒残差
  Eigen::Matrix3d R_r_e = twist_trajctory->GetSensorEP(EventSensor).q.toRotationMatrix();
  LOG(ERROR) << "Marginize Size: " << twist2_margin_vec_.end() - twist2_margin_vec_.begin() + 1 << std::endl;
  for(auto it_temp_ = twist2_margin_vec_.begin(); it_temp_ != twist2_margin_vec_.end(); it_temp_++)
  {
    // [step5-1] 多普勒残差
    const double twist_timestamp = it_temp_->timestamp;
    // LOG(ERROR) << "twist time = " << twist_timestamp << ", maxTime = " << trajectory_->maxTime(RadarSensor) << " dt = " << trajectory_->getDt() << std::endl;
    // LOG(ERROR) << "twist in " << (int)((twist_timestamp - last_max_time) / trajectory_->getDt());
    // if(last_max_time > twist_timestamp)
    //   continue;

    LOG(ERROR) << "twist time = " << twist_timestamp << ", maxTime = " << twist_trajctory->maxTime(RadarSensor) << " dt = " << trajectory_->getDt() << std::endl;
    // LOG(ERROR) << "twist in " << (int)((twist_timestamp - last_max_time) / twist_trajctory->getDt());

    Eigen::Quaterniond q_e_r = twist_trajctory->GetSensorEP(EventSensor).q;
    Eigen::Vector3d t_e_r = twist_trajctory->GetSensorEP(EventSensor).p;

    estimator->AddBodyLocalVelocityMeasurementAnalytic(twist_timestamp, para_bv_vec[1], 
            it_temp_->linear_vel_vec_, linear_weight, linear_w_weight, R_weight, true);
    estimator->AddBodyLocalAngularVelocityMeasurementAnalytic(twist_timestamp, para_bw_vec[1], 
                it_temp_->angular_vel_vec_, omega_weight, omega_w_weight, true);
    }

  // LOG(ERROR) << "Marginasize: Add Event Flow Measurement" << std::endl;

  estimator->SaveMarginalizationInfo(marg_info,
                                     marg_parameter_blocks);
  if(marg_info != nullptr)
    LOG(ERROR) << "[After Prior]  marg/left: " << marg_info->m << "/"
            << marg_info->n;
  else
    LOG(ERROR) << "Failed Margin info" << std::endl;

  LOG(ERROR) << "Save Marginalization Info " << std::endl;
}


void LooseEstimate()
{
  LOG(ERROR) << "  ----------------- LooseEstimate -------------------- " << std::endl;
  std::chrono::time_point<std::chrono::high_resolution_clock> start_time = std::chrono::high_resolution_clock::now();

  if(!initial_done)
    return ;

  if(twist2_vec_.empty())
    return;
  LOG(ERROR) << "twist2_vec_.size = " << twist2_vec_.size() << std::endl;

  // [step4] 根据样条段，分割当前区间
 
  // double last_max_time = (twist_spline.maxTimeNs() - EP_RtoI.t_offset_ns) * NS_TO_S;
  double last_max_time = twist_trajctory->maxTime(RadarSensor);
  double traj_max_time = last_max_time + t_add; // 单位: 秒
  LOG(ERROR) << "last traj_max_time = " << last_max_time << std::endl;
  LOG(ERROR) << "traj_max_time = " << traj_max_time << std::endl;
  LOG(ERROR) << "twist2_vec_.back().timestamp = " << twist2_vec_.back().timestamp << std::endl;
  if(twist2_vec_.back().timestamp < traj_max_time)
    return;

  auto it = std::find_if(twist2_vec_.rbegin(), twist2_vec_.rend(), 
                      [traj_max_time](const TwistData2& data) {
                          return data.timestamp < traj_max_time;
                      });

  for(auto& twis : twist2_vec_)
  {
    LOG(ERROR) << "loop timestamp = " << twis.timestamp << std::endl;
  }

  // SE3d last_knot = trajectory_->getLastKnot();
  // Eigen::Matrix<double,6, 1> last_knot = twist_trajctory->getLastKnot();
  LOG(ERROR) << "Get Last Knot" << std::endl;
  Eigen::Vector3d last_linear_knot, last_angular_knot;
  if(twist_trajctory->getKnotSize() < 1)  // 为空
  {
    last_linear_knot.setZero();
    last_angular_knot.setZero();
  }
  // else
  // {
  //   Eigen::VectorXd last_knots(6);
  //   // last_knots.resize(6);
  //   last_knots = twist_trajctory->getLastKnots();
  //   last_linear_knot = last_knots.block<3,1>(0,0);
  //   last_angular_knot = last_knots.block<3,1>(3,0);
  // }
  else 
  {
    auto [lin_knot, ang_knot] = twist_trajctory->getLastKnots();
    last_linear_knot = lin_knot;
    last_angular_knot = ang_knot;
  }


  // discard
  /*{
    // Eigen::Matrix<double, 6, 1>last_knot = twist_trajctory->getLastKnot();
    // Eigen::Matrix<double, 6, 1>last_knot;
    // last_knot.setZero();        // 速度空间清零
    // Eigen::Vector3d last_linear_knot = twist_spline.getLinearKnots().back();
    // Eigen::Vector3d last_angular_knot = twist_spline.getAngularKnots().back();
  }*/

  if(it == twist2_vec_.rend())  // 没有合理的数据,直接返回,这种一般是存在缺失值
  {
    LOG(ERROR) << " Not Suitable Data " << std::endl;

    // twist_spline.extendKnotsTo(traj_max_time * S_TO_NS, last_linear_knot, last_angular_knot);

    // trajectory_->extendKnotsTo(traj_max_time * S_TO_NS, last_knot); // 扩展到最新的数据
    twist_trajctory->extendKnotsTo(traj_max_time * S_TO_NS, last_linear_knot, last_angular_knot); // 扩展到最新的数据
    // twist_spline.SetActiveTime(traj_max_time);
    // trajectory_->SetActiveTime(traj_max_time);
    twist_trajctory->SetActiveTime(traj_max_time);
    LOG(ERROR) << " Update Active Time " << traj_max_time << std::endl;

    // twist_spline.SetForcedFixedTime(traj_max_time);
    // trajectory_->SetForcedFixedTime(traj_max_time);         // 毫米波雷达无畸变,只有一个更新时间
    twist_trajctory->SetForcedFixedTime(traj_max_time);
    LOG(ERROR) << " Set Forced Fixed Time " << traj_max_time << std::endl;

    LOG(ERROR) << "Piror_id.first = " << prior_ctrl_id.first << ", " << prior_ctrl_id.second << std::endl;
    
    /*{
      LOG(ERROR) << "twist2_vec_.size()" << twist2_vec_.size() << std::endl;
      LOG(ERROR) << "it_base2 - twist2_vec_.begin()" << it_base2 - twist2_vec_.begin() << std::endl;
      if(it_base2 == twist2_vec_.begin())
          twist2_vec_.erase(twist2_vec_.begin());
      else
          if(it_base2 + 1 == twist2_vec_.end())
            twist2_vec_.clear();
          else
          twist2_vec_.erase(twist2_vec_.begin(), it_base2 + 1);

      // if (it_base2 != twist2_vec_.end()) {
      //     twist2_vec_.erase(twist2_vec_.begin(), std::next(it_base2));

    }*/

    // 清空边缘化信息
    marg_info = nullptr;
  //  for (double* parameters : marg_parameter_blocks) {
  //       delete[] parameters;
  //   }
    marg_parameter_blocks.clear();
    LOG(ERROR) << "Missing Data" << std::endl;
    return;
  }

  it_base2 = ((it == twist2_vec_.rend())? twist2_vec_.begin() : it.base() - 1); // it 的正向跌代器
  LOG(ERROR) << "it->timestamp = " << it->timestamp << std::endl;
  LOG(ERROR) << "it_base2->timestamp = " << it_base2->timestamp << std::endl;
  assert(it->timestamp == it_base2->timestamp && "Not Same Place");
  // 确定终点
  cur_time = it_base2->timestamp;
  all_twist_bias_[cur_time] = all_twist_bias_[last_time];
  double update_time = cur_time;  // 注意是相对时间的 秒
  
  // LOG(ERROR) << "CHECK max time 1 = " << trajectory_->maxTime(RadarSensor) << std::endl;
  LOG(ERROR) << "it->timestamp * S_TO_NS = " << it->timestamp * S_TO_NS << std::endl;
  // trajectory_->extendKnotsTo(it->timestamp * S_TO_NS, last_knot); // 扩展到最新的数据


  twist_trajctory->extendKnotsTo(traj_max_time * S_TO_NS, last_linear_knot, last_angular_knot); // 扩展到最新的数据
  // twist_spline.extendKnotsTo(it->timestamp * S_TO_NS, last_linear_knot, last_angular_knot);
  // LOG(ERROR) << "CHECK max time 2 = " << trajectory_->maxTime(RadarSensor) << std::endl;

  InitPrior();

  // Debug: 某端时间是合理的
  /*{
    LOG(ERROR) << " trajectory_ extend knots timestamp: " << std::setprecision(9) << it->timestamp << std::endl;
    LOG(ERROR) << " check update timestamp: " << std::setprecision(9) << it->timestamp - traj_max_time << std::endl;  // should be <
    // 如果下一个存在,检查一下下一个的时间戳
    if(it_base2 != twist2_vec_.end() && (it_base2 + 1) != twist2_vec_.end())
    {
      LOG(ERROR) << " trajectory_ extend knots timestamp: " << std::setprecision(9) << (it + 1)->timestamp << std::endl;
      LOG(ERROR) << " check update timestamp: " << std::setprecision(9) << (it - 1)->timestamp - traj_max_time << std::endl;
    }
  }*/

  std::map<int, double*> para_bw_vec;
  std::map<int, double*> para_bv_vec;
  {
    // 分配内存并深拷贝 omega_bias 数据
    para_bw_vec[0] = new double[3];
    std::memcpy(para_bw_vec[0], all_twist_bias_.at(last_time).omega_bias.data(), 3 * sizeof(double));

    para_bw_vec[1] = new double[3];
    std::memcpy(para_bw_vec[1], all_twist_bias_.at(cur_time).omega_bias.data(), 3 * sizeof(double));

    // 分配内存并深拷贝 vel_bias 数据
    para_bv_vec[0] = new double[3];
    std::memcpy(para_bv_vec[0], all_twist_bias_.at(last_time).vel_bias.data(), 3 * sizeof(double));

    para_bv_vec[1] = new double[3];
    std::memcpy(para_bv_vec[1], all_twist_bias_.at(cur_time).vel_bias.data(), 3 * sizeof(double));
  }
  std::chrono::time_point<std::chrono::high_resolution_clock> time1 = std::chrono::high_resolution_clock::now();

  // // decide prior index
  double opt_min_time = twist2_vec_.begin()->timestamp;
  // LOG(ERROR) << " opt_min_time = " << opt_min_time << std::endl;
  int opt_idx = twist_trajctory->GetCtrlIndex(opt_min_time * S_TO_NS);
  LOG(ERROR) << "opt_idx = " << opt_idx << std::endl;
  // // option.ctrl_to_be_opt_now = std::min(prior_ctrl_id.first, opt_idx);

  // option.ctrl_to_be_opt_now = opt_idx;
  double opt_max_time = it_base2->timestamp;
  // int scan_idx = twist_trajctory->GetCtrlIndex(opt_max_time * S_TO_NS);
  // option.ctrl_to_be_opt_later = std::max(scan_idx, twist_trajctory->Get_N());
  // LOG(ERROR) << "check ctrl later idx = std::max [" << scan_idx 
  //            << ", " << twist_trajctory->Get_N() << "]" << std::endl;

  TrajectoryEstimatorOptions option;
  option.lock_EPs.at(EventSensor).Unlock();
  option.is_marg_state = false;
  option.show_residual_summary = true;
  option.ctrl_to_be_opt_now = prior_ctrl_id.first;
  option.ctrl_to_be_opt_later = prior_ctrl_id.second;


  // [step5] 创建优化器
  // TrajectoryEstimator::Ptr estimator(new TrajectoryEstimator(trajectory_, option, "Update Traj"));
  TrajectoryEstimator2::Ptr estimator(new TrajectoryEstimator2(twist_trajctory, option, "Update Traj"));
  LOG(ERROR) << " start estimate " << std::endl;


  size_t knots_num = twist_trajctory->getKnotSize();
  LOG(ERROR) << "spline knots_num = " << knots_num << std::endl;

  // LOG(ERROR) << "Radar max time = " << trajectory_->maxTime(RadarSensor)  << std::endl;
  // LOG(ERROR) << "activate time = " << trajectory_->GetActiveTime()  << std::endl;
  // LOG(ERROR) << "forcefixed time = " << trajectory_->GetForcedFixedTime()  << std::endl;
  // LOG(ERROR) << "optimization time between [" << opt_min_time << ", " << opt_max_time << "]" << std::endl;
  // LOG(ERROR) << "optimization ctrl idx between [" << option.ctrl_to_be_opt_now 
  //            << ", " << option.ctrl_to_be_opt_later << "]" << std::endl;

  LOG(ERROR) << "last max time = " << last_max_time << std::endl;
  LOG(ERROR) << "activate time = " << twist_trajctory->GetActiveTime()  << std::endl;
  LOG(ERROR) << "forcefixed time = " << twist_trajctory->GetForcedFixedTime()  << std::endl;
  LOG(ERROR) << "optimization time between [" << opt_min_time << ", " << opt_max_time << "]" << std::endl;
  LOG(ERROR) << "optimization ctrl idx between [" << option.ctrl_to_be_opt_now 
             << ", " << option.ctrl_to_be_opt_later << "]" << std::endl;

  // HAO TODO: 这部分暂时不需要
  // TODO: 确定 prior_ctrl_id.first 的数值
  /*if (LocatedInFirstSegment2(opt_min_time)) {
    // estimator->SetFixedIndex(trajectory_->Get_N() - 1);
    estimator->SetFixedIndex(twist_trajctory->Get_N() - 1);
    LOG(ERROR) << "estimator->SetFixedIndex = " << twist_trajctory->Get_N() - 1 << std::endl;
    // LOG(ERROR) << "estimator->SetFixedIndex = " << trajectory_->Get_N() - 1 << std::endl;
    // estimator->AddStartTimePose(original_pose_);
  } else {
    estimator->SetFixedIndex(prior_ctrl_id.first - 1);
    LOG(ERROR) << "estimator->SetFixedIndex = " << prior_ctrl_id.first - 1 << std::endl;
    LOG(ERROR) << "trajectory_.fixed_control_point_index_ = " << estimator->GetFixedControlIndex() << std::endl;

    // trajectory_->opt_min_init_time_tmp = opt_min_time;
    // trajectory_->opt_min_init_time_tmp = last_time;
    // trajectory_->opt_init_fixed_idx_tmp = estimator->GetFixedControlIndex();
  }*/

  estimator->SetFixedIndex(std::max(opt_idx - 1, 0));
  // estimator->SetFixedIndex(prior_ctrl_id.first - 1);
  // LOG(ERROR) << "estimator->SetFixedIndex = " << prior_ctrl_id.first - 1 << std::endl;
  LOG(ERROR) << "estimator->SetFixedIndex = " << std::max(opt_idx - 1, 0) << std::endl;
  LOG(ERROR) << "trajectory_.fixed_control_point_index_ = " << estimator->GetFixedControlIndex() << std::endl;

  // [step5] 因子图优化
  /// 因子图优化
  std::chrono::time_point<std::chrono::high_resolution_clock> time3;
  std::chrono::time_point<std::chrono::high_resolution_clock> time4;
  std::chrono::time_point<std::chrono::high_resolution_clock> time5;
  long int doppler_factors_num = 0;
  long int event_flow_factors_num = 0;

    // 雷达线速度必须存在
    time5 = std::chrono::high_resolution_clock::now();

    // LOG(ERROR) << " Add all factor into the solver " << std::endl;
    // LOG(ERROR) << "Estimator: Add Doppler Factors for " << doppler_factors_num << std::endl;
    // LOG(ERROR) << "Estimator: Add EventFlow Factors for " << event_flow_factors_num << std::endl;
    // LOG(ERROR) << "Estimator: Add Bias Factors for " << 1 << std::endl;
  //}/// 因子图优化

  LOG(ERROR) << "First [Prior Ctrl ID] = [" << prior_ctrl_id.first << ","
              << prior_ctrl_id.second << "] "
              << prior_ctrl_id.second - prior_ctrl_id.first + 1 ;

  // 松耦合
  {
    // [step5-1] 优化问题建模
    {
      time3 = std::chrono::high_resolution_clock::now();
      // assert(marg_info == nullptr);
      LOG(ERROR) << "Estimation marg_info = " << ((marg_info)? "True" : "False") << std::endl;
      // marg_info 后面会一直存在
      // [step5-0] 先验因子

        // LOG(ERROR) << "[Prior Ctrl ID] = [" << prior_ctrl_id.first << ","
        //     << prior_ctrl_id.second << "] " << std::endl;
      if (marg_info) {
        // 确定下一次边缘化的部分
        // prior_ctrl_id.first = option.ctrl_to_be_opt_later;
        // prior_ctrl_id.second = twist_trajctory->numKnots() - 1;

        // 边缘化示例子
        // [0 1 2 3]  不边缘化
        // [ [0 1 2 3] 4 5 6 7] 边缘化 [0 1 2 3]
        // [[4 5 6 7] 8 9 10 11] 边缘化 [4 5 6 7]
        // prior_ctrl_id.first = prior_ctrl_id.second + 1;
        // prior_ctrl_id.second = twist_trajctory->numKnots() - update_every_k_knot_;

        // LOG(ERROR) << "[Prior Ctrl ID] = [" << prior_ctrl_id.first << ","
        //     << prior_ctrl_id.second << "] "
        //     << prior_ctrl_id.second - prior_ctrl_id.first + 1 ;

        estimator->AddMarginalizationFactor(marg_info,
                                            marg_parameter_blocks);
        LOG(ERROR) << " Add Marginalize " << std::endl;
      }
      
      time4 = std::chrono::high_resolution_clock::now();
      // 根据量测构建因子图优化(Loop Optimization)
      Eigen::Matrix3d R_r_e = twist_trajctory->GetSensorEP(EventSensor).q.toRotationMatrix();
      LOG(ERROR) << " Twist Size = " << (it_base2 - twist2_vec_.begin() + 1) << std::endl;
      // LOG(ERROR) << " Aver Freq = " << 1.0 / (it_base2->timestamp - twist2_vec_.begin()->timestamp) / (it_base2 - twist2_vec_.begin() + 1) << " Hz" << std::endl;

      for(auto it_temp_ = twist2_vec_.begin(); it_temp_ <= it_base2; it_temp_++)
      {
        // [step5-1] 多普勒残差
        const double twist_timestamp = it_temp_->timestamp;
        // LOG(ERROR) << "twist time = " << twist_timestamp << ", maxTime = " << trajectory_->maxTime(RadarSensor) << " dt = " << trajectory_->getDt() << std::endl;
        // LOG(ERROR) << "twist in " << (int)((twist_timestamp - last_max_time) / trajectory_->getDt());
        // if(last_max_time > twist_timestamp)
        //   continue;

        LOG(ERROR) << "twist time = " << twist_timestamp << ", maxTime = " << twist_trajctory->maxTime(RadarSensor) << " dt = " << trajectory_->getDt() << std::endl;
        // LOG(ERROR) << "twist in " << (int)((twist_timestamp - last_max_time) / twist_trajctory->getDt());

        Eigen::Quaterniond q_e_r = twist_trajctory->GetSensorEP(EventSensor).q;
        Eigen::Vector3d t_e_r = twist_trajctory->GetSensorEP(EventSensor).p;
        // double omega_weight = 1.0 / (it_temp_->angular_cov.norm() + 1e-2);
        // LOG(ERROR) << " before angular_bias: " << std::endl;

        LOG(ERROR) << "it_temp_->linear_vel_vec_ = " << it_temp_->linear_vel_vec_.transpose() << std::endl;
        LOG(ERROR) << "it_temp_->angular_vel_vec_ = " << it_temp_->linear_vel_vec_.transpose() << std::endl;
        estimator->AddBodyLocalVelocityMeasurementAnalytic(twist_timestamp, para_bv_vec[1], 
                    it_temp_->linear_vel_vec_, linear_weight, linear_w_weight, R_weight, false);
        estimator->AddBodyLocalAngularVelocityMeasurementAnalytic(twist_timestamp, para_bw_vec[1], 
                    it_temp_->angular_vel_vec_, omega_weight, omega_w_weight, false);
        /*{
          // double linear_weight = 1.0 / it_temp_->linear_cov.norm();
          // double* vel_bias_ = para_bv_vec[1];

          sensor_msgs::PointCloud2ConstIterator<float> iter_x(it_temp_->point_cloud, "x");
          sensor_msgs::PointCloud2ConstIterator<float> iter_y(it_temp_->point_cloud, "y");
          sensor_msgs::PointCloud2ConstIterator<float> iter_z(it_temp_->point_cloud, "z");
          sensor_msgs::PointCloud2ConstIterator<float> iter_doppler(it_temp_->point_cloud, "doppler");
          
          LOG(ERROR) << "set pre radar pcl size: " << it_temp_->point_cloud.width << std::endl;

          // std::vector<Eigen::Vector3d> pt_vec;
          // std::vector<double> pt_doppler_vec;
          for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z, ++iter_doppler) {
              //
              Eigen::Vector3d pt;
              pt << *iter_x, *iter_y, *iter_z;
              // 提取当前点的 doppler 值
              float pt_doppler = *iter_doppler;

              // pt_vec.push_back(pt);
              // pt_doppler_vec.push_back(pt_doppler);

            // estimator->AddDopplerMeasurementAnalytic(twist_timestamp, pt, vel_bias_,
            //   pt_doppler, R_r_e, linear_weight, false);    // 边缘化在先验处进行
            estimator->AddDopplerMeasurementAnalytic2(twist_timestamp, pt, para_bv_vec[1],
              pt_doppler, R_r_e, linear_weight, linear_w_weight, option.is_marg_state);    // 边缘化在先验处进行

          }
        }*/

        // 降采样
        /*{
          int total_points = it_temp_->point_cloud.width;
          int sample_size = 25;
          sample_size = std::min(sample_size, total_points); // 防止点数不足25

          // 构建索引数组
          std::vector<int> indices(total_points);
          std::iota(indices.begin(), indices.end(), 0);

          // 随机打乱索引
          std::random_device rd;
          std::mt19937 g(rd());
          std::shuffle(indices.begin(), indices.end(), g);

          // 随机采样的索引前25个
          std::vector<int> sample_indices(indices.begin(), indices.begin() + sample_size);

          // 准备访问迭代器
          sensor_msgs::PointCloud2ConstIterator<float> iter_x(it_temp_->point_cloud, "x");
          sensor_msgs::PointCloud2ConstIterator<float> iter_y(it_temp_->point_cloud, "y");
          sensor_msgs::PointCloud2ConstIterator<float> iter_z(it_temp_->point_cloud, "z");
          sensor_msgs::PointCloud2ConstIterator<float> iter_doppler(it_temp_->point_cloud, "doppler");

          // 为方便随机访问，把点先读到vector里（可优化为直接随机访问，但ros pointcloud2迭代器不支持直接索引）
          std::vector<Eigen::Vector3d> pts(total_points);
          std::vector<float> dopplers(total_points);
          int idx = 0;
          for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z, ++iter_doppler, ++idx) {
              pts[idx] << *iter_x, *iter_y, *iter_z;
              dopplers[idx] = *iter_doppler;
          }

          LOG(ERROR) << "sample_indices.size = " << sample_indices.size() << std::endl;

          // 只对随机采样的25个点进行计算
          for (int i : sample_indices) {
              Eigen::Vector3d pt = pts[i];
              float pt_doppler = dopplers[i];
              estimator->AddDopplerMeasurementAnalytic2(twist_timestamp, pt, para_bv_vec[1],
                  pt_doppler, R_r_e, linear_weight, linear_w_weight, false);
          }
        }*/

        // double* angular_bias_ = para_bw_vec[1];
        /*
        event_flow_factors_num += it_temp_->best_inliers.size();
        LOG(ERROR) << " Add Flow Points: " << it_temp_->best_inliers.size() << std::endl;
        for(int i = 0; i < it_temp_->best_inliers.size(); i++)
        {
          Eigen::Vector3d pixel(it_temp_->best_inliers[i].x, it_temp_->best_inliers[i].y, 1.0);
          Eigen::Vector3d pixel_cord = K.inverse() * pixel;         
          // estimator->AddEventFlowMeasurementAnalytic2(twist_timestamp, pixel_cord, it_temp_->best_flow[i], it_temp_->linear_vel_vec_,
          //     q_e_r, t_e_r, para_bv_vec[1], para_bw_vec[1], time_offset, omega_weight, omega_w_weight, false);
          estimator->AddEventFlowMeasurementAnalytic3(twist_timestamp, pixel_cord, it_temp_->best_flow[i], it_temp_->linear_vel_vec_,
              q_e_r, t_e_r, para_bv_vec[1], para_bw_vec[1], time_offset, omega_weight, omega_w_weight, false);
        }
        */
        // LOG(ERROR) << "Marginasize: Add Event Flow " << std::endl;

        // TODO: twist.best_flow 是统一的光流,可以修改为每个事件的光流
        // LOG(INFO) << " Add Event Flow " << std::endl;
        LOG(ERROR) << " Add Event Flow " << std::endl;
      }
      // std::ofstream residual_file("/home/hao/Desktop/radar-event-new/src/TwistEstimator/output/res.txt", std::ios::out | std::ios::app);
      // residual_file << "done" << std::endl;
      // residual_file.close();

      // std::ofstream jac_knot_file("/home/hao/Desktop/radar-event-new/src/TwistEstimator/output/jac.txt", std::ios::out | std::ios::app);
      // jac_knot_file << "done" << std::endl;
      // jac_knot_file.close();
    }
      
      
      // [3] Twist Bias 估计
      // Bias 只跟两次量测时间相关，而和具体量测数量无关
      {
        double delta_time = cur_time - last_time;
        delta_time = std::max(delta_time, 1e-3);

        // double dt = 1. / 200.; // opt_weight_.imu_noise.imu_frequency;
        // double dt = 1. / 10.; // opt_weight_.radar_frequency;
        double dt = 1. / 20.; // opt_weight_.radar_frequency;
        double cov = delta_time / dt * (dt * dt);
        LOG(ERROR) << "delta_time = " << delta_time << std::endl;
        LOG(ERROR) << "cov = " << cov << std::endl;
        Eigen::Matrix<double, 6, 1> sqrt_info;
        sqrt_info.setOnes();
        sqrt_info *=  (1. / std::sqrt(cov)); // * opt_weight_.bias_info_vec;

        LOG(ERROR) << "100 * sqrt_info = " << 100 * sqrt_info << std::endl;

        LOG(ERROR) << " Add Bias Factor " << std::endl;

        estimator->AddBiasFactor(para_bw_vec[0], para_bw_vec[1], 
                                  para_bv_vec[0], para_bv_vec[1], 
                                  1, 100 * sqrt_info); // 100 * sqrt_info);

        LOG(ERROR) << " Add Bias: " << std::endl;
        LOG(ERROR) << "last time omega_bias = " << all_twist_bias_.at(last_time).omega_bias << std::endl;
        LOG(ERROR) << "cur time omega_bias = " << all_twist_bias_.at(cur_time).omega_bias << std::endl; 
        LOG(ERROR) << "last time vel_bias = " << all_twist_bias_.at(last_time).vel_bias << std::endl; 
        LOG(ERROR) << "cur time vel_bias = " << all_twist_bias_.at(cur_time).vel_bias << std::endl; 

      }

      LOG(ERROR) << " Add all factor into the solver " << std::endl;
      LOG(ERROR) << "Estimator: Add Doppler Factors for " << doppler_factors_num << std::endl;
      LOG(ERROR) << "Estimator: Add EventFlow Factors for " << event_flow_factors_num << std::endl;
      LOG(ERROR) << "Estimator: Add Bias Factors for " << 1 << std::endl;
    

    // 雷达线速度必须存在
    time5 = std::chrono::high_resolution_clock::now();
    // if(doppler_factors_num > 0)
    // {   
      // [step5-2] 优化问题求解
      {
        LOG(ERROR) << " start to solve problem " << std::endl;

        // ceres::Solver::Summary summary = estimator->Solve(50, false);
        ceres::Solver::Summary summary = estimator->Solve(50, false, 8);  // use multi-thread
        LOG(ERROR) << summary.BriefReport();
        LOG(ERROR) << "Traj Update Successful/Unsuccessful steps: "
                  << summary.num_successful_steps << "/"
                  << summary.num_unsuccessful_steps;
        LOG(ERROR) << "Traj Update details: ";
        LOG(ERROR) << summary.FullReport();

        LOG(ERROR) << "end to solve problem " << std::endl;
      }
    // }
  }/// 因子图优化*/




  /// 发布结果
  std::chrono::time_point<std::chrono::high_resolution_clock> time6 = std::chrono::high_resolution_clock::now();
  {
    auto bias = GetLatestBias();
    // PublishLatestBias(bias.omega_bias, bias.vel_bias);
    // auto pose = twist_trajctory->GetRadarPose(it->timestamp);
    // HAO TODO: 修改
    Eigen::Quaterniond q(Eigen::Quaterniond::Identity());
    Eigen::Vector3d t(0.0, 0.0, 0.0);
    Sophus::SE3d pose(q, t);
        
    // Eigen::Vector3d linear_velocity = pose.rotationMatrix().transpose() * trajectory_->GetTransVelWorld(it->timestamp);    // radar velocity
    Eigen::Vector3d linear_velocity = twist_trajctory->GetLiearVelWorld(it->timestamp);    // radar velocity
    Eigen::Vector3d angular_velocity = twist_trajctory->GetAngularVelWorld(it->timestamp);  

    LOG(ERROR) << "Estimate Velocity:\n"
               << "origin linear velocity = \t" << it->linear_vel_vec_(0) << ", " << it->linear_vel_vec_(1) << ", " << it->linear_vel_vec_(2) << "\n"
               << "estimate linear velocity = \t" << linear_velocity(0) << ", " << linear_velocity(1) << ", " << linear_velocity(2) << "\n"
               << "origin angular velocity = \t" << it->angular_vel_vec_(0) << ", " << it->angular_vel_vec_(1) << ", " << it->angular_vel_vec_(2) << "\n"
               << "estimate angular velocity = \t" << angular_velocity(0) << ", " << angular_velocity(1) << ", " << angular_velocity(2) << "\n";        
  

    // Eigen::Quaterniond q_extract = pose.rotationQuaternion();
    /*Eigen::Quaterniond q_extract(pose.rotationMatrix());
    Eigen::Vector3d t_extract = pose.rotationMatrix() * pose.translation();
    

    // extrincs parameter event w.r.t to radar
    ExtrinsicParam Extrin_e_r = twist_trajctory->GetSensorEP(EventSensor);
    Eigen::Matrix4d T_e_r = Extrin_e_r.Get_T();
    double timeoffset_e_r = Extrin_e_r.Get_Timeoffset();

    LOG(ERROR) << "Estimate Result:\n" 
              << "timestamp = \t" << std::setprecision(18) << it->timestamp + trajectory_->GetDataStartTime() << "\n"
              << "position = \t" << t_extract.transpose() << "\n"
              << "quaternion = \t" << q_extract.coeffs().transpose() << "\n"
              << "linear velocity = \t" << linear_velocity(0) << ", " << linear_velocity(1) << ", " << linear_velocity(2) << "\n"
              << "angular velocity = \t" << angular_velocity(0) << ", " << angular_velocity(1) << ", " << angular_velocity(2) << "\n"
              << "omega_bias = \t" << Eigen::Vector3d(para_bw_vec[1]).transpose() << "\n"
              << "linear_bias = \t" << Eigen::Vector3d(para_bv_vec[1]).transpose() << "\n"
              << "T_e_r = \t" << T_e_r << "\n"
              << "time_offset = \t" << timeoffset_e_r << std::endl;
  
    // ceres_debug_path
    // for estimation residual
    estimator->GetResidualSummary().PrintSummary(twist_trajctory->minTimeNs(),
                                                 twist_trajctory->getDtNs());          
  */
    double pose_time = it->timestamp + relative_start_time;
    // Save_Result(pose_time, pose, bias);
    // Save_Result(pose_time, pose, bias, linear_velocity, angular_velocity);

    Save_Result(pose_time, pose, bias, linear_velocity, angular_velocity);
  
  }  /// 发布结果

  // check velocity estimation
  /* std::fstream vel_output_file;
  vel_output_file.open("/home/hao/Desktop/radar-event-new/src/TwistEstimator/output/estimate.tum", std::ios::out | std::ios::app);
  for(auto it_temp_ = twist2_vec_.begin(); it_temp_ <= it_base2; it_temp_++)
  {
    auto pose = trajectory_->GetRadarPose(it_temp_->timestamp);
    Eigen::Vector3d linear_velocity = pose.rotationMatrix().transpose() * trajectory_->GetTransVelWorld(it_temp_->timestamp);    // radar velocity
    Eigen::Vector3d angular_velocity = trajectory_->GetRotVelBody(it_temp_->timestamp);  
    LOG(ERROR) << "Estimate Velocity:\n"
               << "origin linear velocity = \t" << it_temp_->linear_vel_vec_(0) << ", " << it_temp_->linear_vel_vec_(1) << ", " << it_temp_->linear_vel_vec_(2) << "\n"
               << "estimate linear velocity = \t" << linear_velocity(0) << ", " << linear_velocity(1) << ", " << linear_velocity(2) << "\n"
               << "origin angular velocity = \t" << it_temp_->angular_vel_vec_(0) << ", " << it_temp_->angular_vel_vec_(1) << ", " << it_temp_->angular_vel_vec_(2) << "\n"
               << "estimate angular velocity = \t" << angular_velocity(0) << ", " << angular_velocity(1) << ", " << angular_velocity(2) << "\n";        
  
    vel_output_file << std::setprecision(20) << it_temp_->timestamp + data_start_time << " ";
    vel_output_file << linear_velocity(0) << " " << linear_velocity(1) << " " << linear_velocity(2) << " ";
    vel_output_file << angular_velocity(0) << " " << angular_velocity(1) << " " << angular_velocity(2) << " ";
    vel_output_file << std::endl;
  } 
  vel_output_file.close();*/

  for(auto it_temp_ = twist2_vec_.begin() + 1; it_temp_ <= it_base2; it_temp_++)
  {
    double ref_max_time = it_temp_->timestamp;
    // for(auto it_ref = (it_temp_ - 1)->timestamp; it_ref <= ref_max_time; it_ref += 0.01)
    for(auto it_ref = (it_temp_ - 1)->timestamp; it_ref <= ref_max_time; it_ref += ref_max_time)
    {

      // auto pose = twist_trajctory->GetRadarPose(it_ref);
      Eigen::Vector3d linear_velocity = twist_trajctory->GetLiearVelWorld(it_ref);
      // Eigen::Vector3d linear_velocity = pose.rotationMatrix().transpose() * trajectory_->GetTransVelWorld(it_temp_->timestamp);    // radar velocity
      Eigen::Vector3d angular_velocity = twist_trajctory->GetAngularVelWorld(it_ref); 

      // Eigen::Vector3d linear_velocity = twist_trajctory->GetLiearVelWorld(it->timestamp);    // radar velocity
      // Eigen::Vector3d angular_velocity = twist_trajctory->GetAngularVelWorld(it->timestamp);  
      if(ref_max_time == it_ref)
        LOG(ERROR) << "Estimate Velocity:\n"
                  << "origin linear velocity = \t" << it_temp_->linear_vel_vec_(0) << ", " << it_temp_->linear_vel_vec_(1) << ", " << it_temp_->linear_vel_vec_(2) << "\n"
                  << "estimate linear velocity = \t" << linear_velocity(0) << ", " << linear_velocity(1) << ", " << linear_velocity(2) << "\n"
                  << "origin angular velocity = \t" << it_temp_->angular_vel_vec_(0) << ", " << it_temp_->angular_vel_vec_(1) << ", " << it_temp_->angular_vel_vec_(2) << "\n"
                  << "estimate angular velocity = \t" << angular_velocity(0) << ", " << angular_velocity(1) << ", " << angular_velocity(2) << "\n" 
                  << "point cloud size = " << it_temp_->point_cloud.width << "\n"
                  << "inliers size = " << it_temp_->best_inliers.size() << "\n"
                  << "flow size = " << it_temp_->best_flow.size() << "\n"
                  << "origin linear norm = " << it_temp_->linear_vel_vec_.norm() << "\n"
                  << "linear norm = " << linear_velocity.norm() << "\n";


      auto bias = GetLatestBias();
      // auto pose = trajectory_->GetRadarPose(it->timestamp);
      // Eigen::Vector3d linear_velocity = pose.rotationMatrix().transpose() * trajectory_->GetTransVelWorld(it->timestamp);    // radar velocity
      // Eigen::Vector3d angular_velocity = trajectory_->GetRotVelBody(it->timestamp);  
      /*Eigen::Quaterniond q_extract(pose.rotationMatrix());
      // Eigen::Vector3d t_extract = pose.translation();
      Eigen::Vector3d t_extract = pose.rotationMatrix() * pose.translation();
      ExtrinsicParam Extrin_e_r = twist_trajctory->GetSensorEP(EventSensor);
      Eigen::Matrix4d T_e_r = Extrin_e_r.Get_T();
      double timeoffset_e_r = Extrin_e_r.Get_Timeoffset();*/
      double pose_time = it_ref + relative_start_time;
      Eigen::Quaterniond q(Eigen::Quaterniond::Identity());
      Eigen::Vector3d t(0.0, 0.0, 0.0);
      Sophus::SE3d pose(q, t);
          
      Save_Result(pose_time, pose, bias, linear_velocity, angular_velocity);
    
      // debug: 是否能从多普勒点云恢复速度估计
      /*{
        const double twist_timestamp = it_temp_->timestamp;
        LOG(ERROR) << "twist time = " << twist_timestamp << ", maxTime = " << trajectory_->maxTime(RadarSensor) << " dt = " << trajectory_->getDt() << std::endl;
        LOG(ERROR) << "twist in " << (int)((twist_timestamp - last_max_time) / trajectory_->getDt());

        LOG(ERROR) << "Estimate Velocity INFO:" << std::endl;
        sensor_msgs::PointCloud2ConstIterator<float> iter_x(it_temp_->point_cloud, "x");
            sensor_msgs::PointCloud2ConstIterator<float> iter_y(it_temp_->point_cloud, "y");
            sensor_msgs::PointCloud2ConstIterator<float> iter_z(it_temp_->point_cloud, "z");
            sensor_msgs::PointCloud2ConstIterator<float> iter_doppler(it_temp_->point_cloud, "doppler");

            // std::vector<Eigen::Vector3d> pt_vec;
            // std::vector<double> pt_doppler_vec;
            Eigen::MatrixXd M_A(it_temp_->point_cloud.width,3);
            Eigen::VectorXd M_b(it_temp_->point_cloud.width);
            for (int i = 0; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z, ++iter_doppler, i++) {
                //
                Eigen::Vector3d pt;
                pt << *iter_x, *iter_y, *iter_z;
                // 提取当前点的 doppler 值
                float pt_doppler = *iter_doppler;

                M_A.block<1,3>(i, 0) = pt.normalized().transpose();
                M_b(i) = pt_doppler;
            }
            LOG(ERROR) << "M_A = " << M_A << std::endl;
            LOG(ERROR) << "M_b = " << M_b << std::endl;

            Eigen::Vector3d lsq_vel;
            Eigen::HouseholderQR<Eigen::MatrixXd> qr(M_A);
            lsq_vel = qr.solve(- M_b);

            LOG(ERROR) << "lsq_vel = " << lsq_vel.transpose() << std::endl;
      }*/

      // std::fstream norm_vel("/home/hao/Desktop/radar-event-new/src/TwistEstimator/output/norm.csv", std::ios::out | std::ios::app); 
      // norm_vel << linear_velocity.norm() << std::endl;
      // norm_vel.close();

      // 机体系下的速度发布
      geometry_msgs::TwistWithCovarianceStamped twist_esti_;
      twist_esti_.header.frame_id = "estimate";
      double twist_timestamp = it_ref + relative_start_time;
      twist_esti_.header.stamp.fromSec(twist_timestamp);

      twist_esti_.twist.twist.linear.x = linear_velocity(0);
      twist_esti_.twist.twist.linear.y = linear_velocity(1);
      twist_esti_.twist.twist.linear.z = linear_velocity(2);

      twist_esti_.twist.twist.angular.x = angular_velocity(0);
      twist_esti_.twist.twist.angular.y = angular_velocity(1);
      twist_esti_.twist.twist.angular.z = angular_velocity(2);

      pub_spline_twist_.publish(twist_esti_);
    }
  }

 // 1-24 修改  
 /*
 double dt = trajectory_->getDt();
  {
    auto bias = GetLatestBias();
    // PublishLatestBias(bias.omega_bias, bias.vel_bias);
    LOG(ERROR) << "check output time = " << cur_time - last_time << "knot time = " << dt << std::endl;
    LOG(ERROR) << "check output condition = " << last_time + 0.5 * dt << " util = " << cur_time << std::endl;
    for(double time = last_time + 0.5 * dt; time < cur_time; time += (0.5 * dt))
    {
      auto pose = trajectory_->GetRadarPose(time);

      auto linear_velocity = pose.rotationMatrix().transpose() * trajectory_->GetTransVelWorld(time);    // radar velocity
      auto angular_velocity = trajectory_->GetRotVelBody(time);  

      // Eigen::Quaterniond q_extract = pose.rotationQuaternion();
      Eigen::Quaterniond q_extract(pose.rotationMatrix());
      Eigen::Vector3d t_extract = pose.translation();
      
      // extrincs parameter event w.r.t to radar
      ExtrinsicParam Extrin_e_r = trajectory_->GetSensorEP(EventSensor);
      Eigen::Matrix4d T_e_r = Extrin_e_r.Get_T();
      double timeoffset_e_r = Extrin_e_r.Get_Timeoffset();

      LOG(ERROR) << "Estimate Result:\n" 
                << "timestamp = \t" << time + trajectory_->GetDataStartTime() << "\n"
                << "position = \t" << t_extract.transpose() << "\n"
                << "quaternion = \t" << q_extract.coeffs().transpose() << "\n"
                << "linear velocity = \t" << linear_velocity.transpose() << "\n"
                << "angular velocity = \t" << angular_velocity.transpose() << "\n"
                << "omega_bias = \t" << Eigen::Vector3d(para_bw_vec[1]).transpose() << "\n"
                << "linear_bias = \t" << Eigen::Vector3d(para_bv_vec[1]).transpose() << "\n"
                << "T_e_r = \t" << T_e_r << "\n"
                << "time_offset = \t" << timeoffset_e_r << std::endl;

      // ceres_debug_path
      // for estimation residual
      estimator->GetResidualSummary().PrintSummary(trajectory_->minTimeNs(),
                                                  trajectory_->getDtNs());          

      double pose_time = time + relative_start_time;
      Save_Result(pose_time, pose, bias, linear_velocity, angular_velocity);
    }
  
  }
  */
  
  // /// 边缘化
  std::chrono::time_point<std::chrono::high_resolution_clock> time7 = std::chrono::high_resolution_clock::now();
  
  // if(doppler_factors_num > 0)
  /*{
    LOG(ERROR) << " Start to Update Prior " << std::endl;
    // UpdatePrior2();
    UpdatePriorLoose();
    LOG(ERROR) << " Update Prior " << std::endl;
  }*/
  // }   /// 边缘化s

  // 前置数据更新
  {
      if (it_base2 == twist2_vec_.begin()) {
          // 添加第一个元素到边缘化列表
          // if (!twist2_vec_.empty())
          twist2_margin_vec_.push_back(twist2_vec_.front());
          twist2_vec_.erase(twist2_vec_.begin());
      } else if (it_base2 + 1 == twist2_vec_.end()) {
          // 全部追加进边缘化列表
          twist2_margin_vec_.insert(twist2_margin_vec_.end(), twist2_vec_.begin(), twist2_vec_.end());
          twist2_vec_.clear();
      } else {
          // 追加部分 [begin, it_base2 + 1)
          LOG(ERROR) << " no twist end " << std::endl;
          twist2_margin_vec_.insert(twist2_margin_vec_.end(), twist2_vec_.begin(), it_base2 + 1);
          twist2_vec_.erase(twist2_vec_.begin(), it_base2 + 1);
      }
  }


  // HAO TODO: 加入 边缘化
  {
    // TrajectoryEstimator2::Ptr estimator
    // ();
    if(use_prior)
      UpdatePrior5();

    // std::vector<double*> parameter_blocks_mutable;
    // for (const double* ptr : parameter_blocks) {
    //     parameter_blocks_mutable.push_back(const_cast<double*>(ptr));
    // }
  }
  
  std::chrono::time_point<std::chrono::high_resolution_clock> time8 = std::chrono::high_resolution_clock::now();
  /// 更新下一个窗口
  {
    LOG(ERROR) << "\nShoule Update Time: " << std::setprecision(9) << traj_max_time << ", "
               << "\nActual Update Time: " <<  update_time << ", "  
               << "\nSetForce Time: " << update_time << ", " // it->timestamp << ", "  
               << "\nRadar Min Time: " << twist_trajctory->minTime(RadarSensor) << ", " 
               << "\nRadar Max Time: " << twist_trajctory->maxTime(RadarSensor) << ", " 
               << "\nTraj Min Time: " << twist_trajctory->minTime() << ", " 
               << "\nTraj Max Time: " << twist_trajctory->maxTime() << ", " 
               << std::endl;           

    // 更新轨迹时间
    // trajectory_->UpdateActiveTime(update_time);
    twist_trajctory->UpdateActiveTime(update_time);
    LOG(ERROR) << " Update Active Time " << update_time << std::endl;

    twist_trajctory->SetForcedFixedTime(it->timestamp);         // 毫米波雷达无畸变,只有一个更新时间
    LOG(ERROR) << " Set Forced Fixed Time " << update_time << std::endl;

    // HAO Debug: 检查更新和固定时间
    LOG(ERROR) << "activate time = " << twist_trajctory->GetActiveTime()  << std::endl;
    LOG(ERROR) << "forcefixed time = " << twist_trajctory->GetForcedFixedTime()  << std::endl;

    // 时间更新
    LOG(ERROR) << " Update Time " << std::setprecision(9) << last_time << "  |-->  " << cur_time << std::endl;
    LOG(ERROR) << " Duration Time " << std::setprecision(9) << cur_time - last_time << std::endl;
    
    // 偏置更新
    all_twist_bias_[last_time].omega_bias = Eigen::Vector3d(para_bw_vec[0]);
    all_twist_bias_[last_time].vel_bias = Eigen::Vector3d(para_bv_vec[0]);
    all_twist_bias_[cur_time].omega_bias = Eigen::Vector3d(para_bw_vec[1]);
    all_twist_bias_[cur_time].vel_bias = Eigen::Vector3d(para_bv_vec[1]);
    last_time = cur_time;

    /*PublishTrajectoryAndMap(twist_trajctory, twist_trajctory->minTime(RadarSensor), 
                            twist_trajctory->maxTime(RadarSensor), twist_trajctory->getDt() * 2);*/

    /*if (it_base2 == twist2_vec_.begin()) {
        // 添加第一个元素到边缘化列表
        // if (!twist2_vec_.empty())
        twist2_vec_.erase(twist2_vec_.begin());
    } else if (it_base2 + 1 == twist2_vec_.end()) {
        // 全部追加进边缘化列表
        twist2_vec_.clear();
    } else {
        // 追加部分 [begin, it_base2 + 1)
        LOG(ERROR) << " no twist end " << std::endl;
        twist2_vec_.erase(twist2_vec_.begin(), it_base2 + 1);
    }*/

    // 插值状态
    // 删除使用后的速度信息
    LOG(ERROR) << " twist2_vec_.before_size() = " << twist2_vec_.size() << std::endl;
    LOG(ERROR) << " it_base2 index = " << it_base2 - twist2_vec_.begin() + 1 << std::endl;
    /*if(it_base2 == twist2_vec_.begin())
      twist2_vec_.erase(twist2_vec_.begin());
    else
      if(it_base2 + 1 == twist2_vec_.end())
        twist2_vec_.clear();
      else
      {
        LOG(ERROR) << " no twist end " << std::endl;
        twist2_vec_.erase(twist2_vec_.begin(), it_base2 + 1); 
      }  */

      // twist2_margin_vec_

        
      // twist2_vec_.erase(twist2_vec_.begin(), it_base2);
    // ((it == twist2_vec_.end())? twist2_vec_.end(): it + 1));

    LOG(ERROR) << " twist2_vec_.size() = " << twist2_vec_.size() << std::endl;
    LOG(ERROR) << " all_twist_bias_.size() = " << all_twist_bias_.size() << std::endl;
  }
    
  std::chrono::time_point<std::chrono::high_resolution_clock> end_time = std::chrono::high_resolution_clock::now();
  {
      std::chrono::duration<double, std::milli> elapsed;
      elapsed = end_time - start_time;
      LOG(ERROR) << "Total Time: " << elapsed.count() << std::endl;
      elapsed = time1 - start_time;
      LOG(ERROR) << "Segment Data: " << elapsed.count() << std::endl;
      elapsed = time3 - time1;
      LOG(ERROR) << "Optimize: " << elapsed.count() << std::endl;
      elapsed = time4 - time3;
      LOG(ERROR) << "Construct Marginize: " << elapsed.count() << std::endl;
      elapsed = time5 - time4;
      LOG(ERROR) << "Construct: " << elapsed.count() << std::endl;
      elapsed = time6 - time5;
      LOG(ERROR) << "Solve: " << elapsed.count() << std::endl;
      elapsed = time7 - time6;
      LOG(ERROR) << "Publish Result: " << elapsed.count() << std::endl;
      elapsed = time8 - time7;
      LOG(ERROR) << "Marginize: " << elapsed.count() << std::endl;
      elapsed = end_time - time8;
      LOG(ERROR) << "Update Time: " << elapsed.count() << std::endl;
  }

  // /// for debug
  // {
  //   std::cout << "Pause press any key to continue, 'q' or 'Q' for exit!" << std::endl;
  //   char ch = std::getchar();
  //   // 检查用户是否按下 'q' 键
  //   if (ch == 'q' || ch == 'Q') {
  //       std::cout << "Exiting TwistEstimator..." << std::endl;
  //       ros::shutdown();  // 关闭 ROS 系统
  //       exit(0);  // 退出程序
  //   }
  // }/// debug

  /// compare for gt

}


void printYamlNode(const YAML::Node& node) {
    // 创建一个 YAML::Emitter，用于序列化 node
    YAML::Emitter out;
    out << node;  // 将 YAML 节点写入 Emitter

    // 打印序列化后的 YAML 字符串
    if (out.good()) {
        std::cout << out.c_str() << std::endl;
    } else {
        std::cerr << "Failed to emit YAML: " << out.GetLastError() << std::endl;
    }
}

void ParseYamlFile(std::string& config_path)
{
  YAML::Node node = YAML::LoadFile(config_path);

  double knot_distance = node["knot_distance"].as<double>();
  std::cout << "knot_distance = " << knot_distance << std::endl;

  // YAML::Node temp_node = YAML::LoadFile("/home/hao/Desktop/demo.yaml");
  if (!(node["radar"]["Extrinsics"]["time_offset"])) {
      LOG(WARNING) << "missing radar time_offset" << std::endl;
  }
  else
  {
      LOG(WARNING) << "existing radar time_offset" << std::endl;
      std::cout << node["radar"]["Extrinsics"]["time_offset"] << std::endl;
  }

  if (!(node["radar"]["Extrinsics"]["Trans"])) {
      LOG(WARNING) << "missing Trans for radar" << std::endl;
  }
  else
  {
      LOG(WARNING) << "existing radar Trans" << std::endl;
      std::cout << node["radar"]["Extrinsics"]["Trans"] << std::endl;
  }
  ExtrinsicParam EP_RtoI, EP_EtoI;
  EP_RtoI.Init(node["radar"]["Extrinsics"]);
  EP_RtoI.t_offset_ns = 0;

  // LOG(WARNING) << "Initial Radar Extrinsic Param " << std::endl;
  double fx = node["event"]["fx"].as<double>();
  double fy = node["event"]["fy"].as<double>();
  double cx = node["event"]["cx"].as<double>();
  double cy = node["event"]["cy"].as<double>();

  K << fx, 0, cx,
    0, fy, cy,
    0,0,1;


  if (!(node["event_extrincs"]["Extrinsics"]["time_offset"])) {
      LOG(WARNING) << "missing event time_offset" << std::endl;
  }
  else
  {
      LOG(WARNING) << "existing event time_offset" << std::endl;
      std::cout << node["event_extrincs"]["Extrinsics"]["time_offset"] << std::endl;
  }

  if (!(node["event_extrincs"]["Extrinsics"]["Trans"])) {
      LOG(WARNING) << "missing Trans for event" << std::endl;
  }
  else
  {
      LOG(WARNING) << "existing event Trans" << std::endl;
      std::cout << node["event_extrincs"]["Extrinsics"]["Trans"] << std::endl;
  }

  EP_EtoI.Init(node["event_extrincs"]["Extrinsics"]);
  EP_EtoI.t_offset_ns = 0;

  // printYamlNode(node["event_extrincs"]["Extrinsics"]);

  update_every_k_knot_ = node["update_every_k_knot"].as<int>();

  output_path = node["output_file"].as<std::string>();
  output_dt = node["output_dt"].as<double>();

  local_dt = node["local_dt"].as<double>();

  linear_weight = node["linear_weight"].as<double>();
  linear_w_weight = node["linear_w_weight"].as<double>();
  R_weight = node["linear_R_weight"].as<double>();

  omega_weight = node["omega_weight"].as<double>();
  omega_w_weight = node["omega_w_weight"].as<double>();

  use_prior = node["use_prior"].as<bool>();
  use_fej = node["use_fej"].as<bool>();
  use_order_opti = node["use_order_opti"].as<bool>();

  // 清空文件内容
  {
    output_file.open(output_path);
    output_file.clear();
    // 加入一个表头
    output_file << "# timestamp" << ", ";  // 设置高精度输出时间戳
    output_file << "p.x" << ", " << "p.y" << ", " << "p.z" << ", ";  // 平移向量
    output_file << "q.x" << ", " << "q.y" << ", " << "q.z" << ", " << "q.w" << ", ";  // 四元数
    output_file << "omega_bias.x" << ", " << "omega_bias.y" << ", " << "omega_bias.z" << ", ";  // 角速度偏置
    output_file << "vel_bias.x" << ", " << "vel_bias.y" << ", " << "vel_bias.z" << ", ";
    output_file << "linear.x" << ", " << "linear.y" << ", " << "linear.z" << ", ";
    output_file << "angular.x" << ", " << "angular.y" << ", " << "angular.z" << std::endl;  // 速度偏置
    output_file.close();
  }

  // ceres_debug_path = node["ceres_debug_file"].as<std::string>();
  // {
  //   ceres_debug_file.open(ceres_debug_path);
  //   ceres_debug_file.clear();
  //   ceres_debug_file.close();
  // }

  trajectory_ = std::make_shared<Trajectory>(knot_distance, 0);
  trajectory_->SetSensorExtrinsics(SensorType::RadarSensor, EP_RtoI);
  trajectory_->SetSensorExtrinsics(SensorType::EventSensor, EP_EtoI);

  // twist_trajctory = std::make_shared<Twist_Trajectory>(knot_distance, 0);
  // twist_trajctory = std::allocate_shared<Twist_Trajectory>(
  //   Eigen::aligned_allocator<Twist_Trajectory>(), knot_distance, 0);
  // twist_trajctory = std::allocate_shared<Twist_Trajectory>
  //             (Eigen::aligned_allocator<Twist_Trajectory>(), knot_distance, 0);

  // twist_trajctory = std::make_shared<Twist_Trajectory>(knot_distance, 0);
  // twist_trajctory = std::allocate_shared<Twist_Trajectory>
  //                 (Eigen::aligned_allocator<Twist_Trajectory>(), knot_distance, 0);

  try {
      twist_trajctory = std::allocate_shared<Twist_Trajectory>(
          Eigen::aligned_allocator<Twist_Trajectory>(), 
          knot_distance, 
          0
      );
  } catch (const std::bad_alloc& e) {
      LOG(ERROR) << "Memory allocation failed: " << e.what();
      // 打印当前内存状态
      struct sysinfo mem_info;
      sysinfo(&mem_info);
      LOG(ERROR) << "Before Free memory: " << mem_info.freeram / 1024 / 1024 << " MB";
  }

  twist_trajctory->SetSensorExtrinsics(SensorType::RadarSensor, EP_RtoI);
  twist_trajctory->SetSensorExtrinsics(SensorType::EventSensor, EP_EtoI);
  // HAO TODO: 
  // linear_spline = std::make_shared<RdSpline>(knot_distance, 0);
  // angular_spline = std::make_shared<RdSpline>(knot_distance, 0);

  local_trajectory_ = std::make_shared<Trajectory>(trajectory_->getDt(), 0);
  local_trajectory_->SetSensorExtrinsics(SensorType::RadarSensor, trajectory_->GetSensorEP(RadarSensor));
  local_trajectory_->SetSensorExtrinsics(SensorType::EventSensor, trajectory_->GetSensorEP(EventSensor));



  if (node["bias"]) {

    // Load parameters from YAML into para_bw_vec and para_bv_vec
    std::vector<double> omega_bias_vec = node["bias"]["omega"].as<std::vector<double>>();
    std::vector<double> vel_bias_vec = node["bias"]["vel"].as<std::vector<double>>();

    all_twist_bias_[0].omega_bias = Eigen::Map<Eigen::Vector3d>(omega_bias_vec.data());
    all_twist_bias_[0].vel_bias = Eigen::Map<Eigen::Vector3d>(vel_bias_vec.data());

    // all_twist_bias_[0].omega_bias = node["bias"]["omega"].as<std::vector<double>>();
    // all_twist_bias_[0].vel_bias = node["bias"]["vel"].as<std::vector<double>>();
    
  } else {
    // Assign {0,0,0} if no bias exists in the YAML
    all_twist_bias_[0].omega_bias << 0, 0, 0;
    all_twist_bias_[0].vel_bias << 0, 0, 0;
    LOG(ERROR) << "Bias Parameters Not Exist!" << std::endl;
  }

  // Print to console and log the information
  std::cout << "para_bw_vec = " << all_twist_bias_[0].omega_bias(0) 
            << ", " << all_twist_bias_[0].omega_bias(1) << ", " << all_twist_bias_[0].omega_bias(2) << "]";   
  std::cout << "para_bv_vec = " << all_twist_bias_[0].vel_bias(0) 
            << ", " << all_twist_bias_[0].vel_bias(1) << ", " << all_twist_bias_[0].vel_bias(2) << "]";   
  LOG(INFO) << "para_bw_vec = " << all_twist_bias_[0].omega_bias(0) 
            << ", " << all_twist_bias_[0].omega_bias(1) << ", " << all_twist_bias_[0].omega_bias(2) << "]";  
  LOG(INFO) << "para_bv_vec = " << all_twist_bias_[0].vel_bias(0) 
            << ", " << all_twist_bias_[0].vel_bias(1) << ", " << all_twist_bias_[0].vel_bias(2) << "]"; 

  if(!CreateCacheFolder(config_path))
  {
    std::cout << "\t Create Cache Folder Failed!" << std::endl;
    LOG(ERROR) << "\t Create Cache Folder Failed!" << "\n";
  }

  std::cout << std::fixed << std::setprecision(4);
  LOG(INFO) << std::fixed << std::setprecision(4);

  std::cout << YELLOW << "Time Offset init: \n" << RESET;
  std::cout << "\t RADARSensor time offset init: "
            << trajectory_->GetSensorEP(RadarSensor).t_offset_ns << " [ns]\n";
  std::cout << "\t EventSensor time offset init: "
            << trajectory_->GetSensorEP(EventSensor).t_offset_ns << " [ns]\n";

  LOG(INFO) << "\t RADARSensor time offset init: "
            << trajectory_->GetSensorEP(RadarSensor).t_offset_ns << " [ns]";
  LOG(INFO) << "\t EventSensor time offset init: "
            << trajectory_->GetSensorEP(EventSensor).t_offset_ns << " [ns]";

  std::cout << YELLOW << "Extrinsic Param: \n" << RESET;
  std::cout << "\t RADARSensor Extrinsic Param: "
            << trajectory_->GetSensorEP(RadarSensor).se3.matrix() << "\n";
  std::cout << "\t EventSensor Extrinsic Param: "
            << trajectory_->GetSensorEP(EventSensor).se3.matrix() << "\n";

  LOG(INFO) << "\t RADARSensor Extrinsic Param: "
            << trajectory_->GetSensorEP(RadarSensor).se3.matrix() << "\n";
  LOG(INFO) << "\t EventSensor Extrinsic Param: "
            << trajectory_->GetSensorEP(EventSensor).se3.matrix() << "\n";
}

bool CreateCacheFolder(const std::string& config_path) {
  boost::filesystem::path path_cfg(config_path);
  std::string cache_path_parent_ = path_cfg.parent_path().string();
  std::string cache_path_ = cache_path_parent_ + "/data/re";
  boost::filesystem::create_directory(cache_path_parent_ + "/data/");
  return true;
}

bool LocatedInFirstSegment(double cur_t) {
  size_t knot_idx = trajectory_->GetCtrlIndex(cur_t * S_TO_NS);

  LOG(ERROR) << " cur index = " << knot_idx << std::endl;

  if (knot_idx < SplineOrder)
    return true;
  else
    return false;
}

bool LocatedLocalInFirstSegment(double cur_t) {
  size_t knot_idx = local_trajectory_->GetCtrlIndex(cur_t * S_TO_NS);

  LOG(ERROR) << " cur index = " << knot_idx << std::endl;

  if (knot_idx < SplineOrder)
    return true;
  else
    return false;
}


bool LocatedInFirstSegment2(double cur_t) {
  size_t knot_idx = twist_trajctory->GetCtrlIndex(cur_t * S_TO_NS);

  LOG(ERROR) << " cur index = " << knot_idx << std::endl;

  if (knot_idx < SplineOrder)
    return true;
  else
    return false;
}

Eigen::Quaterniond GetGlobalFrame() {
  Eigen::Vector3d gravity_;
  Eigen::Vector3d z_axis = gravity_ / gravity_.norm();
  Eigen::Vector3d e_1(1, 0, 0);
  Eigen::Vector3d x_axis = e_1 - z_axis * z_axis.transpose() * e_1;
  x_axis = x_axis / x_axis.norm();
  Eigen::Matrix<double, 3, 1> y_axis =
      Eigen::SkewSymmetric<double>(z_axis) * x_axis;

  Eigen::Matrix<double, 3, 3> Rot;
  Rot.block<3, 1>(0, 0) = x_axis;
  Rot.block<3, 1>(0, 1) = y_axis;
  Rot.block<3, 1>(0, 2) = z_axis;

  Eigen::Matrix3d R_Map_To_G = Rot.inverse();
  Eigen::Quaterniond q_MtoG(R_Map_To_G);
  return q_MtoG;
}

void PublishTF(ros::Time tf_timestamp, Eigen::Quaterniond quat, Eigen::Vector3d pos,
                std::string from_frame, std::string to_frame) {
  static tf::TransformBroadcaster tbr;
  tf::Transform transform;
  transform.setOrigin(tf::Vector3(pos[0], pos[1], pos[2]));
  tf::Quaternion tf_q(quat.x(), quat.y(), quat.z(), quat.w());
  transform.setRotation(tf_q);

  LOG(ERROR) << "Check TF " << std::setprecision(18) << tf_timestamp.toSec() << std::endl;
  tbr.sendTransform(tf::StampedTransform(transform, tf_timestamp,
                                          to_frame, from_frame));
}

void assign_values(double* dest, const YAML::Node& node) {
    // 假设 node 是一个包含 3 个元素的数组
    for (size_t i = 0; i < node.size(); ++i) {
        dest[i] = node[i].as<double>(); // 将 YAML 数组的值转换为 double 并赋值
    }
}

void Save_Result(double timestamp, SE3d pose, TwistBias bias, Eigen::Vector3d linear_velocity, Eigen::Vector3d angular_velocity)
{
  output_file.open(output_path, std::ios::out | std::ios::app);

  // 提取 SE3d 中的数据：旋转矩阵（R）和平移向量（t）
  Eigen::Matrix3d R = pose.rotationMatrix(); // 旋转矩阵
  Eigen::Quaterniond q(R);
  Eigen::Vector3d p = pose.translation();     // 平移向量

  // 确保四元数是单位四元数
  q.normalize();

  output_file << std::setprecision(18) << timestamp << ", ";  // 设置高精度输出时间戳
  output_file << p(0) << ", " << p(1) << ", " << p(2) << ", ";  // 平移向量
  output_file << q.x() << ", " << q.y() << ", " << q.z() << ", " << q.w() << ", ";  // 四元数
  output_file << bias.omega_bias(0) << ", " << bias.omega_bias(1) << ", " << bias.omega_bias(2) << ", ";  // 角速度偏置
  output_file << bias.vel_bias(0) << ", " << bias.vel_bias(1) << ", " << bias.vel_bias(2) << ", ";
  output_file << linear_velocity(0) << ", " << linear_velocity(1) << ", " << linear_velocity(2) << ", ";
  output_file << angular_velocity(0) << ", " << angular_velocity(1) << ", " << angular_velocity(2) << std::endl;  // 速度偏置

  output_file.close();

}

void PublishTrajectoryAndMap(Trajectory::Ptr trajectory, double min_time,
                               double max_time, double dt)
{
  LOG(ERROR) << "PublishTrajectoryAndMap " << std::endl;

  // Set Current System Time for Visualization
  // ros::Time time_now = ros::Time::now();

  ros::Time time_now;
  time_now.fromSec(max_time + trajectory->GetDataStartTime());

  //ros::Time::now();  // HAO TODO：
  ros::Time t_temp; 
  // if (min_time < trajectory->minTime(IMUSensor))
  //   min_time = trajectory->minTime(IMUSensor);
  // if (max_time > trajectory->maxTime(IMUSensor))
  //   max_time = trajectory->maxTime(IMUSensor);

  // Check start and end time
  if (min_time < 0) min_time = 0;
  if (max_time <= min_time) return;

  LOG(ERROR) << "Publish Radar Pose " << std::endl;
  geometry_msgs::PoseStamped cur_p_in_G;
  if (pub_spline_trajectory_.getNumSubscribers() != 0) {
    std::vector<geometry_msgs::PoseStamped> poses_geo;
    for (double t = min_time; t < max_time; t += dt) {
      // SE3d pose = trajectory->GetIMUPose(t);
      SE3d pose = trajectory->GetRadarPose(t);
      geometry_msgs::PoseStamped poseIinG;
      poseIinG.header.stamp = t_temp.fromSec(t);
      poseIinG.header.frame_id = "map";
      tf::pointEigenToMsg(pose.translation(), poseIinG.pose.position);
      tf::quaternionEigenToMsg(pose.unit_quaternion(),
                                poseIinG.pose.orientation);
      poses_geo.push_back(poseIinG);
      cur_p_in_G = poses_geo.back();
    }

    // Publish TF 
    LOG(ERROR) << "Publish TF " << std::endl;
    {
      Eigen::Vector3d position(cur_p_in_G.pose.position.x,
                            cur_p_in_G.pose.position.y,
                            cur_p_in_G.pose.position.z);
      Eigen::Quaterniond orientation(cur_p_in_G.pose.orientation.w,
                                    cur_p_in_G.pose.orientation.x,
                                    cur_p_in_G.pose.orientation.y,
                                    cur_p_in_G.pose.orientation.z);
      orientation.normalize(); 
      PublishTF(time_now, orientation, position, "radar", "map");
    }

    nav_msgs::Path traj_path;
    traj_path.header.stamp = time_now;
    traj_path.header.frame_id = "map";
    traj_path.poses = poses_geo;
    pub_spline_trajectory_.publish(traj_path);
  }

  LOG(ERROR) << "Publish Control Pose " << std::endl;
  if (pub_spline_ctrl_.getNumSubscribers() != 0) {
    std::vector<geometry_msgs::PoseStamped> poses_ctrl;
    for (size_t i = 0; i < trajectory->numKnots(); ++i) {
      double t = min_time + i * trajectory->getDt();
      geometry_msgs::PoseStamped geo_ctrl;
      geo_ctrl.header.stamp = t_temp.fromSec(t);
      geo_ctrl.header.frame_id = "map";
      tf::pointEigenToMsg(trajectory->getKnotPos(i), geo_ctrl.pose.position);
      tf::quaternionEigenToMsg(trajectory->getKnotSO3(i).unit_quaternion(),
                                geo_ctrl.pose.orientation);
      poses_ctrl.push_back(geo_ctrl);
    }

    nav_msgs::Path traj_ctrl;
    traj_ctrl.header.stamp = time_now;
    traj_ctrl.header.frame_id = "map";
    traj_ctrl.poses = poses_ctrl;
    pub_spline_ctrl_.publish(traj_ctrl);
  }

  LOG(ERROR) << "Publish Control Pose 2 " << std::endl;
  if (pub_spline_ctrl_cloud_.getNumSubscribers() != 0) {
    VPointCloud ctrl_cloud;
    for (size_t i = 0; i < trajectory->numKnots(); ++i) {
      const Eigen::Vector3d &p = trajectory->getKnotPos(i);
      VPoint ctrl_p;
      ctrl_p.x = p[0];
      ctrl_p.y = p[1];
      ctrl_p.z = p[2];
      ctrl_p.intensity = 100;

      ctrl_cloud.push_back(ctrl_p);
    }

    sensor_msgs::PointCloud2 cloud_msg;
    pcl::toROSMsg(ctrl_cloud, cloud_msg);
    cloud_msg.header.stamp = time_now; //ros::Time::now();
    cloud_msg.header.frame_id = "map";
    pub_spline_ctrl_cloud_.publish(cloud_msg);
  }

  LOG(ERROR) << "Publish Active Control Pose " << std::endl;
  if (pub_spline_active_ctrl_cloud_.getNumSubscribers() != 0) {
    size_t opt_idx =
        trajectory->GetCtrlIndex(trajectory->opt_min_lio_time_tmp * S_TO_NS);

    VPointCloud active_ctrl_cloud;
    for (size_t i = opt_idx; i < trajectory->numKnots(); ++i) {
      const Eigen::Vector3d &p = trajectory->getKnotPos(i);
      VPoint ctrl_p;
      ctrl_p.x = p[0];
      ctrl_p.y = p[1];
      ctrl_p.z = p[2];
      ctrl_p.intensity = 100;

      active_ctrl_cloud.push_back(ctrl_p);
    }

    sensor_msgs::PointCloud2 active_cloud_msg;
    pcl::toROSMsg(active_ctrl_cloud, active_cloud_msg);
    active_cloud_msg.header.stamp = ros::Time::now();
    active_cloud_msg.header.frame_id = "map";
    pub_spline_active_ctrl_cloud_.publish(active_cloud_msg);
  }

  // Publish Map

}

void PublishMargCtrlCloud(const VPointCloud &marg_ctrl_cloud) {
  if (pub_spline_marg_ctrl_cloud_.getNumSubscribers() != 0 &&
      marg_ctrl_cloud.size() > 0) {
    sensor_msgs::PointCloud2 cloud_msg;
    pcl::toROSMsg(marg_ctrl_cloud, cloud_msg);
    cloud_msg.header.stamp = ros::Time::now();
    cloud_msg.header.frame_id = "map";
    pub_spline_marg_ctrl_cloud_.publish(cloud_msg);
  }
}

/*
void PublishLatestBias(const Eigen::Vector3d &gyro_bias,
                        const Eigen::Vector3d &accel_bias,
                        bool is_LIO = true) {
  if (pub_latest_bias_vio_.getNumSubscribers() != 0 ||
      pub_latest_bias_lio_.getNumSubscribers() != 0) {
    sensor_msgs::Imu imu_msg;
    imu_msg.header.stamp = ros::Time::now();
    imu_msg.angular_velocity.x = gyro_bias[0];
    imu_msg.angular_velocity.y = gyro_bias[1];
    imu_msg.angular_velocity.z = gyro_bias[2];
    imu_msg.linear_acceleration.x = accel_bias[0];
    imu_msg.linear_acceleration.y = accel_bias[1];
    imu_msg.linear_acceleration.z = accel_bias[2];
    if (is_LIO)
      pub_latest_bias_lio_.publish(imu_msg);
    else
      pub_latest_bias_vio_.publish(imu_msg);
  }
}*/

};
//}// namespace twist_estimator

#endif