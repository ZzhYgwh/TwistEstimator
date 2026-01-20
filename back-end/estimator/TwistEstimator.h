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
#include <sensor_msgs/Imu.h>

#include <pcl/common/transforms.h>            //pcl::transformPointCloud
#include <pcl_conversions/pcl_conversions.h>  //pcl::fromROSMsg
#include <eigen_conversions/eigen_msg.h>

// #include "spline/rd6_spline.h"
#include <sys/sysinfo.h>

using SO3d = Sophus::SO3<double>;
using SE3d = Sophus::SE3<double>;

struct TwistBias {
  TwistBias()
      : omega_bias(Eigen::Vector3d::Zero()),
        vel_bias(Eigen::Vector3d::Zero()),
        acc_bias(Eigen::Vector3d::Zero()),
        gyr_bias(Eigen::Vector3d::Zero()),
        gravity(Eigen::Vector3d(0, 0, -9.8)) {}
  Eigen::Vector3d omega_bias;
  Eigen::Vector3d vel_bias;
  Eigen::Vector3d acc_bias;
  Eigen::Vector3d gyr_bias;
  Eigen::Vector3d gravity;
};

class TwistEstimator
{
EIGEN_MAKE_ALIGNED_OPERATOR_NEW
public:
// **** Global Variable ****
const int gyro_weight = 50;
const int velocity_weight = 50;

Trajectory::Ptr trajectory_;
TrajectoryEstimatorOptions options;
int update_every_k_knot_;

std::string output_path;
std::fstream output_file;
double output_dt;

double local_dt = 3.0;  // 定义局部时间范围

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
std::deque<sensor_msgs::Imu::Ptr> imu_buffer;
std::deque<sensor_msgs::Imu::Ptr>::iterator imu_start;
std::deque<sensor_msgs::Imu::Ptr>::iterator imu_end;

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

Eigen::Matrix3d Ri_in_world;

double last_max_time;    
double traj_max_time; 
Twist_Trajectory::Ptr twist_trajctory;

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
  std::string file;
  ros::NodeHandle private_nh_("~");
  private_nh_.param<std::string>("config_file", file, "dji.yaml");
  LOG(ERROR) << " config_file: " << file;

  ParseYamlFile(file);

  CreateEstimator();

  twist_vec_.reserve(50);
  
  if(t_add < 0)
    t_add = update_every_k_knot_ * trajectory_->getDt();

  LOG(INFO) << "Update traj every " << t_add << " second\n"
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

  q0.setIdentity();
  t0.setZero();

  Ri_in_world.setIdentity();

  LOG(INFO) << "Initialized Successfully" << std::endl;

}

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

  all_twist_bias_[cur_time] = all_twist_bias_[last_time];

  time_offset = new double(0.0);
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

// 每次估计 一定时间内的 轨迹, 轨迹会重置
Trajectory::Ptr local_trajectory_;
bool local_initial = false;

// TightEstimate
void UpdatePrior()
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
  std::map<int, double*> imu_ba_vec;
  std::map<int, double*> imu_bg_vec;
  std::map<int, double*> imu_g_vec;
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

    // 分配内存并深拷贝 vel_bias 数据
    imu_ba_vec[0] = new double[3];
    std::memcpy(imu_ba_vec[0], all_twist_bias_.at(last_time).acc_bias.data(), 3 * sizeof(double));

    imu_ba_vec[1] = new double[3];
    std::memcpy(imu_ba_vec[1], all_twist_bias_.at(cur_time).acc_bias.data(), 3 * sizeof(double));

    imu_bg_vec[0] = new double[3];
    std::memcpy(imu_bg_vec[0], all_twist_bias_.at(last_time).gyr_bias.data(), 3 * sizeof(double));

    imu_bg_vec[1] = new double[3];
    std::memcpy(imu_bg_vec[1], all_twist_bias_.at(cur_time).gyr_bias.data(), 3 * sizeof(double));

    imu_g_vec[0] = new double[3];
    std::memcpy(imu_bg_vec[0], all_twist_bias_.at(last_time).gravity.data(), 3 * sizeof(double));

    imu_g_vec[1] = new double[3];
    std::memcpy(imu_bg_vec[1], all_twist_bias_.at(cur_time).gravity.data(), 3 * sizeof(double));   

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
      }
      LOG(ERROR) << "Get enough points" << std::endl;

      // 只对随机采样的25个点进行计算
      for (int i : sample_indices) {
          Eigen::Vector3d pt = pts[i];
          float pt_doppler = dopplers[i];
          estimator->AddDopplerMeasurementAnalytic2(twist_timestamp, pt, para_bv_vec[1],
              pt_doppler, R_r_e, global_fej_state_, use_fej, 
              use_order_opti, linear_weight, linear_w_weight, true);
      }
    }
    LOG(ERROR) << "Marginasize: Add Doppler Markers " << std::endl;

    // break;

    // double* angular_bias_ = para_bw_vec[1];
    // event_flow_factors_num += it_temp_->best_inliers.size();
    LOG(ERROR) << " Add Flow Points: " << it_temp_->best_inliers.size() << std::endl;
    for(int i = 0; i < it_temp_->best_inliers.size(); i++)
    {
      Eigen::Vector3d pixel(it_temp_->best_inliers[i].x, it_temp_->best_inliers[i].y, 1.0);
      Eigen::Vector3d pixel_cord = K.inverse() * pixel;         
      // estimator->AddEventFlowMeasurementAnalytic2(twist_timestamp, pixel_cord, it_temp_->best_flow[i], it_temp_->linear_vel_vec_,
      //     q_e_r, t_e_r, para_bv_vec[1], para_bw_vec[1], time_offset, omega_weight, omega_w_weight, false);
      estimator->AddEventFlowMeasurementAnalytic3(twist_timestamp, pixel_cord, 
      // it_temp_->best_flow[i], 
      it_temp_->normal_flows[i], it_temp_->normal_norms[i], it_temp_->linear_vel_vec_,
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

void Estimate()
{
  LOG(ERROR) << "  ----------------- Estimate -------------------- " << std::endl;
  std::chrono::time_point<std::chrono::high_resolution_clock> start_time = std::chrono::high_resolution_clock::now();

  if(!initial_done)
    return ;

  if(twist2_vec_.empty())
    return;

  last_max_time = twist_trajctory->maxTime(RadarSensor);
  traj_max_time = last_max_time + t_add; // 单位: 秒
  if(twist2_vec_.back().timestamp < traj_max_time)
    return;

  auto it = std::find_if(twist2_vec_.rbegin(), twist2_vec_.rend(), 
                      [this](const TwistData2& data) {
                          return data.timestamp < traj_max_time;
                      });

  Eigen::Vector3d last_linear_knot;
  Eigen::Vector3d last_angular_knot;

  if(twist_trajctory->getKnotSize() < 1)  // 为空
  {
    last_linear_knot.setZero();
    last_angular_knot.setZero();
  }
  else 
  {
    auto [lin_knot, ang_knot] = twist_trajctory->getLastKnots();
    last_linear_knot = lin_knot;
    last_angular_knot = ang_knot;
  }
  
  size_t knot_num = twist_trajctory->getKnotSize();
  // LOG(ERROR) << "knot_num = " << knot_num << std::endl;
  if(it == twist2_vec_.rend())  // 没有合理的数据,直接返回,这种一般是存在缺失值
  {
    twist_trajctory->extendKnotsTo(traj_max_time * S_TO_NS, last_linear_knot, last_angular_knot); // 扩展到最新的数据
    twist_trajctory->SetActiveTime(traj_max_time);
    twist_trajctory->SetForcedFixedTime(traj_max_time);

    // 清空边缘化信息
    ClearPrior();

    return;
  }

  it_base2 = ((it == twist2_vec_.rend())? twist2_vec_.begin() : it.base() - 1); // it 的正向跌代器

  assert(it->timestamp == it_base2->timestamp && "Not Same Place");
  // 确定终点
  cur_time = it_base2->timestamp;
  all_twist_bias_[cur_time] = all_twist_bias_[last_time];
  double update_time = cur_time;  // 注意是相对时间的 秒
  
  twist_trajctory->extendKnotsTo(traj_max_time * S_TO_NS, last_linear_knot, last_angular_knot); // 扩展到最新的数据

  InitPrior();

  std::map<int, double*> para_bw_vec;
  std::map<int, double*> para_bv_vec;
  std::map<int, double*> imu_ba_vec;
  std::map<int, double*> imu_bg_vec;
  std::map<int, double*> imu_g_vec;
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

    // 分配内存并深拷贝 vel_bias 数据
    imu_ba_vec[0] = new double[3];
    std::memcpy(imu_ba_vec[0], all_twist_bias_.at(last_time).acc_bias.data(), 3 * sizeof(double));

    imu_ba_vec[1] = new double[3];
    std::memcpy(imu_ba_vec[1], all_twist_bias_.at(cur_time).acc_bias.data(), 3 * sizeof(double));

    imu_bg_vec[0] = new double[3];
    std::memcpy(imu_bg_vec[0], all_twist_bias_.at(last_time).gyr_bias.data(), 3 * sizeof(double));

    imu_bg_vec[1] = new double[3];
    std::memcpy(imu_bg_vec[1], all_twist_bias_.at(cur_time).gyr_bias.data(), 3 * sizeof(double));

    imu_g_vec[0] = new double[3];
    std::memcpy(imu_bg_vec[0], all_twist_bias_.at(last_time).gravity.data(), 3 * sizeof(double));

    imu_g_vec[1] = new double[3];
    std::memcpy(imu_bg_vec[1], all_twist_bias_.at(cur_time).gravity.data(), 3 * sizeof(double));   

  }
  std::chrono::time_point<std::chrono::high_resolution_clock> time1 = std::chrono::high_resolution_clock::now();

  // [step5] 创建优化器
  TrajectoryEstimatorOptions option;
  option.lock_EPs.at(EventSensor).Unlock();
  option.is_marg_state = false;
  option.show_residual_summary = true;
  TrajectoryEstimator2::Ptr estimator(new TrajectoryEstimator2(twist_trajctory, option, "Update Traj"));

  // decide prior index
  double opt_min_time = twist2_vec_.begin()->timestamp;
  int opt_idx = twist_trajctory->GetCtrlIndex(opt_min_time * S_TO_NS);
  option.ctrl_to_be_opt_now = opt_idx;
  double opt_max_time = it_base2->timestamp;
  int scan_idx = twist_trajctory->GetCtrlIndex(opt_max_time * S_TO_NS);
  option.ctrl_to_be_opt_later = std::max(scan_idx, twist_trajctory->Get_N());

  size_t knots_num = twist_trajctory->getKnotSize();

  estimator->SetFixedIndex(std::max(opt_idx - 1, 0));

  // [step5] 因子图优化
  /// 因子图优化
  std::chrono::time_point<std::chrono::high_resolution_clock> time3;
  std::chrono::time_point<std::chrono::high_resolution_clock> time4;
  std::chrono::time_point<std::chrono::high_resolution_clock> time5;
  long int doppler_factors_num = 0;
  long int event_flow_factors_num = 0;

  // 雷达线速度必须存在
  time5 = std::chrono::high_resolution_clock::now();

  {
    // [step5-1] 优化问题建模
    {
      time3 = std::chrono::high_resolution_clock::now();

      if (marg_info) {
        estimator->AddMarginalizationFactor(marg_info,
                                            marg_parameter_blocks);
        // LOG(ERROR) << " Add Marginalize " << std::endl;
      }
      
      time4 = std::chrono::high_resolution_clock::now();
      // 根据量测构建因子图优化(Loop Optimization)
      Eigen::Matrix3d R_r_e = twist_trajctory->GetSensorEP(EventSensor).q.toRotationMatrix();
      // LOG(ERROR) << " Twist Size = " << (it_base2 - twist2_vec_.begin() + 1) << std::endl;
      // LOG(ERROR) << " Aver Freq = " << 1.0 / (it_base2->timestamp - twist2_vec_.begin()->timestamp) / (it_base2 - twist2_vec_.begin() + 1) << " Hz" << std::endl;

      for(auto it_temp_ = twist2_vec_.begin(); it_temp_ <= it_base2; it_temp_++)
      {
        // [step5-1] 多普勒残差
        const double twist_timestamp = it_temp_->timestamp;
        Eigen::Quaterniond q_e_r = twist_trajctory->GetSensorEP(EventSensor).q;
        Eigen::Vector3d t_e_r = twist_trajctory->GetSensorEP(EventSensor).p;

        // 降采样
        // doppler 线速度估计
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
              estimator->AddDopplerMeasurementAnalytic2(twist_timestamp, pt, para_bv_vec[1],
                  pt_doppler, R_r_e, global_fej_state_, use_fej, 
                  use_order_opti, linear_weight, linear_w_weight, false);
          }
        }

        // event 角速度估计
        event_flow_factors_num += it_temp_->best_inliers.size();
        for(int i = 0; i < it_temp_->best_inliers.size(); i++)
        {
          Eigen::Vector3d pixel(it_temp_->best_inliers[i].x, it_temp_->best_inliers[i].y, 1.0);
          Eigen::Vector3d pixel_cord = K.inverse() * pixel;         
          estimator->AddEventFlowMeasurementAnalytic3(twist_timestamp, pixel_cord, // it_temp_->best_flow[i], 
              it_temp_->normal_flows[i], it_temp_->normal_norms[i], it_temp_->linear_vel_vec_,
              q_e_r, t_e_r, para_bv_vec[1], para_bw_vec[1], time_offset, global_fej_state_, use_fej, 
              use_order_opti, omega_weight, omega_w_weight, false);
        }
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
        // LOG(ERROR) << "delta_time = " << delta_time << std::endl;
        // LOG(ERROR) << "cov = " << cov << std::endl;
        Eigen::Matrix<double, 6, 1> sqrt_info;
        sqrt_info.setOnes();
        sqrt_info *=  (1. / std::sqrt(cov)); // * opt_weight_.bias_info_vec;

        // LOG(ERROR) << "100 * sqrt_info = " << 100 * sqrt_info << std::endl;

        // LOG(ERROR) << " Add Bias Factor " << std::endl;

        estimator->AddBiasFactor(para_bw_vec[0], para_bw_vec[1], 
                                  para_bv_vec[0], para_bv_vec[1], 
                                  1, 100 * sqrt_info); // 100 * sqrt_info);
        // add imu bias
        estimator->AddBiasFactor(imu_ba_vec[0], imu_ba_vec[1], 
                                  imu_bg_vec[0], imu_bg_vec[1], 
                                  1, 100 * sqrt_info); // 100 * sqrt_info);
      }

    // 雷达线速度必须存在
    time5 = std::chrono::high_resolution_clock::now();

      {
        ceres::Solver::Summary summary = estimator->Solve(50, false, 8);  // use multi-thread
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

    // radar velocity
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

    }
  }  /// 发布结果

  for(auto it_temp_ = twist2_vec_.begin() + 1; it_temp_ <= it_base2; it_temp_++)
  {
    double ref_max_time = it_temp_->timestamp;

    for(auto it_ref = (it_temp_ - 1)->timestamp; it_ref <= ref_max_time; it_ref += output_dt)
    {
      Eigen::Vector3d linear_velocity = twist_trajctory->GetLiearVelWorld(it_ref);
      Eigen::Vector3d angular_velocity = twist_trajctory->GetAngularVelWorld(it_ref); 

      if(ref_max_time == it_ref)
        LOG(INFO) << "Estimate Velocity:\n"
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
      double pose_time = it_ref + relative_start_time;

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

  // 边缘化
  std::chrono::time_point<std::chrono::high_resolution_clock> time7 = std::chrono::high_resolution_clock::now();

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
    if(use_prior)
      UpdatePrior();
  }

  // 后置IMU数据更新
  /*{
    imu_buffer.erase(imu_start, imu_end);
  }*/
  
  std::chrono::time_point<std::chrono::high_resolution_clock> time8 = std::chrono::high_resolution_clock::now();
  /// 更新下一个窗口
  {
    // 更新轨迹时间
    twist_trajctory->UpdateActiveTime(update_time);

    twist_trajctory->SetForcedFixedTime(it->timestamp);         // 毫米波雷达无畸变,只有一个更新时间

    // 偏置更新
    all_twist_bias_[last_time].omega_bias = Eigen::Vector3d(para_bw_vec[0]);
    all_twist_bias_[last_time].vel_bias = Eigen::Vector3d(para_bv_vec[0]);
    all_twist_bias_[cur_time].omega_bias = Eigen::Vector3d(para_bw_vec[1]);
    all_twist_bias_[cur_time].vel_bias = Eigen::Vector3d(para_bv_vec[1]);
    all_twist_bias_[last_time].acc_bias = Eigen::Vector3d(imu_ba_vec[0]);
    all_twist_bias_[last_time].gyr_bias = Eigen::Vector3d(imu_bg_vec[0]);
    all_twist_bias_[cur_time].acc_bias = Eigen::Vector3d(imu_ba_vec[1]);
    all_twist_bias_[cur_time].gyr_bias = Eigen::Vector3d(imu_bg_vec[1]);
    all_twist_bias_[last_time].gravity = Eigen::Vector3d(imu_g_vec[0]);
    all_twist_bias_[cur_time].gravity = Eigen::Vector3d(imu_g_vec[1]);

    last_time = cur_time;
  }
    
  std::chrono::time_point<std::chrono::high_resolution_clock> end_time = std::chrono::high_resolution_clock::now();
  {
      std::chrono::duration<double, std::milli> elapsed;
      elapsed = end_time - start_time;
      LOG(INFO) << "Total Time: " << elapsed.count() << std::endl;
      elapsed = time1 - start_time;
      LOG(INFO) << "Segment Data: " << elapsed.count() << std::endl;
      elapsed = time3 - time1;
      LOG(INFO) << "Optimize: " << elapsed.count() << std::endl;
      elapsed = time4 - time3;
      LOG(INFO) << "Construct Marginize: " << elapsed.count() << std::endl;
      elapsed = time5 - time4;
      LOG(INFO) << "Construct: " << elapsed.count() << std::endl;
      elapsed = time6 - time5;
      LOG(INFO) << "Solve: " << elapsed.count() << std::endl;
      elapsed = time7 - time6;
      LOG(INFO) << "Publish Result: " << elapsed.count() << std::endl;
      elapsed = time8 - time7;
      LOG(INFO) << "Marginize: " << elapsed.count() << std::endl;
      elapsed = end_time - time8;
      LOG(INFO) << "Update Time: " << elapsed.count() << std::endl;
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

  std::string config_path_without_name = config_path.substr(0, config_path.find_last_of('/'));

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

  output_path = config_path_without_name + "/../output/" + node["output_file"].as<std::string>();
  LOG(ERROR) << "output_path = " << output_path;
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


  LOG(ERROR) << "get bias" << std::endl;
  if (node["bias"]) {

    // Load parameters from YAML into para_bw_vec and para_bv_vec
    std::vector<double> omega_bias_vec = node["bias"]["omega"].as<std::vector<double>>();
    std::vector<double> vel_bias_vec = node["bias"]["vel"].as<std::vector<double>>();

    all_twist_bias_[0].omega_bias = Eigen::Map<Eigen::Vector3d>(omega_bias_vec.data());
    all_twist_bias_[0].vel_bias = Eigen::Map<Eigen::Vector3d>(vel_bias_vec.data());
    LOG(ERROR) << "get sensor bias" << std::endl;
    // all_twist_bias_[0].omega_bias = node["bias"]["omega"].as<std::vector<double>>();
    // all_twist_bias_[0].vel_bias = node["bias"]["vel"].as<std::vector<double>>();
    std::vector<double> acc_bias_vec = node["bias"]["acc"].as<std::vector<double>>();
    std::vector<double> gyro_bias_vec = node["bias"]["gyr"].as<std::vector<double>>();
    all_twist_bias_[0].acc_bias = Eigen::Map<Eigen::Vector3d>(acc_bias_vec.data());
    all_twist_bias_[0].gyr_bias = Eigen::Map<Eigen::Vector3d>(gyro_bias_vec.data());
    LOG(ERROR) << "get imu bias" << std::endl;
    
  } else {
    // Assign {0,0,0} if no bias exists in the YAML
    all_twist_bias_[0].omega_bias << 0, 0, 0;
    all_twist_bias_[0].vel_bias << 0, 0, 0;

    all_twist_bias_[0].acc_bias << 0, 0, 0;
    all_twist_bias_[0].gyr_bias << 0, 0, 0;

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
  LOG(INFO) << "para_bw_vec = " << all_twist_bias_[0].acc_bias(0) 
            << ", " << all_twist_bias_[0].acc_bias(1) << ", " << all_twist_bias_[0].acc_bias(2) << "]";  
  LOG(INFO) << "para_bv_vec = " << all_twist_bias_[0].gyr_bias(0) 
            << ", " << all_twist_bias_[0].gyr_bias(1) << ", " << all_twist_bias_[0].gyr_bias(2) << "]"; 
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
  LOG(INFO) << "PublishTrajectoryAndMap " << std::endl;

  ros::Time time_now;
  time_now.fromSec(max_time + trajectory->GetDataStartTime());
  ros::Time t_temp; 

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