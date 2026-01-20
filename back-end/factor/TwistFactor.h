
#pragma once

#include <ceres/ceres.h>

#include "spline/spline_segment.h"
#include "utils/parameter_struct.h"

#include "split_spline_view.h"

#include "event_flow_detector/event_flow_detector.h"

#include "DebugFile.h"

#include <iomanip>  // for std::setprecision(6)

#include <Eigen/Core>
#include <Eigen/Dense>

#include <chrono>


// 构建反对称矩阵
// 特殊给2D 像素点使用
Eigen::Matrix3d Skew(const cv::Point2d& vec) {
    Eigen::Matrix3d result;
    result << 0, -1.0, vec.y, 1.0, 0, -vec.x, -vec.y, vec.x, 0;
    return result;
}

// 给3D Vector使用
Eigen::Matrix3d Skew(const Eigen::Vector3d& vec) {
    Eigen::Matrix3d skewMat;
    skewMat <<  0,       -vec(2),  vec(1),
                vec(2),  0,       -vec(0),
               -vec(1),  vec(0),  0;
    return skewMat;
}

std::string Output(const Eigen::RowVectorXd& vec) {
    std::stringstream ss;
    ss << "[";
    for (int i = 0; i < vec.size(); ++i) {
        ss << vec(i);
        if (i < vec.size() - 1) {
            ss << ", ";  // 在元素之间加逗号
        }
    }
    ss << "]";
    return ss.str();
}

template <typename Derived>
std::string Output_M(const Eigen::MatrixBase<Derived>& mat) {
// std::string Output_M(const Eigen::MatrixXd& mat) {
    std::stringstream ss;
    ss << "[";
    for (int i = 0; i < mat.rows(); ++i) {
        ss << "[";
        for (int j = 0; j < mat.cols(); ++j) {
            ss << mat(i, j);
            if (j < mat.cols() - 1) {
                ss << ", ";  // 在元素之间加逗号
            }
        }
        ss << "]";
        if (i < mat.rows() - 1) {
            ss << ",\n ";  // 在每一行之间换行
        }
    }
    ss << "]";
    return ss.str();
}

std::string Output_double(const double& val) {
    std::stringstream ss;
    ss << "[";
    // ss << std::fixed << std::setprecision(6) << val;  // 控制精度为6位小数（根据需求调整）
    ss << val;  // 控制精度为6位小数（根据需求调整）
    ss << "]";
    return ss.str();
}


struct FEJ_STATE
{
  Eigen::Vector3d linear_velocity_;
  Eigen::Vector3d angular_velocity_;
  Eigen::Vector3d linear_bias_;
  Eigen::Vector3d angular_bias_;
};


// struct FactorOption
// {
//   bool opti_bias = false;
//   bool opti_linear = false;
//   bool opti_doppler_correct = false;
// }; // struct FactorOption


// HAO TODO: 以雷达的时间为准
class DopplerFactor : public ceres::CostFunction, SplitSpineView{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  using SO3View = So3SplineView;
  using R3View = RdSplineView;
  using SplitView = SplitSpineView;

  using Vec3d = Eigen::Matrix<double, 3, 1>;
  using Vec6d = Eigen::Matrix<double, 6, 1>;
  using Mat3d = Eigen::Matrix<double, 3, 3>;
  using SO3d = Sophus::SO3<double>;

  DopplerFactor(int64_t time_ns, const Eigen::Vector3d& pt, 
            const double& doppler, const Eigen::Matrix3d& R_e_r,
            const SplineMeta<SplineOrder>& spline_segment_meta,
            double weight, double w_weight = 0.05)// const Vec6d& info_vec)
      : time_ns_(time_ns),
        pt_(pt), doppler_(doppler),
        spline_meta_(spline_segment_meta),
        weight_(weight), w_weight_(w_weight)
        // info_vec_(info_vec) 
        {
          set_num_residuals(1);           // 定义残差值的大小(doppler速度残差)

          size_t knot_num = spline_meta_.NumParameters();

          // TODO: 需要提供采样曲线的多普勒速度和偏执
          for (size_t i = 0; i < knot_num; ++i) {             
            mutable_parameter_block_sizes()->push_back(4);    // HAO TODO:
          }
          for (size_t i = 0; i < knot_num; ++i) {
            mutable_parameter_block_sizes()->push_back(3);
          }

          mutable_parameter_block_sizes()->push_back(3);  // linear bias

          // mutable_parameter_block_sizes()->push_back(3);  // angular bias

          // mutable_parameter_block_sizes()->push_back(1);  // time_offset 雷达时间作为系统时间,不存在时间偏移
        }

   virtual bool Evaluate(double const* const* parameters, double* residuals,
                        double** jacobians) const {
    typename SO3View::JacobianStruct J_R;
    // typename SO3View::JacobianStruct J_rot_a;
    typename R3View::JacobianStruct J_v;

    LOG(ERROR) << "Evaluate DopplerFactor " << std::endl;

    // 解析状态参数 parameters
    int knot_num = this->spline_meta_.NumParameters();

    int64_t t_corrected = time_ns_; //  + (int64_t)time_offset_in_ns;


    // 估计的速度
    Eigen::Vector3d v_inG;
    SO3d S_ItoG;
    if (jacobians){
      // LOG(ERROR) << std::setprecision(20) << " t_corrected = " << t_corrected << std::endl;

      // HAO TODO:
      S_ItoG = SO3View::EvaluateRp(t_corrected, spline_meta_.segments.at(0),
                              parameters, &J_R);

      // LOG(ERROR) << "first J_R.start_idx = " <<  J_R.start_idx << std::endl;

      v_inG = R3View::velocity(t_corrected,
                              spline_meta_.segments.at(0),
                              parameters + knot_num, &J_v);
    }else{
      // gyro_ = SO3View::VelocityBody(t_corrected,
      //                               spline_meta_.segments.at(0),
      //                               parameters + R_offset[0], nullptr);

      // HAO TODO:
      S_ItoG = SO3View::EvaluateRp(t_corrected, spline_meta_.segments.at(0),
                              parameters, nullptr);
      v_inG = R3View::velocity(t_corrected,
                              spline_meta_.segments.at(0),
                              parameters + knot_num, nullptr);
    }

    // LOG(ERROR) << "Interpolation velocity = " << Output(v_inG.transpose()) << std::endl;

    // LOG(ERROR) << "Pointer to linear_bias: " << parameters[2 * knot_num] << std::endl;
    Eigen::Map<Vec3d const> linear_bias(parameters[2 * knot_num]);          // 速度偏置
    

    // residuals[0] = weight_ * (doppler_ - pt_.normalized().transpose() * S_ItoG.matrix().inverse() * (v_inG + linear_bias).eval());

    // Eigen::Vector3d corrected_velocity = v_inG + linear_bias;      // 注意这里的 linear_bias 也是定义在 
    // double doppler_actual = pt_.normalized().transpose() * S_ItoG.matrix().inverse() * corrected_velocity;
    // residuals[0] = doppler_ - doppler_actual;
    // residuals[0] *= weight_;

    // 1-10 TODO: 修正 偏置应该在各自的传感器上 linear_bias 应该在radar系上
    // Eigen::Vector3d corrected_velocity = v_inG;      // 注意这里的 linear_bias 也是定义在 
    // double doppler_actual = pt_.normalized().transpose() * (S_ItoG.matrix().inverse() * v_inG + linear_bias); // 这里的I定位为雷达系,S_ItoG.matrix().inverse() * v_inG 则是雷达系的参考速度
    // residuals[0] = doppler_ - doppler_actual;
    // residuals[0] *= weight_;

    // 1-24 check:
    // double doppler_actual = pt_.normalized().transpose() * (S_ItoG.matrix().inverse() * v_inG + linear_bias); // 这里的I定位为雷达系,S_ItoG.matrix().inverse() * v_inG 则是雷达系的参考速度
    // residuals[0] = doppler_ - doppler_actual;

    // residuals[0] = (weight_ * (doppler_ - pt_.normalized().transpose() * (S_ItoG.matrix().transpose() * v_inG + linear_bias)));    
    Eigen::Matrix3d temp = S_ItoG.matrix();
    Eigen::Map<Eigen::Matrix3d> S_GtoI_map(temp.data());
    Eigen::Matrix3d S_GtoI = S_GtoI_map.transpose();

    Eigen::Vector3d v_inI_compen = (S_GtoI * v_inG + linear_bias);

    // HAO TODO: 4-6 修改
    // Eigen::Vector3d v_inI_compen = v_inG + linear_bias;
    // LOG(ERROR) << "linear_bias = " << Output(linear_bias.transpose()) << std::endl;
    // LOG(ERROR) << "v_inG = " << Output(v_inG.transpose()) << std::endl;
    // LOG(ERROR) << "v_inI_compen = " << v_inI_compen << std::endl;
    // LOG(ERROR) << "doppler_ = " << doppler_ << std::endl;
    // LOG(ERROR) << "pt_.normalized().transpose() = " << Output(pt_.normalized().transpose()) << std::endl;
    // double temp_residuals = (- pt_.normalized().transpose() * v_inI_compen).eval();
    // double temp_residuals = (- pt_.normalized().transpose() * v_inI_compen).value();
    double temp_residuals = (doppler_ - pt_.normalized().dot(v_inI_compen));

    // LOG(ERROR) << "temp_residuals = " << temp_residuals << std::endl;
    residuals[0] = (doppler_ - pt_.normalized().dot(v_inI_compen));
    // LOG(ERROR) << "Doppler residuals = " << residuals[0] << "weight = " << weight_ << std::endl;
    residuals[0] = weight_ * residuals[0];
    // LOG(ERROR) << "weight * Doppler residuals = " << residuals[0] << std::endl;


    debug_ceres.Open();
    if(!clear_debug)
    {
      clear_debug = true;
      debug_ceres.Close();

      // debug_ceres.debug_file.open(debug_ceres.file_path, std::ios::trunc);
      // // debug_ceres.debug_file.clear();
      // debug_ceres.debug_file.close();

      debug_ceres.Open();
      // debug_ceres.debug_file << "Start to Record \n";
    }
    // debug_ceres.debug_file << std::endl;
    // debug_ceres.debug_file << "    ---------------------- Evaluate DopplerFactor ------------------    " << std::endl;
    // LOG(ERROR) << "open debug file: " << debug_ceres.file_path << std::endl;
    // debug_ceres.debug_file << "Doppler residuals = " << std::setprecision(20) << residuals[0] << std::endl;
    // LOG(ERROR) << "residuals done" << std::endl;

    // debug
    Eigen::RowVector3d pt_row_vec = pt_.transpose();

    Eigen::RowVector3d linear_bias_row_vec = linear_bias.transpose(); //.eval();

    // assert(parameters[2 * knot_num] != nullptr && "parameters[2 * knot_num] is nullptr");
    // // debug_ceres.debug_file << "Details: weight_ = " << std::setprecision(5) << weight_ 
    
    // debug_ceres.debug_file << "Doppler Details: weight_ = " <<  weight_ << std::endl;
    // debug_ceres.debug_file << "Doppler Details: w_weight_ = " <<  w_weight_ << std::endl;
    // debug_ceres.debug_file << "Doppler residuals = " << std::setprecision(20) <<  residuals[0];
    // // debug_ceres.debug_file << "\n pt_row_vec = [" <<  pt_row_vec << "] ";
    // debug_ceres.debug_file << "\n pt_row_vec = [" <<  Output(pt_row_vec) << "] ";
    // debug_ceres.debug_file << "\n pt_row_vec.norm = [" <<  Output(pt_row_vec.normalized()) << "] "
                                  // << "\nS_GtoI = [" << Output_M(S_GtoI) << "] "
                                  // << "\ndoppler_ = [" << doppler_ << "] "
                                  // << "\nS_ItoG = [" << Output_M(S_ItoG.matrix()) << "] "
                                  // << "\nv_inG = [" << Output(v_inG.transpose()) << "] " 
                                  // << "\nlinear_bias = [" << Output(linear_bias_row_vec) << "] " << std::endl;

    // 不评估雅可比就返回
    if (!jacobians) {
      LOG(ERROR) << "Doppler No Jacobians " << std::endl;
      // debug_ceres.debug_file << "No Jacobians" << std::endl;
      debug_ceres.Close();
      
      return true;
    }
    else
    {
      LOG(ERROR) << " Calculate Jacobians " << std::endl;
    }

    // debug_ceres_jacobis.Open();
    // debug_ceres_jacobis.debug_file << "Evaluate DopplerFactor Res J_R_0 J_R_1 J_R_2 J_R_3 J_p_0 J_p_1 J_p_2 J_p_3 J_linear_bias \n";
    // debug_ceres_jacobis.debug_file << residuals[0] << " ";
 
    // 位姿雅可比
    // Eigen::Matrix3d jac_lhs_R;
        // residuals[0] = 
    // (weight_ * (doppler_ - pt_.normalized().transpose() * (S_ItoG.matrix().transpose() * v_inG + linear_bias)));
    Eigen::Vector3d jac_lhs_R = Skew(v_inG).transpose() * S_ItoG.matrix() * pt_.normalized();
    // jac_lhs_R = - weight_ * pt_.normalized().transpose() * S_ItoG.matrix().inverse() *  SO3::hat(v_inG + linear_bias);

    // jac_lhs_R =  - weight_ * (v_inG + linear_bias) * pt_.normalized().transpose();

    // jac_lhs_R =  - weight_ * (pt_.normalized().transpose().cross((v_inG + linear_bias))).transpose();
    // jac_lhs_R = - weight_ * (pt_.normalized().cross((v_inG + linear_bias)));

    // 1-10 TODO: 修正
    // 修正的残差是
    // pt_.normalized().transpose() * (S_ItoG.matrix().inverse() * v_inG + linear_bias)

    // jac_lhs_R = - weight_ * (pt_.normalized() *  S_ItoG.matrix().inverse() * v_inG);
    // jac_lhs_R = (pt_.normalized().transpose() *  S_ItoG.matrix().inverse() *  Skew(v_inG)).transpose(); // weight_ 在下面乘了
  
    // 1-12 修改
    // jac_lhs_R = (Skew(pt_.normalized()).transpose() * S_ItoG.matrix().transpose() * v_inG);
    // jac_lhs_R += ((pt_.normalized().transpose() * S_ItoG.matrix().inverse() * Skew(v_inG)).transpose()).eval();
    // jac_lhs_R += ((pt_.normalized().transpose() * S_ItoG.matrix().inverse() * Skew(v_inG)).transpose());

    jac_lhs_R = (jac_lhs_R);
    // LOG(ERROR) << " jac_lhs_R = " << Output(jac_lhs_R) << std::endl;
    // debug_ceres.debug_file << "jac_lhs_R = " << Output(jac_lhs_R) << std::endl;
    // // debug_ceres.debug_file << "details = [" << jac_lhs_R.transpose() << "]\n";

    // 速度雅可比 J_v_d_
    // Eigen::Vector3d J_v_d_ = - S_ItoG.matrix().inverse() * pt_.normalized();
    // 1-10 修改
    // Eigen::Vector3d J_v_d_ = - S_ItoG.matrix().inverse() * pt_.normalized();  // use with transpose()

    // 1-24 check:


    // residual = doppler_ - pt_.normalized().transpose() * (S_ItoG.matrix().inverse() * v_inG + linear_bias)
    // - pt_.normalized().transpose() * S_ItoG.transpose() * v_inG / S_ItoG
    // pt_.normalized().transpose() * (S_ItoG.transpose() - (S_ItoG * (I + skew(theta)).transpose()) * v_inG / theta
    // - pt_.normalized().transpose() * (S_ItoG * skew(theta)).transpose() * v_inG / theta
    // - pt_.normalized().transpose() * skew(theta).transpose() * S_ItoG.transpose() * v_inG / theta
    // - (skew(theta) * pt_.normalized()).transpose() * S_ItoG.transpose() * v_inG  / theta
    // - ( - skew(pt_.normalized()) * theta).transpose() * S_ItoG.transpose() * v_inG  / theta
    // theta.transpose() * skew(pt_.normalized()).transpose() * S_ItoG.transpose() * v_inG  / theta.transpose() .transpose()
    // (skew(pt_.normalized()).transpose() * S_ItoG.transpose() * v_inG).transpose()

    // 偏置雅可比 J_v_b_  
    // Eigen::Vector3d J_v_b_ = - S_ItoG.matrix().inverse() * pt_.normalized();

    // 1-10 修改
    // Eigen::Vector3d J_v_b_ = - pt_.normalized().transpose();
    // residuals[0] = 
    // (weight_ * (doppler_ - pt_.normalized().transpose() * (S_ItoG.matrix().transpose() * v_inG + linear_bias)));
    // Eigen::Vector3d J_v_b_ = - weight_ *  S_ItoG.matrix() * pt_.normalized();
    // Eigen::Vector3d J_v_b_ = - S_ItoG.matrix().inverse() * pt_.normalized();
    // LOG(ERROR) << "SplineOrder = " << SplineOrder << std::endl;

    // LOG(ERROR) << " start to calculate Rotation control points " << std::endl;

    /// Rotation control point
    Eigen::Matrix<double, 4, 4> Jac_R;
    for (size_t i = 0; i < knot_num; i++) {

      size_t idx = i + J_R.start_idx;
      // LOG(ERROR) << " J_R.start_idx = " <<  J_R.start_idx << std::endl;
      // LOG(ERROR) << "idx = " << idx << std::endl;
      if (jacobians[idx]) {

        // for Debug
        // double* temp_test = jacobians[idx];
        // for(size_t j = 0;j < 12;j++)
        // {
        //   LOG(ERROR) << "j = " << j << std::endl;
        //   LOG(ERROR) << "jacobians[j] = " << temp_test[j] << std::endl;
        // }
      
        Eigen::Map<Eigen::Matrix<double, 1, 4, Eigen::RowMajor>> jac_kont_R(
            jacobians[idx]);
        jac_kont_R.setZero();
       
      //  jac_kont_R.setZero();

       // 在雷达多普勒中不优化旋转    
       {
        Eigen::RowVector3d jac_kont_R_pre = jac_lhs_R.transpose() * J_R.d_val_d_knot[i];

        // debug_ceres.debug_file << "J_R.d_val_d_knot[" << i << "] = [" <<  Output_M(J_R.d_val_d_knot[i]) << "]\n";

        // debug_ceres.debug_file << "J_R_pre.d_val_d_knot[" << i << "] = [" << Output(jac_kont_R_pre) << "]" << std::endl;
        jac_kont_R.block<1, 3>(0, 0) += (w_weight_ * weight_ * jac_kont_R_pre).eval();

        LOG(ERROR) << " J_R.d_val_d_knot[ " << i << "]" << std::endl;    

        Eigen::RowVectorXd jac_kont_R_copy = jac_kont_R;

        LOG(ERROR) << "J_R_" << i << " = [" << Output(jac_kont_R_copy) << std::endl;
        // // debug_ceres.debug_file << "J_R_" << i << " = [" << jac_kont_R_copy.transpose() << "]\n";
        // debug_ceres.debug_file << "J_R_" << i << " = [" << Output(jac_kont_R_copy) << "]\n";
      
        Jac_R.block<1,4>(i, 0) = jac_kont_R_copy;

        // // debug_ceres_jacobis.debug_file << jac_kont_R_copy.transpose() << " ";
        // debug_ceres_jacobis.debug_file << Output(jac_kont_R_copy) << " ";
      }
        // Eigen::RowVectorXd jac_kont_R_copy = jac_kont_R;
        // Jac_R.block<1,4>(i, 0) = jac_kont_R_copy;
      
      }
    }
    // LOG(ERROR) << "Calculate Rotation Control Jacobbi " << std::endl;

    // 检查 I:
    // debug_ceres.debug_file << "Jac_R = " << Output_M(Jac_R) << std::endl;

    // LOG(ERROR) << "Jacobi for position J_v." << std::endl;
      // for (size_t i = 0; i < knot_num; ++i) {
      //     LOG(ERROR) << "Mat3[" << i << "] =\n" << J_v.d_val_d_knot[i] << "\n\n";
      // }
    Eigen::Vector3d J_v_b_;
     
    // HAO TODO: 4-6修改
    J_v_b_ = (- S_ItoG.matrix() * pt_.normalized());
    // J_v_b_ = (- weight_ * pt_.normalized());
    // debug_ceres.debug_file << "- pt_.normalized.transpose = " << Output(- pt_.normalized().transpose()) << "\n";
    // debug_ceres.debug_file << "S_ItoG = " << Output_M(S_ItoG.matrix()) << "\n";
    // J_v_d_ += ((- pt_.normalized().transpose() * S_ItoG.matrix().inverse()).transpose()).eval();
    // LOG(ERROR) << " J_v_d_ = " << std::endl;
    // debug_ceres.debug_file << "J_v_b_ = " << Output(J_v_b_.transpose()) << "\n";

    Eigen::Matrix<double, 4, 3> Jac_p;
    /// position control point
    for (size_t i = knot_num; i < 2 * knot_num; i++) {
      size_t idx = i;   // [0, knot_num - 1]
      // LOG(ERROR) << "idx = " << idx << std::endl;
      if (jacobians[idx]) {
        Eigen::Map<Eigen::Matrix<double, 1, 3, Eigen::RowMajor>> jac_kont_p(
            jacobians[idx]);
        jac_kont_p.setZero();
        
        double d_val_d_knot = J_v.d_val_d_knot[i - knot_num];
        Eigen::VectorXd temp_cal;
        // temp_cal.setZero();
        // temp_cal = (J_v_d_ * J_v.d_val_d_knot[i - knot_num]);
        temp_cal = (d_val_d_knot * J_v_b_);

        double J_v_d_copy = J_v.d_val_d_knot[i - knot_num];
        // LOG(ERROR) << "J_v.d_val_d_knot[i - knot_num] = " << J_v_d_copy << std::endl;
        // debug_ceres.debug_file << "J_v.d_val_d_knot[" << i - knot_num << "] = " << J_v_d_copy << std::endl;


        // // debug_ceres.debug_file << "temp_cal = [" << temp_cal << "]"
        //                        << std::endl;

        // // debug_ceres.debug_file << "temp_cal = [" << J_v_d_ << "] "
        //                        << " * [" << J_v.d_val_d_knot[i - knot_num] << "]"
        //                        << " = 1: " << (J_v_d_ * J_v.d_val_d_knot[i - knot_num])
        //                        << "\n"
        //                        << " = 2: " << (J_v_d_ * J_v.d_val_d_knot[i - knot_num]).eval()
        //                        << std::endl;
        /// 1*1 1*3
        // jac_kont_p =  (J_v_d_ * J_v.d_val_d_knot[i - knot_num]).eval(); // TODO: i - knot_num 0-4 的范围
        jac_kont_p =  (weight_ * J_v.d_val_d_knot[i - knot_num] * J_v_b_ ).eval(); // TODO: i - knot_num 0-4 的范围
        // jac_kont_p = (weight_ * jac_kont_p).eval();

        Eigen::RowVectorXd jac_kont_p_copy; // = jac_kont_p;
        jac_kont_p_copy = jac_kont_p;
        // // debug_ceres.debug_file << "J_p_" << i - knot_num << " = [" << jac_kont_p_copy.transpose() << "]\n";

        // LOG(ERROR) << "J_v_d_" << i - knot_num << " = \n[" << J_v_d_ << "]\n";
        // LOG(ERROR) << "J_v_b_ " << std::endl;
        // // debug_ceres.debug_file << "J_v_d_" << i - knot_num << " = [" << J_v_d_.transpose() << "]\n";
        // debug_ceres.debug_file << "J_v_b_ = [" << Output(J_v_b_.transpose()) << "]\n";
        // // debug_ceres.debug_file << "J_v_d_" << i - knot_num << " = \n[" << J_v_d_.transpose() << "]\n";
        // LOG(ERROR) << "J_v_.d_val_d_knot_" << std::endl;
        // // debug_ceres.debug_file << "J_v_.d_val_d_knot_" << i - knot_num << " = [" << (J_v.d_val_d_knot[i - knot_num]) << "]\n";
        // debug_ceres.debug_file << "J_v_.d_val_d_knot_" << i - knot_num << " = [" << (J_v_d_copy) << "]\n";

        // Eigen::Matrix<double, 1, 3, Eigen::RowMajor> jac_kont_p_copy = jac_kont_p;
        // Eigen::VectorXd jac_kont_p_copy = jac_kont_p;
        // LOG(ERROR) << "J_p_" << i - knot_num << " = \n[" << jac_kont_p_copy << "]\n";
        // // debug_ceres.debug_file << "J_p_" << i - knot_num << " = [" << jac_kont_p_copy.transpose() << "]\n";
        // LOG(ERROR) << "J_p_" << i - knot_num << " = [" << Output(jac_kont_p_copy) << std::endl;
        // debug_ceres.debug_file << "J_p_" << i - knot_num << " = [" << Output(jac_kont_p_copy) << "]\n";

        Jac_p.block<1,3>(i - knot_num, 0) = jac_kont_p_copy;

        // // debug_ceres_jacobis.debug_file << jac_kont_p_copy.transpose() << " ";
        // LOG(ERROR) << "jac_kont_p_copy = " << " ";
        // debug_ceres_jacobis.debug_file <<  (jac_kont_p_copy) << " ";
      }
    }

    LOG(ERROR) << "Calculate Position Control Jacobbi " << std::endl;

    // // DEBUG:
    // LOG(ERROR) << "Parameters.size = " << this->parameter_block_sizes().size() << std::endl;

    // // LOG(ERROR) << "Parameters.size = " << mutable_parameter_block_sizes().size() << std::endl;
    // LOG(ERROR) << "Search for = " << 2 * knot_num << std::endl;

    // for (int i = 0; i < this->parameter_block_sizes().size(); ++i) {
    //     LOG(ERROR) << "Block size at index " << i << " = " << parameter_block_sizes()[i];
    // }
    // // 使用所有的参数块
    // for (int i = 0; i < this->parameter_block_sizes().size(); ++i) {
    //     LOG(ERROR) << "Parameter " << i << " = " << parameters[i][0];
    // }
    // LOG(ERROR) << "Parameter for last = " << parameters[8][0] 
    // << ", " << parameters[8][1] 
    // << ", " << parameters[8][2] << std::endl;

    // for (size_t i = 0; i < this->parameter_block_sizes().size(); ++i) {
    //     if (jacobians[i] != nullptr) {
    //         LOG(ERROR) << "Jacobian at index " << i << " is valid.";
    //     } else {
    //         LOG(ERROR) << "Jacobian at index " << i << " is nullptr.";
    //     }
    // }

    // [3] velocity_bias 的雅可比
    // residuals[0] = 
    // (weight_ * (doppler_ - pt_.normalized().transpose() * (S_ItoG.matrix().transpose() * v_inG + linear_bias)));    
    // Eigen::Vector3d J_v_b_ = - weight_ * pt_.normalized();
    Eigen::RowVectorXd J_velocity_bias_copy_;
    Eigen::Vector3d J_v_bias_ = - pt_.normalized();
    
    if(jacobians[2 * knot_num])
    {
      // Eigen::Map<Eigen::Matrix<double, 1, 3>> J_velocity_bias_(jacobians[2 * knot_num]);
       
      // LOG(ERROR) << "J_velocity_bias_ = " << ((jacobians[2 * knot_num] == nullptr)? "nullptr": "exist") << std::endl;
      Eigen::Map<Eigen::Matrix<double, 1, 3, Eigen::RowMajor>> J_velocity_bias_(jacobians[2 * knot_num]);
      // LOG(ERROR) << "J_velocity_bias_ = " << J_velocity_bias_ << std::endl;

      // LOG(ERROR) << "J_velocity_rot_ after = " << ((jacobians[2 * knot_num - 1] == nullptr)? "nullptr": "exist") << std::endl;

      // J_velocity_bias_.setZero();
      // LOG(ERROR) << "J_v_b_ = " << J_v_b_.transpose() << std::endl;
      // J_velocity_bias_ = (J_v_b_.transpose()).eval();
      J_velocity_bias_ = (weight_ * J_v_bias_.transpose()).eval();
      // LOG(ERROR) << "J_v_b_ = " << jacobians[2 * knot_num][0] << ", " 
      //             << jacobians[2 * knot_num][1] << ", "
      //             << jacobians[2 * knot_num][2] << ", "<< std::endl;
      // LOG(ERROR) << "J_v_b_ = " << J_velocity_bias_ << std::endl;
      // LOG(ERROR) << "J_v_b_ = " << J_velocity_bias_ << std::endl;

      // Eigen::Matrix<double, 1, 3, Eigen::RowMajor> J_velocity_bias_copy_ = J_velocity_bias_;
      J_velocity_bias_copy_ = J_velocity_bias_;
      // LOG(ERROR) << "J_v_b_" << " = [\n" << J_velocity_bias_copy_ << "]\n";
      // LOG(ERROR) << "J_velocity_bias_copy_";
      // debug_ceres.debug_file << "J_v_bias_" << Output(J_v_bias_.transpose()) << std::endl;
      // debug_ceres.debug_file << "J_velocity_bias_ = " << Output(J_velocity_bias_) << std::endl;
      // debug_ceres_jacobis.debug_file << Output(J_velocity_bias_copy_) << "]\n";
    }
    else
    {
      LOG(ERROR) << "No Calculate Velocity Bias Jacobbi " << std::endl;
    }

    // LOG(ERROR) << "No Calculate Velocity Bias Jacobbi " << std::endl;

    // [4] timeoffset 本身无偏置,因此不用加入
    // Eigen::Map<Eigen::Matrix<double, 1, 3>> J_timeoffset_(jacobians[knot_num + 2]);
    // J_timeoffset_.setZero();
    // J_timeoffset_ = J_v_b_;    

    // LOG(ERROR) << "DopplerFactor Evaluation Done" << std::endl;

    // LOG(ERROR) << "All Jacobi for Doppler factor" << std::endl;
    // debug_ceres.debug_file << "All Jacobi for Doppler factor " << std::endl;
    // debug_ceres.debug_file << "Jac_R_ = " << Output_M(Jac_R) << std::endl;
    // debug_ceres.debug_file << "J_p_ = " << Output_M(Jac_p) << std::endl;
    // debug_ceres.debug_file << "J_velocity_bias_ = " << Output(J_velocity_bias_copy_) << std::endl;

    // debug_ceres.debug_file << std::endl;
    debug_ceres.Close();
    // debug_ceres_jacobis.Close();

    // LOG(ERROR) << "print close" << std::endl;

    return true;
  }

private:
    int64_t time_ns_;
    Eigen::Vector3d pt_;
    SplineMeta<SplineOrder> spline_meta_;
    // Vec6d info_vec_;

    //TODO: gravity is not necessary
    // Eigen::Vector3d gravity;
    double doppler_;
    double weight_;

    // 某些参数的弱优化权重
    double w_weight_;

    // Eigen::Vector3d J_v_d_;
    // Eigen::Vector3d J_v_b_;
};


class EventAgularFactor : public ceres::CostFunction, SplitSpineView{
 public:
 EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  using SO3View = So3SplineView;
  using R3View = RdSplineView;
  using SplitView = SplitSpineView;

  using Vec3d = Eigen::Matrix<double, 3, 1>;
  using Vec6d = Eigen::Matrix<double, 6, 1>;
  using Mat3d = Eigen::Matrix<double, 3, 3>;
  using Quatd = Eigen::Quaterniond;
  using SO3d = Sophus::SO3<double>;

  // EventAgularFactor(int64_t time_ns, const cv::Point2d pt, const event_flow_velocity flow, 
  //           const Eigen::Vector3d doppler_velocity,
  //           // const Eigen::Quaterniond & q_e_r, const Eigen::Vector3d& t_e_r,
  //           const Eigen::Quaterniond q_e_r, const Eigen::Vector3d t_e_r,
  //           const SplineMeta<SplineOrder>& spline_segment_meta,
  //           double weight) // const Vec6d& info_vec)
  EventAgularFactor(int64_t time_ns, const Eigen::Vector3d pt, const event_flow_velocity flow, 
            const Eigen::Vector3d doppler_velocity,
            // const Eigen::Quaterniond & q_e_r, const Eigen::Vector3d& t_e_r,
            const Eigen::Quaterniond q_e_r, const Eigen::Vector3d t_e_r,
            const SplineMeta<SplineOrder>& spline_segment_meta,
            double weight, double w_weight = 0.05) // const Vec6d& info_vec)
      : time_ns_(time_ns),
        pt_(pt), doppler_velocity_(doppler_velocity),
        q_e_r(q_e_r), t_e_r(t_e_r),
        spline_meta_(spline_segment_meta),
        lock_extrincs(true),      // HAO TODO: 暂时不优化外参
        weight_(weight), w_weight_(w_weight)

    // info_vec_(info_vec) 
    {
      set_num_residuals(3);           // 定义残差值的大小(事件角速度残差)

      flow_ << flow.x, flow.y, 0;

      size_t knot_num = this->spline_meta_.NumParameters();
      // LOG(ERROR) << "knot_num = " << knot_num << std::endl;

      // TODO: 需要提供采样曲线的多普勒速度和偏执                   
      for (size_t i = 0; i < knot_num; ++i) {             
        mutable_parameter_block_sizes()->push_back(4);   // rotation
      }
      for (size_t i = 0; i < knot_num; ++i) {
        mutable_parameter_block_sizes()->push_back(3);   // position
      }
      mutable_parameter_block_sizes()->push_back(3);    // linear bias // 1 -11 修改
      mutable_parameter_block_sizes()->push_back(3);    // omega bias
      
      // if(!optition.lock_extrincs)
      LOG(ERROR) << "lock extrincs? " << ((lock_extrincs)? "True": "False");
      if(! lock_extrincs)
      {
        mutable_parameter_block_sizes()->push_back(4);  // q_e_r
        mutable_parameter_block_sizes()->push_back(3);  // t_e_r
      }
      mutable_parameter_block_sizes()->push_back(1);  // time_offset 事件时间和雷达时间的偏移
    }

   virtual bool Evaluate(double const* const* parameters, double* residuals,
                        double** jacobians) const {
      typename SO3View::JacobianStruct J_R;
      typename SO3View::JacobianStruct J_w;
      typename R3View::JacobianStruct J_v;

      LOG(ERROR) << "Evaluate EventAgularFactor " << std::endl;

      debug_ceres.Open();


      // debug_ceres.debug_file << std::endl;
      // debug_ceres.debug_file << "    ---------------------- Evaluate EventAgularFactor ------------------    " << std::endl;


      // debug_ceres.debug_file << "Evaluate EventAgularFactor \n";

      size_t knot_num = spline_meta_.NumParameters();
      // Eigen::Map<Vec3d const> omega_bias(parameters[2 * knot_num]);

      // 1-11修改
      Eigen::Map<Vec3d const> linear_bias(parameters[2 * knot_num]);
      Eigen::Map<Vec3d const> omega_bias(parameters[2 * knot_num + 1]);
      int t_offset_index = 2 * knot_num + 2;

      // Eigen::Map<Mat3d const> T_R_e_r(parameters[knot_num + 1]);
      // HAO TODO: 修改
      // int t_offset_index = 2 * knot_num + 1;
      if(! lock_extrincs)
      {
        Eigen::Map<Quatd const> T_q_e_r(parameters[2 * knot_num + 2]);
        Eigen::Map<Vec3d const> T_t_e_r(parameters[2 * knot_num + 3]);
        t_offset_index = 2 * knot_num + 4;
      }

      double time_offset_in_ns = parameters[t_offset_index][0];

      LOG(ERROR) << "Correct time: " << time_offset_in_ns << std::endl;
      LOG(ERROR) << "Max Time: " << spline_meta_.segments.at(0).MaxTimeNs() << std::endl;
      LOG(ERROR) << "Min Time: " << spline_meta_.segments.at(0).MinTimeNs() << std::endl;
      

      int64_t t_corrected = time_ns_ + (int64_t)time_offset_in_ns;

      t_corrected = std::min(t_corrected, spline_meta_.segments.at(0).MaxTimeNs() - 1);
      t_corrected = std::max(t_corrected, spline_meta_.segments.at(0).MinTimeNs() + 1);

      LOG(ERROR) << "Interplate Event Flow " << std::endl;

      // 估计的速度 
      // Eigen::Vector3d gyro_, vel_, rot_accel, accel_r3;
      Eigen::Vector3d gyro_, rot_accel, v_inG;
      SO3d S_ItoG;
      if (jacobians){
        S_ItoG = SO3View::EvaluateRp(t_corrected, spline_meta_.segments.at(0),
                        parameters, &J_R);

        gyro_ = SO3View::VelocityBody(t_corrected,
                                            spline_meta_.segments.at(0),
                                            parameters, &J_w);

        rot_accel = SO3View::accelerationBody(t_corrected,  
                                              spline_meta_.segments.at(0), 
                                              parameters);

        v_inG = R3View::velocity(t_corrected,
                                      spline_meta_.segments.at(0),
                                      parameters + knot_num, &J_v);   

        // accel_r3 = R3View::acceleration(t_corrected,
        //                             spline_meta_.segments.at(0),
        //                             parameters + knot_num, nullptr);   // &J_a
      }else{
        S_ItoG = SO3View::EvaluateRp(t_corrected, spline_meta_.segments.at(0),
                        parameters, nullptr);
                               
        gyro_ = SO3View::VelocityBody(t_corrected,
                                      spline_meta_.segments.at(0),
                                      parameters, nullptr);

        rot_accel = SO3View::accelerationBody(t_corrected,  
                                      spline_meta_.segments.at(0), 
                                      parameters);
      
        v_inG = R3View::velocity(t_corrected,
                              spline_meta_.segments.at(0),
                              parameters + knot_num, nullptr);   

        // vel_ = R3View::velocity(t_corrected,
        //                               spline_meta_.segments.at(0),
        //                               parameters + knot_num, nullptr);

        // accel_r3 = R3View::acceleration(t_corrected,
        //                             spline_meta_.segments.at(0),
        //                             parameters + knot_num, nullptr);
      }

      LOG(ERROR) << "Set Jacobi Matrix " << std::endl;

      // // Conversion vel
      // Eigen::Matrix3d R_e_r = q_e_r.toRotationMatrix();   
      // Eigen::Vector3d pre_vec = Skew(pt_) * R_e_r * ((gyro_ + angular_bias).cross(t_e_r) + doppler_velocity_); // use it add tranpose()
      // Eigen::Vector3d post_vec = (flow_ +  Skew(pt_) * R_e_r * (gyro_ + angular_bias));

      
      // Conversion vel
      Eigen::Matrix3d R_e_r = q_e_r.toRotationMatrix();   
      // Eigen::Vector3d pre_vec = Skew(pt_) * R_e_r * ((gyro_ + omega_bias).cross(t_e_r) + S_ItoG.matrix().inverse() * v_inG); // use it add tranpose()
      // Eigen::Vector3d post_vec = (flow_ + Skew(pt_) * R_e_r * (gyro_ + omega_bias));

      // 1-11 修改
      // Eigen::Vector3d pre_vec = Skew(pt_) * R_e_r.transpose() * (S_ItoG.matrix().inverse() * v_inG + linear_bias - gyro_.cross( - R_e_r.transpose() * t_e_r)); // use it add tranpose()
      // Eigen::Vector3d post_vec = flow_ + Skew(pt_) * (R_e_r.transpose() * gyro_ + omega_bias); // TODO: omega_bias is in event
      
      // T_e_r.inv = [R_e_r.transpose()   -R_e_r.transpose() * t_e_r
      //                                    1    ]

      // R_e_r

      // debug_ceres.debug_file << "R_e_r = "  << Output_M(R_e_r) << std::endl;
      // debug_ceres.debug_file << "t_e_r = "  << Output_M(t_e_r) << std::endl;

      Eigen::Matrix4d T_e_r;
      T_e_r.setZero();
      T_e_r.block<3,3>(0,0) = R_e_r;
      T_e_r.block<3,1>(0,3) = t_e_r;

      // debug_ceres.debug_file << "T_e_r.invese() = "  << Output_M(T_e_r) << std::endl;
      // debug_ceres.debug_file << "T_e_r.invese() = "  << Output_M(T_e_r.inverse()) << std::endl;

      // 1-13 修改
      Eigen::Vector3d t_r_e_large_0 =  R_e_r.transpose() * t_e_r;
      LOG(ERROR) << "Test t_r_e_large_0 = " << Output(t_r_e_large_0.transpose()) << std::endl;
      Eigen::Vector3d t_r_e = -1.0 * t_r_e_large_0;
      LOG(ERROR) << "Test t_r_e = " << std::endl;
      LOG(ERROR) << "Test t_r_e = " << Output(t_r_e.transpose()) << std::endl;

      Eigen::Matrix4d T_r_e;
      T_r_e.setZero();
      T_r_e.block<3,3>(0,0) = R_e_r.transpose().eval();
      T_r_e.block<3,1>(0,3) = t_r_e;

      // debug_ceres.debug_file << "check T_e_r.invese() = " << Output_M(T_r_e) << std::endl;

      Eigen::Vector3d pixel_cord = pt_;
      // Eigen::Vector3d pre_vec = Skew(pixel_cord) * (R_e_r.transpose() * (S_ItoG.matrix().inverse() * v_inG + linear_bias) - gyro_.cross(t_r_e));
      // Eigen::Vector3d pre_vec = Skew(pixel_cord) * (R_e_r.transpose() * (S_ItoG.matrix().transpose() * v_inG + linear_bias) + gyro_.cross(t_r_e));
      
      // 1-24 修改
      // use for transpose()
      Eigen::Vector3d pre_vec = Skew(pixel_cord) * (R_e_r.transpose() * ((S_ItoG.matrix().transpose() * v_inG + linear_bias) 
                                  + (gyro_ + R_e_r * omega_bias).cross(t_r_e)));

      // 1-24 未修改
      // Eigen::Vector3d post_vec = flow_ + Skew(pixel_cord) * (R_e_r.transpose() * gyro_ + omega_bias); // TODO: omega_bias is in event

      // 3-18 修改 法向光流的计算
      Eigen::Vector3d grad;
      grad << -1.0 / flow_(0), -1.0 / flow_(1), 0.0;
      double normal_norm = 1.0 / grad.norm();
      Eigen::Vector3d normal_flow = grad * normal_norm;

      // Eigen::Vector3d 
      double post_vec = normal_norm + normal_flow.transpose() * Skew(pixel_cord) * (R_e_r.transpose() * gyro_ + omega_bias);
      
      
      // debug_ceres.debug_file << "pre_vec = " << Output(pre_vec.transpose()) << std::endl;
      // debug_ceres.debug_file << "post_vec = " << post_vec << std::endl;

      // // debug_ceres.debug_file << "post_vec = " << Output((pre_vec * post_vec).transpose()) << std::endl;

      // // debug_ceres.debug_file << "residuals address: " << static_cast<void*>(residuals) << std::endl;

      Eigen::Vector3d residual_temp = pre_vec * post_vec;

      // 残差计算
      // residuals[0] =  weight_ * pre_vec.transpose() * post_vec;   // 1 * 3
      // residuals[0] = (weight_ * pre_vec.transpose() * post_vec);   // 1 * 3
      Eigen::Map<Eigen::Vector3d> residual(residuals);
      residual.setZero();
      // residuals[0] = (weight_ * pre_vec.transpose() * post_vec); // 1 * 3
      // residual = pre_vec * post_vec;
      residual = residual_temp;
      // debug_ceres.debug_file << "event residuals 1 = " << Output(residual.transpose()) << std::endl;
      residual = (weight_ * residual).eval();

      // // debug_ceres.debug_file << "residuals = " << std::setprecision(5) << residuals[0] << std::endl;
      // // debug_ceres.debug_file << "residuals = " << residuals[0] << std::endl;
      // // debug_ceres.debug_file << "event residuals = " << residuals[0] << std::endl;
      // debug_ceres.debug_file << "event residuals = " << Output(residual) << std::endl;

      // 技术问题,Eigen无法赋值,手动处理
      residual[0] = residual(0);
      residual[1] = residual(1);
      residual[2] = residual(2);
      
      // debug_ceres.debug_file << "event residuals origin = " << residuals[0] << ", " << residuals[1] << ", " << residuals[2] << std::endl;
      LOG(ERROR) << "residuals done" << std::endl;


      // Eigen::RowVectorXd pt_row_vec = pt_.transpose();
      // Eigen::RowVectorXd gyro_row_vec = gyro_.transpose();
      // Eigen::RowVectorXd omega_bias_row_vec = omega_bias.transpose();
      Eigen::RowVector3d omega_bias_row_vec = omega_bias.transpose();
      // LOG(ERROR) << "A" << std::endl;
      // Eigen::RowVectorXd t_e_r_row_vec = t_e_r.transpose();
      Eigen::RowVector3d t_e_r_row_vec = t_e_r.transpose();
      // LOG(ERROR) << "B" << std::endl;
      // // debug_ceres.debug_file << "Details: weight_ = " << std::setprecision(5) << weight_ 
      //                               << "\n Skew(pt_) = [" <<  Skew(pt_) << "] "
      //                               << "\n R_e_r = [" <<  R_e_r << "] "
      //                               << "\n gyro_ = [" <<  gyro_.transpose() << "] "
      //                               << "\n omega_bias = [" <<  omega_bias_row_vec << "] "
      //                               << "\n t_e_r = [" <<  t_e_r.transpose() << "] "
      //                               << "\nS_ItoG = [" << S_ItoG.matrix().inverse() << "] "
      //                               << "\nv_inG = [" << v_inG.transpose() << "] " << std::endl;
      // LOG(ERROR) << "start to output" << std::endl;
      // LOG(ERROR) << "Details: weight_ = " << std::setprecision(5) << weight_<< std::endl;
      // LOG(ERROR) << "Skew(pt_) = [" << std::setprecision(3) << Skew(pt_) << "] "<< std::endl;
      // LOG(ERROR) << "R_e_r = [" << std::setprecision(6) << R_e_r << "] "<< std::endl;
      // LOG(ERROR) << "gyro_ = [" << std::setprecision(4) << gyro_.transpose() << "] "<< std::endl;
      // LOG(ERROR) << "omega_bias = [" << std::setprecision(4) << omega_bias_row_vec << "] "<< std::endl;
      // LOG(ERROR) << "t_e_r = [" << std::setprecision(2) << t_e_r_row_vec << "] "<< std::endl;
      // LOG(ERROR) << "S_ItoG = [" << std::setprecision(3) << S_ItoG.matrix().inverse() << "] "<< std::endl;
      // LOG(ERROR) << "v_inG = [" << std::setprecision(4) << v_inG.transpose() << "] " << std::endl;

      //Eigen::Vector3d pre_vec = 
      // Skew(pixel_cord) * (R_e_r.transpose() * ((S_ItoG.matrix().transpose() * v_inG + linear_bias) + (gyro_ + R_e_r * omega_bias).cross(t_r_e)));
      // double post_vec = normal_norm + normal_flow.transpose() * Skew(pixel_cord) * (R_e_r.transpose() * gyro_ + omega_bias);

      // debug_ceres.debug_file << "Event Details: weight_ = " << weight_ 
                                    // << "\n w_weight_ = [" <<  w_weight_
                                    // << "\n pt_ = [" <<  Output(pixel_cord) << "] "
                                    // << "\n Skew(pt_) = [" <<  Output_M(Skew(pixel_cord)) << "] "
                                    // << "\n grad = [" <<  Output(grad) << "] "
                                    // << "\n normal_norm = [" << normal_norm << "] "
                                    // << "\n normal_flow = [" <<  Output(normal_flow) << "] "
                                    // << "\n R_e_r = [" <<  Output_M(R_e_r) << "] "          
                                    // // << "\n gyro_ = [" <<  gyro_.transpose() << "] "
                                    // << "\n gyro_ = [" <<  Output(gyro_.transpose()) << "] "
                                    // << "\n linear_bias = [" <<  Output(linear_bias) << "] "
                                    // << "\n omega_bias = [" <<  Output(omega_bias_row_vec) << "] "
                                    // << "\n t_e_r = [" <<  Output(t_e_r) << "] "
                                    // << "\n t_r_e = [" <<  Output(t_r_e) << "] "
                                    // << "\nS_ItoG = [" << Output_M(S_ItoG.matrix()) << "] "
                                    // << "\nS_GtoI = [" << Output_M(S_ItoG.matrix().inverse()) << "] "
                                    // << "\nv_inG = [" << Output(v_inG.transpose()) << "] " << std::endl;

      // // debug_ceres.debug_file << "pre_vec = " <<  pre_vec.transpose() << std::endl;
      // // debug_ceres.debug_file << "post_vec = " <<  post_vec.transpose() << std::endl;

      Eigen::Vector3d residuals_copy = residual;
      // LOG(ERROR) << "Angular Residual = " << residuals[0] << std::endl;
      LOG(ERROR) << "Angular Residual = " << Output(residuals_copy.transpose()) << std::endl;

      

      // 不评估雅可比就返回
      if (!jacobians) {
        LOG(ERROR) << "EventAgularFactor No Jacobi!" << std::endl;
        // debug_ceres.debug_file << "EventAgularFactor No Jacobi!" << std::endl;
        debug_ceres.Close();

        // // debug_ceres_jacobis.Close();
        return true;
      }

      // debug_ceres_jacobis.Open();
      // debug_ceres_jacobis.debug_file << "Evaluate EventAgularFactor Res J_R_0 J_R_1 J_R_2 J_R_3 J_p_0 J_p_1 J_p_2 J_p_3 J_linear_bias J_angular_bias J_t \n";
      // debug_ceres_jacobis.debug_file << Output(residuals_copy.transpose()) << std::endl;

      // TODO: 补充雅可比
      // if (jacobians) {
          // [1] gyro_ 的雅可比
          // Eigen::Matrix3d J_gyro_pre_ = (Skew(pt_) * R_e_r * Skew(t_e_r));
          // Eigen::Matrix3d J_gyro_post_ = Skew(pt_) * R_e_r;
          // 1-11 修改
          // Eigen::Matrix3d J_gyro_pre_ = (Skew(pt_) * R_e_r * Skew(- R_e_r.transpose() * t_e_r)); ///use for transpose()
          // Eigen::Matrix3d J_gyro_post_ = Skew(pt_) * R_e_r;

        // 1-13 修改
        // Eigen::Matrix3d J_gyro_pre_ = (Skew(pt_) * Skew(- R_e_r.transpose() * t_e_r)); ///use for transpose()
          //  Eigen::Matrix3d J_gyro_pre_ = (Skew(pixel_cord) * Skew(t_r_e)); ///use for transpose()

          // // 1-24 修改
          // Eigen::Matrix3d J_gyro_pre_ =  - Skew(pixel_cord) * R_e_r.transpose() * Skew(t_r_e); ///use for transpose()

          // Eigen::Matrix3d J_gyro_post_ = Skew(pixel_cord) * R_e_r.transpose();

          // 3-18 修改
          Eigen::Matrix3d J_gyro_pre_ =  - Skew(pixel_cord) * R_e_r.transpose() * Skew(t_r_e); ///use for transpose()
          Eigen::RowVector3d J_gyro_post_ = normal_flow.transpose() * Skew(pixel_cord) * R_e_r.transpose();

          LOG(ERROR) << "Calculate gyro Jacobbi " << std::endl;

          // debug_ceres.debug_file << "J_gyro_pre_ = " <<  Output_M(J_gyro_pre_) << std::endl;
          // debug_ceres.debug_file << "J_gyro_post_ = " <<  Output(J_gyro_post_) << std::endl;

          // [2] S_ItoG 的雅可比
          // Eigen::Matrix3d J_S_pre_ = - Skew(pt_) * R_e_r * S_ItoG.matrix().inverse() * SO3d::hat(v_inG);
          // 1-11 修改
          // Eigen::Matrix3d J_S_pre_ = - Skew(pixel_cord) * R_e_r.transpose() * S_ItoG.matrix().inverse() * SO3d::hat(v_inG);

          // 1-24 修改
          Eigen::Matrix3d J_S_pre_ = - Skew(pixel_cord) * R_e_r.transpose() * S_ItoG.matrix().transpose() * Skew(v_inG);

          // debug_ceres.debug_file << "J_S_pre_ = " <<  Output_M(J_S_pre_) << std::endl;

          LOG(ERROR) << "Calculate S_ItoG Jacobbi " << std::endl;

          ///[step1-2] jacobians of control points
          /// Rotation control point
          // Eigen::Matrix4d Jac_R;
          Eigen::Matrix<double, 12, 4> Jac_R;
          for (size_t i = 0; i < knot_num; i++) {
            size_t idx = i;
            if (jacobians[idx]) {
              Eigen::Map<Eigen::Matrix<double, 3, 4, Eigen::RowMajor>> jac_kont_R(
                  jacobians[idx]);
              jac_kont_R.setZero();
                    
              /// 1*3 3*3
              // jac_kont_R.block<1, 3>(0, 0) =
              //     (J_gyro_pre_.transpose() * J_w.d_val_d_knot[i] * post_vec).transpose()
              //     + pre_vec.transpose() * J_gyro_post_ * J_w.d_val_d_knot[i] + (J_S_pre_.transpose() * post_vec).transpose();
              // jac_kont_R = (weight_ * jac_kont_R).eval();

              // 1-24 修改
              // jac_kont_R.block<3, 3>(0, 0) =
              //     ((J_gyro_pre_ * J_w.d_val_d_knot[i]).transpose() * post_vec).transpose()
              //     + pre_vec.transpose() * J_gyro_post_ * J_w.d_val_d_knot[i] + ((J_S_pre_ * J_R.d_val_d_knot[i]).transpose() * post_vec).transpose();
              
              // 3-8 修改
              // jac_kont_R.block<3, 3>(0, 0) =
              //     ((J_gyro_pre_.transpose() * J_w.d_val_d_knot[i]).transpose() * post_vec) //.transpose()
              //     + pre_vec * J_gyro_post_ * J_w.d_val_d_knot[i] + ((J_S_pre_ * J_R.d_val_d_knot[i]).transpose() * post_vec).transpose(); 

              // HAO TODO: 不使用多普勒优化旋转变量
              // {
              jac_kont_R.block<3, 3>(0, 0) =
                  ((J_gyro_pre_ * J_w.d_val_d_knot[i]).transpose() * post_vec) //.transpose()
                  + pre_vec.transpose() * J_gyro_post_.transpose() * J_w.d_val_d_knot[i]
                  + ((J_S_pre_ * J_R.d_val_d_knot[i]).transpose() * post_vec); 

              jac_kont_R = (weight_ * jac_kont_R).eval();  
              // }       

              // Eigen::RowVector4d jac_kont_R_copy = jac_kont_R;
              Eigen::Matrix<double, 3, 4> jac_kont_R_copy = jac_kont_R;
              // debug_ceres.debug_file << "J_R.d_val_d_knot[" << i << "] = [" << Output_M(J_R.d_val_d_knot[i]) << "]\n";
              // debug_ceres.debug_file << "J_w.d_val_d_knot[" << i << "] = [" << Output_M(J_w.d_val_d_knot[i]) << "]\n";
              // debug_ceres.debug_file << "jac_lhs_R_" << i << " = [" << Output_M(jac_kont_R_copy) << "]\n";

              Jac_R.block<3,4>(3 * i, 0) = jac_kont_R_copy;

              // Special for Jacobias
              // debug_ceres_jacobis.debug_file << jac_kont_R_copy << std::endl;
            }
          }
          LOG(ERROR) << "Add jacobians for Rotation control point " << std::endl;



          // [3] velocity 的雅可比
          // Eigen::Map<Eigen::Matrix<double, 1, 3>> J_vel_(jacobians[knot_num]);
          // Eigen::Matrix3d J_vel_ = Skew(pt_) * R_e_r * S_ItoG.matrix().inverse();
          // 1-11 修改
          Eigen::Matrix3d J_vel_ = (Skew(pixel_cord) * R_e_r.transpose() * S_ItoG.matrix().transpose()).eval();
          // Eigen::Matrix3d J_vel_ = Eigen::Matrix3d::Zero();
          // J_vel_ = J_vel_.transpose()
          LOG(ERROR) << "Calculate velocity Jacobbi " << std::endl;
          // // debug_ceres.debug_file << "J_vel_ = [" << Output_M(J_vel_) << "]\n";

          /// position control point
          Eigen::Matrix<double, 12, 3> Jac_p;
          for (size_t i = knot_num; i <  2 * knot_num; i++) {
            size_t idx = i;
            if (jacobians[idx]) {
              // Eigen::Map<Eigen::Matrix<double, 1, 3, Eigen::RowMajor>> jac_kont_p(
              //     jacobians[idx]);
              Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor>> jac_kont_p(
                  jacobians[idx]);
              jac_kont_p.setZero();

              LOG(ERROR) << "J_v.d_val_d_knot[i - knot_num] = " << J_v.d_val_d_knot[i - knot_num] << std::endl;         
              LOG(ERROR) << "J_vel_ = " << Output_M(J_vel_) << std::endl;
              LOG(ERROR) << "post_vec = " << post_vec << std::endl;
              // LOG(ERROR) << "J_v.d_val_d_knot[i - knot_num] * J_vel_.transpose() * post_vec = " << J_v.d_val_d_knot[i - knot_num] * J_vel_.transpose() * post_vec << std::endl;

              /// 1*1 1*3
              // jac_kont_p = ((J_v.d_val_d_knot[i - knot_num] * J_vel_.transpose() * post_vec).transpose());
              // jac_kont_p = (weight_ * jac_kont_p);

              // jac_kont_p = (weight_ * (post_vec.transpose() * J_vel_ *  J_v.d_val_d_knot[i - knot_num])).eval();

              // 2025-4-3 不对线速度进行优化
              {
                // 1-24 修改
                jac_kont_p = (w_weight_ * weight_ * ((J_vel_ *  J_v.d_val_d_knot[i - knot_num]).transpose() * post_vec)).eval();
                

                Eigen::Matrix3d jac_kont_p_copy = jac_kont_p;
                // debug_ceres.debug_file << "J_v.d_val_d_knot[" << i - knot_num << "] = [" << J_v.d_val_d_knot[i - knot_num] << "]\n";
                // debug_ceres.debug_file << "J_vel_ = " << Output_M(J_vel_) << std::endl;  
                // debug_ceres.debug_file << "jac_kont_p_" << i - knot_num << " = [" << Output_M(jac_kont_p_copy) << "]\n";
                // // debug_ceres.debug_file << "jac_kont_p_" << i - knot_num << " = [" << jac_kont_p_copy.transpose() << "]\n";

                // Jac_p.block<1, 3>(i - knot_num, 0) = jac_kont_p_copy.transpose();
                // debug_ceres_jacobis.debug_file << Output_M(jac_kont_p_copy) << " ";
                Jac_p.block<3, 3>(i - knot_num, 0) = jac_kont_p_copy;
              }

              // Special for Jacobias
              // // debug_ceres_jacobis.debug_file << Output_M(jac_kont_p_copy) << " ";  


              // Eigen::Matrix3d jac_kont_p_copy = jac_kont_p;
              // Jac_p.block<3, 3>(i - knot_num, 0) = jac_kont_p_copy;      
            }
          }
          LOG(ERROR) << "Add jacobians for Position control point " << std::endl;


          // // Debug Residual & Jacobi
          // for (int i = 0; i < this->parameter_block_sizes().size(); ++i) {
          //     LOG(ERROR) << "Block size at index " << i << " = " << parameter_block_sizes()[i];
          // }
          // // 使用所有的参数块
          // for (int i = 0; i < this->parameter_block_sizes().size(); ++i) {
          //     LOG(ERROR) << "Parameter " << i << " = " << parameters[i][0];
          // }
          // LOG(ERROR) << "Parameter for last = " << parameters[8][0] 
          // << ", " << parameters[8][1] 
          // << ", " << parameters[8][2] << std::endl;

          // for (size_t i = 0; i < this->parameter_block_sizes().size(); ++i) {
          //     if (jacobians[i] != nullptr) {
          //         LOG(ERROR) << "Jacobian at index " << i << " is valid.";
          //     } else {
          //         LOG(ERROR) << "Jacobian at index " << i << " is nullptr.";
          //     }
          // }

          // 1-11 修改
          // [3] linear_bias 的雅可比          
          // Eigen::RowVector3d Jac_v_bias;
          Eigen::Matrix3d Jac_v_bias;
          if(jacobians[2 * knot_num])
          {
            // Eigen::Map<Eigen::Matrix<double, 1, 3>> J_linear_bias_(jacobians[2 * knot_num]); 
            Eigen::Map<Eigen::Matrix<double, 3, 3>> J_linear_bias_(jacobians[2 * knot_num]); 
            J_linear_bias_.setZero();

            
            // 2025-4-3 不对linear_bias进行优化
            { 
              // J_linear_bias_ = (weight_ * (Skew(pixel_cord) * R_e_r.transpose()).transpose() * post_vec).transpose().eval();
              // 3-30 修改
              J_linear_bias_ = (w_weight_ * weight_ * (Skew(pixel_cord) * R_e_r.transpose() * S_ItoG.matrix().transpose()).transpose() * post_vec).eval();
              Jac_v_bias = J_linear_bias_;

              // debug_ceres.debug_file << "J_linear_bias_ = " << Output_M(Jac_v_bias) << "]\n";
              // debug_ceres_jacobis.debug_file << Output_M(Jac_v_bias) << " ";
            }

            // Jac_v_bias = J_linear_bias_;
            // // debug_ceres.debug_file << "J_linear_bias_ = " << Output_M(Jac_v_bias) << "]\n";
            
          }

          // [4] omega_bias 的雅可比
          // Eigen::RowVector3d Jac_w_bias;
          Eigen::Matrix3d Jac_w_bias;
          if(jacobians[2 * knot_num + 1])
          {
            // Eigen::Map<Eigen::Matrix<double, 1, 3>> J_angular_bias_(jacobians[2 * knot_num + 1]);

            Eigen::Map<Eigen::Matrix<double, 3, 3>> J_angular_bias_(jacobians[2 * knot_num + 1]);
            J_angular_bias_.setZero();
            // 
            // J_angular_bias_ = pre_vec.transpose() * Skew(pixel_cord);
            
            // // 1-24 修改
            // J_angular_bias_ = (((Skew(pixel_cord) * R_e_r.transpose() * (- SO3d::hat(t_r_e) * R_e_r)).transpose() * post_vec).transpose() + pre_vec.transpose() * Skew(pixel_cord));     

            // 3-18 修改
            // J_angular_bias_ = (((Skew(pixel_cord) * R_e_r.transpose() * (- SO3d::hat(t_r_e) * R_e_r)).transpose() * post_vec).transpose() + pre_vec.transpose() * normal_flow.transpose() * Skew(pixel_cord));
            J_angular_bias_ = (((Skew(pixel_cord) * R_e_r.transpose() * (- Skew(t_r_e) * R_e_r)).transpose() * post_vec).transpose() 
                                + pre_vec * normal_flow.transpose() * Skew(pixel_cord));
                                 
            LOG(ERROR) << "J_angular_bias_ = " << Output_M(J_angular_bias_) << std::endl;

            // Eigen::VectorXd J_angular_bias_copy = J_angular_bias_;
            // Jac_w_bias = J_angular_bias_copy.transpose();


            J_angular_bias_ = (weight_ * J_angular_bias_).eval();
            Jac_w_bias = J_angular_bias_;
            // debug_ceres.debug_file << "J_angular_bias_ = " << Output_M(Jac_w_bias) << "]\n";

            // // debug_ceres.debug_file << "J_angular_bias_ = " << Output(J_angular_bias_copy) << "]\n";

            // Special for Jacobias
            // debug_ceres_jacobis.debug_file << Output_M(Jac_w_bias) << std::endl;
          }
          LOG(ERROR) << "Add jacobians for Angular Bias" << std::endl;

          // 1- 11 之前
          // // [3] angular_bias 的雅可比
          // Eigen::RowVector3d Jac_w_bias;
          // if(jacobians[2 * knot_num])
          // {
          //   Eigen::Map<Eigen::Matrix<double, 1, 3>> J_angular_bias_(jacobians[2 * knot_num]);
          //   J_angular_bias_.setZero();
          //   J_angular_bias_ = (J_gyro_pre_.transpose() * post_vec).transpose() + pre_vec.transpose() * Skew(pt_) * R_e_r;

          //   // Eigen::VectorXd J_angular_bias_copy = J_angular_bias_;
          //   // Jac_w_bias = J_angular_bias_copy.transpose();


          //   Jac_w_bias = J_angular_bias_.eval();
          //   // debug_ceres.debug_file << "J_angular_bias_ = " << Output(Jac_w_bias) << "]\n";

          //   // // debug_ceres.debug_file << "J_angular_bias_ = " << Output(J_angular_bias_copy) << "]\n";

          //   // Special for Jacobias
          //   // debug_ceres_jacobis.debug_file << Output(Jac_w_bias) << " ";
          // }
          // LOG(ERROR) << "Add jacobians for Angular Bias" << std::endl;

          // if(!optition.lock_extrincs)
          // {
              // 雅可比矩阵
              // Eigen::Matrix<double, 1, 4> J;

          // 对于旋转部分的雅可比
          /* 推导过程
          residuals[0] = (Skew(pt_) * R_e_r * (gyro_.cross(t_e_r) + vel_)).transpose() * 
                              (flow_ - Skew(pt_) * R_e_r * (gyro_ + angular_bias))
          A = (Skew(pt_) * R_e_r * (gyro_.cross(t_e_r) + vel_)).transpose()
          B = (flow_ - Skew(pt_) * R_e_r * (gyro_ + angular_bias))
          dA = Skew(pt_) * dR_theta * R_e_r * (gyro_.cross(t_e_r) + vel_)).transpose()
          dA.transpose() = Skew(pt_) * dR_theta * R_e_r * (gyro_.cross(t_e_r) + vel_))

          residuals[0] / R_e_r = ???
          */
          // if(!optition.lock_extrincs)

          LOG(ERROR) << "lock_extrincs: " << ((lock_extrincs)? "True":"False") << std::endl;
          // int t_offset_index = 2 * knot_num + 1;
          // int t_offset_index = 2 * knot_num + 2;
          if(!lock_extrincs)
          {
            if(jacobians[2 * knot_num + 2] && jacobians[2 * knot_num + 3])
            // if(jacobians[2 * knot_num + 1] && jacobians[2 * knot_num + 2])
            {
              Eigen::Map<Eigen::Matrix<double, 3, 3>> J_R(jacobians[2 * knot_num + 2]);
              Eigen::Map<Eigen::Matrix<double, 3, 3>> J_t(jacobians[2 * knot_num + 3]);
              // Eigen::MatrixXd J_pre_R = - (Skew(pixel_cord) * Skew(R_e_r * ((gyro_ + omega_bias).cross(t_e_r) + doppler_velocity_))).transpose();
              
              // Eigen::MatrixXd J_pre_t = - (Skew(pixel_cord) * R_e_r * Skew(gyro_ + omega_bias)).transpose();

              // Eigen::MatrixXd J_post_R = - (Skew(pixel_cord) * Skew(R_e_r * ((gyro_ + omega_bias).cross(t_e_r) + doppler_velocity_)));

              // 3-17 修改
              Eigen::Matrix3d J_pre_R = - Skew(pixel_cord) * R_e_r.transpose() * Skew(((S_ItoG.matrix().transpose() * v_inG + linear_bias) + (gyro_ + R_e_r * omega_bias).cross(t_r_e)))
                                        + Skew(pixel_cord) * R_e_r.transpose() * Skew(t_r_e) * R_e_r * Skew(omega_bias);
              
              Eigen::Matrix3d J_pre_t = Skew(pixel_cord) * R_e_r.transpose() * Skew(gyro_ + R_e_r * omega_bias); // .transpose()

              Eigen::RowVector3d J_post_R =  - normal_flow.transpose() * Skew(pixel_cord) * R_e_r.transpose() * Skew(gyro_);  // .transpose()

              // // debug_ceres.debug_file << "J_pre_R = " << J_pre_R << "]\n";
              // // debug_ceres.debug_file << "J_pre_t = " << J_pre_t << "]\n";
              // // debug_ceres.debug_file << "J_post_R = " << J_post_R << "]\n";

              // debug_ceres.debug_file << "J_pre_R = " << Output_M(J_pre_R) << "]\n";
              // debug_ceres.debug_file << "J_pre_t = " << Output_M(J_pre_t) << "]\n";
              // debug_ceres.debug_file << "J_post_R = " << Output_M(J_post_R) << "]\n";

              J_R = (J_pre_R.transpose() * post_vec + pre_vec * J_post_R).transpose();
              J_t = (J_pre_t.transpose() * post_vec).transpose();

              Eigen::Matrix3d J_R_copy = J_R;
              Eigen::Matrix3d J_t_copy = J_t;

              // // debug_ceres.debug_file << "J_R = " << J_R_copy << "]\n";
              // // debug_ceres.debug_file << "J_t = " << J_t_copy << "]\n";

              // debug_ceres.debug_file << "J_R = " << Output_M(J_R_copy) << "]\n";
              // debug_ceres.debug_file << "J_t = " << Output_M(J_t_copy) << "]\n";

              // 如果优化 extrincs,那么Jacobi需要重新对应
              // t_offset_index =  2 * knot_num + 4;
            }
          }

          /*
            Eigen::Matrix3d R_e_r = q_e_r.toRotationMatrix();   
            Eigen::Vector3d pre_vec = Skew(pt_) * R_e_r * ((gyro_ + rot_accel * t_e_r + angular_bias).cross(t_e_r) + S_ItoG.matrix().inverse() * v_inG); // use it add tranpose()
            Eigen::Vector3d post_vec = (flow_ +  Skew(pt_) * R_e_r * (gyro_ + rot_accel * t_e_r + angular_bias));
            double residual = pre_vec.transpose() * post_vec
          */

          // 添加时移优化
          // Eigen::RowVectorXd Jac_t;
          LOG(ERROR) << "Evaluate timeoffset" << std::endl;
          double Jac_t =0;
          if(jacobians[t_offset_index])
          {
            // 这个雅可比是标量
            // Eigen::Map<Eigen::Matrix<double, 1, 3, Eigen::RowMajor>> J_t_offset(jacobians[t_offset_index], 3);  // Assuming a 3D vector
             
            // J_t_offset.setZero();
            // Eigen::Map<Eigen::Matrix<double, 1, 1, Eigen::RowMajor>> J_t_vec(jacobians[t_offset_index]);

            // 3-18 修改
            Eigen::Map<Eigen::Matrix<double, 1, 3, Eigen::RowMajor>> J_t_vec(jacobians[t_offset_index]);
            J_t_vec.setZero();
            // Eigen::VectorXd 
            // J_t_vec = ((Skew(pt_) * R_e_r * t_e_r.cross(rot_accel)).transpose() * post_vec)
            //               + pre_vec.transpose() * (Skew(pt_) * R_e_r * rot_accel);
            // J_t_vec *= (1e-9 * weight_);

            // 1-11 修改
            // J_t_vec = ( - (Skew(pixel_cord) * R_e_r.transpose() * t_e_r.cross(rot_accel)).transpose() * post_vec)
            //               + pre_vec.transpose() * (Skew(pixel_cord) * R_e_r.transpose() * rot_accel);
            // J_t_vec = (1e-9 * weight_ * J_t_vec).eval();
            LOG(ERROR) << "J_t_vec allocate" << std::endl;
            // 1-13 修改
            /*
            Eigen::Vector3d t_r_e_temp = R_e_r.transpose() * t_e_r;
            Eigen::Vector3d t_r_e = -1.0 * t_r_e_temp;
            LOG(ERROR) << "t_r_e = " << t_r_e << std::endl;
            Eigen::Vector3d J_pre_d_t = Skew(pixel_cord) * Skew(t_r_e) * rot_accel; 
            LOG(ERROR) << "J_pre_d_t = " << J_pre_d_t << std::endl;
            Eigen::Vector3d J_post_d_t = Skew(pixel_cord) * R_e_r.transpose() * rot_accel;
            LOG(ERROR) << "J_post_d_t = " << J_post_d_t << std::endl; 
            LOG(ERROR) << "J_pre_d_t = " << J_pre_d_t << std::endl;
            LOG(ERROR) << "J_post_d_t = " << J_post_d_t << std::endl;
            Eigen::Matrix<double, 1, 1> J_t_temp = J_pre_d_t.transpose() * post_vec
                          + pre_vec.transpose() * J_post_d_t;
                          // J_t_vec = 
            J_t_vec = 1e-9 * weight_ * J_t_temp;
            J_t_vec = J_t_vec.eval();
            */


            // 1-24 修改
            // Eigen::Vector3d pre_vec = Skew(pixel_cord) * (R_e_r.transpose() * ((S_ItoG.matrix().transpose() * v_inG + linear_bias) + (gyro_ + rot_accel * dt + R_e_r * omega_bias).cross(t_r_e))); 
            // Eigen::Vector3d post_vec = flow_ + Skew(pixel_cord) * (R_e_r.transpose() * (gyro_ + rot_accel * dt + omega_bias)); // TODO: omega_bias is in event

            Eigen::Vector3d t_r_e_temp = R_e_r.transpose() * t_e_r;
            Eigen::Vector3d t_r_e = -1.0 * t_r_e_temp;
            LOG(ERROR) << "t_r_e = " << Output(t_r_e.transpose()) << std::endl;
            // Eigen::Vector3d J_pre_d_t = - 1.0 * Skew(pixel_cord) * R_e_r.transpose() * Skew(t_r_e) * rot_accel; 
            // LOG(ERROR) << "J_pre_d_t = " << J_pre_d_t << std::endl;
            // Eigen::Vector3d J_post_d_t = Skew(pixel_cord) * R_e_r.transpose() * rot_accel;

            // 3-18 修改     
            Eigen::Vector3d J_pre_d_t = - Skew(pixel_cord) * R_e_r.transpose() * Skew(t_r_e) * rot_accel; 
            // LOG(ERROR) << "J_pre_d_t = " << J_pre_d_t << std::endl;
            double J_post_d_t = normal_flow.transpose() * Skew(pixel_cord) * R_e_r.transpose() * rot_accel;
            // LOG(ERROR) << "J_post_d_t = " << J_post_d_t << std::endl; 

            LOG(ERROR) << "J_pre_d_t = " << Output(J_pre_d_t.transpose()) << std::endl;
            LOG(ERROR) << "J_post_d_t = " << J_post_d_t << std::endl;
            Eigen::RowVector3d J_t_temp = J_pre_d_t.transpose() * post_vec
                          + pre_vec.transpose() * J_post_d_t;
                          // J_t_vec = 
            J_t_vec = 1e-9 * weight_ * J_t_temp;
            J_t_vec = J_t_vec.eval();

           /*
            Eigen::Matrix3d R_e_r = q_e_r.toRotationMatrix();   
            Eigen::Vector3d pre_vec = Skew(pt_) * R_e_r * ((gyro_ + rot_accel * t_e_r + angular_bias).cross(t_e_r) + S_ItoG.matrix().inverse() * v_inG); // use it add tranpose()
            Eigen::Vector3d post_vec = (flow_ +  Skew(pt_) * R_e_r * (gyro_ + rot_accel * t_e_r + angular_bias));
            double residual = pre_vec.transpose() * post_vec
          */      


            // Eigen::VectorXd J_t_vec_copy = J_t_vec;
            // // debug_ceres.debug_file << "J_t_vec = " << J_t_vec_copy << "]\n";
            // Jac_t = J_t_vec_copy.transpose();

            // Jac_t = J_t_vec.transpose().eval();

            Jac_t = J_t_vec(0, 0);

            // debug_ceres_jacobis.debug_file << Jac_t << "]\n";

            // double J_t_offset = J_t_vec.eval();
            // J_t_offset *= 1e-9 * weight_;
            // jacobians[t_offset_index][0] = J_t_offset;
          }

          LOG(ERROR) << "Add jacobians for J_t_offset" << std::endl;

      // }

      LOG(ERROR) << "All Jacobi for Event factor" << std::endl;
      // debug_ceres.debug_file << "All Jacobi for Event factor" << std::endl;
      // debug_ceres.debug_file << "Jac_R_ = " << Output_M(Jac_R) << std::endl;
      // debug_ceres.debug_file << "Jac_p_ = " << Output_M(Jac_p) << std::endl;
      // debug_ceres.debug_file << "Jac_linear_bias_ = " << Output_M(Jac_v_bias) << std::endl;
      // debug_ceres.debug_file << "Jac_angular_bias_ = " << Output_M(Jac_w_bias) << std::endl;
      // debug_ceres.debug_file << "Jac_t = " << Jac_t << std::endl;
      // // debug_ceres.debug_file << "Jac_t = " << Output(Jac_t) << std::endl;
      // // debug_ceres.debug_file << "Jac_t = " << Output_double(Jac_t) << std::endl;

      // // debug_ceres.debug_file << std::endl;
      debug_ceres.Close();
      // debug_ceres_jacobis.Close();

      LOG(ERROR) << "All Jacobi for Event factor Done" << std::endl;

      return true;
    }

private:
    int64_t time_ns_;
    // cv::Point2d pt_;
    Eigen::Vector3d pt_;
    Eigen::Vector3d flow_;
    Eigen::Vector3d doppler_velocity_;
    Eigen::Quaterniond q_e_r;
    Eigen::Vector3d t_e_r;
    SplineMeta<SplineOrder> spline_meta_;
    // Vec6d info_vec_;
    Eigen::Vector3d angular_bias;
    Eigen::Vector3d gravity;
    bool lock_extrincs;
    double weight_;
    // 某些参数的弱优化权重
    double w_weight_;
};


// bias_gyr_i, bias_gyr_j, bias_acc_i, bias_acc_j
class NewBiasFactor : public ceres::SizedCostFunction<6, 3, 3, 3, 3> {
 public:
 EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  NewBiasFactor(double dt, const Eigen::Matrix<double, 6, 1>& sqrt_info) {
    double sqrt_dt = std::sqrt(dt);
    sqrt_info_.setZero();
    sqrt_info_.diagonal() = sqrt_info / sqrt_dt;
  }
  virtual bool Evaluate(double const* const* parameters, double* residuals,
                        double** jacobians) const {
    using Vec3d = Eigen::Matrix<double, 3, 1>;
    using Vec6d = Eigen::Matrix<double, 6, 1>;

    LOG(ERROR) << "Evaluate Bias Factor " << std::endl;

    debug_ceres.Open();
    // debug_ceres.debug_file << std::endl;
    // debug_ceres.debug_file << "    ---------------------- Evaluate NewBiasFactor ------------------    " << std::endl;


    if (parameters[0] == nullptr || parameters[1] == nullptr || parameters[2] == nullptr || parameters[3] == nullptr) {
        LOG(ERROR) << "Parameter pointer is null!";
        return false;  // 或者进行适当的错误处理
    }


    // // Check the sizes match.
    // const std::vector<int>& parameter_block_sizes_temp =
    //       this->parameter_block_sizes();
    // LOG(ERROR) << "parameters need have " << (parameter_block_sizes_temp).size() <<std::endl;
    // for(auto& param_block_size: parameter_block_sizes_temp)
    // {
    //   LOG(ERROR) << "parameter size = " << param_block_size <<std::endl;
    // }

    // if(parameters == nullptr)
    // {
    //   LOG(ERROR) << "parameters not exist " << std::endl;
    //   return false;
    // }
    // else
    // {
    //   LOG(ERROR) << "parameters exist " << std::endl;
    // }

    // if(parameters[0] != nullptr)
    //   LOG(ERROR) << " parameters[0] exist " << std::endl;
    // else
    //   LOG(ERROR) << " parameters[0] not exist " << std::endl;

    Eigen::Map<Vec3d const> bias_w_i(parameters[0]);
    Eigen::Map<Vec3d const> bias_w_j(parameters[1]);
    Eigen::Map<Vec3d const> bias_v_i(parameters[2]);
    Eigen::Map<Vec3d const> bias_v_j(parameters[3]);

    // LOG(ERROR) << "Get All Bias Parameters " << std::endl;

    // LOG(ERROR) << "bias_w_i: " << Output(bias_w_i.transpose());
    // LOG(ERROR) << "bias_w_j: " << Output(bias_w_j.transpose());
    // LOG(ERROR) << "bias_v_i: " << Output(bias_v_i.transpose());
    // LOG(ERROR) << "bias_v_j: " << Output(bias_v_j.transpose());
    // LOG(ERROR) << "bias_w_j - bias_w_i: " << Output(bias_w_j - bias_w_i);
    // LOG(ERROR) << "bias_v_j - bias_v_i: " << Output(bias_v_j - bias_v_i);


    Vec6d res;
    res.block<3, 1>(0, 0) = bias_w_j - bias_w_i;
    res.block<3, 1>(3, 0) = bias_v_j - bias_v_i;

    Eigen::Map<Vec6d> residual(residuals);


    debug_ceres.Open();
    // debug_ceres_jacobis.Open();
    if(!clear_debug)
    {
      clear_debug = true;
      debug_ceres.Close();

      // debug_ceres.debug_file.open(debug_ceres.file_path, std::ios::trunc);
      // // debug_ceres.debug_file.clear();
      // debug_ceres.debug_file.close();

      debug_ceres.Open();
      // debug_ceres.debug_file << "Start to Record \n";
    }
    // debug_ceres.debug_file << "Evaluate BiasFactor \n";

    // LOG(ERROR) << "open debug file: " << debug_ceres.file_path << std::endl;

    Vec6d residual_vec = residual;
    // // debug_ceres.debug_file << "residuals = " << std::setprecision(5) << residual_vec.transpose() << std::endl;
    LOG(ERROR) << "Bias residuals = " << Output(residual_vec) << std::endl;
    LOG(ERROR) << "residuals done" << std::endl;

    Eigen::RowVectorXd bias_w_i_row_vec = bias_w_i.transpose();
    Eigen::RowVectorXd bias_w_j_row_vec = bias_w_j.transpose();
    Eigen::RowVectorXd bias_v_i_row_vec = bias_v_i.transpose();
    Eigen::RowVectorXd bias_v_j_row_vec = bias_v_j.transpose();
    // debug_ceres.debug_file << "Details: weight_ = " << std::setprecision(5)
    //                               << "\n bias_w_i = [" <<  Output(bias_w_i_row_vec) << "] "
    //                               << "\n bias_w_j = [" <<  Output(bias_w_j_row_vec) << "] "
    //                               << "\n bias_v_i = [" <<  Output(bias_v_i_row_vec) << "] "
    //                               << "\n bias_v_j = [" <<  Output(bias_v_j_row_vec) << "] "
    //                                << "\n sqrt_info_ = [" <<  sqrt_info_ << std::endl;

                                
    LOG(ERROR) << "Details: weight_ = " << std::setprecision(5)
                                  << "\n bias_w_i = [" <<  Output(bias_w_i_row_vec) << "] "
                                  << "\n bias_w_j = [" <<  Output(bias_w_j_row_vec) << "] "
                                  << "\n bias_v_i = [" <<  Output(bias_v_i_row_vec) << "] "
                                  << "\n bias_v_j = [" <<  Output(bias_v_j_row_vec) << "] "
                                   << "\n sqrt_info_ = [" <<  sqrt_info_ << std::endl;


    // LOG(ERROR) << "sqrt_info_ = \n" << Output_M(sqrt_info_) << std::endl;
    // LOG(ERROR) << "res = " << Output(res.transpose()) << std::endl;

    residual = sqrt_info_ * res;
    // + Vec6d::Constant(10); // HAO TODO: Test residual of bias if too small

    // LOG(ERROR) << "residual = " << Output(residual.transpose()) << std::endl;


    // // Debug: Residual & Jacobi
    // for (int i = 0; i < this->parameter_block_sizes().size(); ++i) {
    //     LOG(ERROR) << "Block size at index " << i << " = " << parameter_block_sizes()[i];
    // }
    // // 使用所有的参数块
    // for (int i = 0; i < this->parameter_block_sizes().size(); ++i) {
    //     LOG(ERROR) << "Parameter " << i << " = " << parameters[i][0];
    // }
    // LOG(ERROR) << "Parameter for last = " << parameters[8][0] 
    // << ", " << parameters[8][1] s
    // << ", " << parameters[8][2] << std::endl;



    if (jacobians) {

      // debug_ceres_jacobis.debug_file << "Evaluate BiasFactor \n";
      // debug_ceres_jacobis.debug_file << Output(res.transpose()) << " ";
      // debug_ceres_jacobis.debug_file << "[ ";

      // for (size_t i = 0; i < this->parameter_block_sizes().size(); ++i) {
      //     if (jacobians[i] != nullptr) {
      //         LOG(ERROR) << "Jacobian at index " << i << " is valid.";
      //     } else {
      //         LOG(ERROR) << "Jacobian at index " << i << " is nullptr.";
      //     }
      // }

      if (jacobians[0]) {
        Eigen::Map<Eigen::Matrix<double, 6, 3, Eigen::RowMajor>> jac_bg_i(
            jacobians[0]);
        jac_bg_i.setZero();
        jac_bg_i.block<3, 3>(0, 0) = - Eigen::Matrix3d::Identity();
        jac_bg_i.applyOnTheLeft(sqrt_info_);


        Eigen::MatrixXd jac_bg_i_copy = jac_bg_i;
        // debug_ceres.debug_file << "J_bg_i = " << Output_M(jac_bg_i_copy) << "]\n";

        LOG(ERROR) << "jac_bg_i_copy = " << Output_M(jac_bg_i_copy)  << std::endl;
        // debug_ceres_jacobis.debug_file << Output_M(jac_bg_i_copy) << "\n";
      }
      if (jacobians[1]) {
        Eigen::Map<Eigen::Matrix<double, 6, 3, Eigen::RowMajor>> jac_bg_j(
            jacobians[1]);
        jac_bg_j.setZero();
        jac_bg_j.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();
        jac_bg_j.applyOnTheLeft(sqrt_info_);

        Eigen::MatrixXd jac_bg_j_copy = jac_bg_j;
        // debug_ceres.debug_file << "` = " << Output_M(jac_bg_j_copy) << "]\n";

        LOG(ERROR) << "jac_bg_j_copy = " << Output_M(jac_bg_j_copy)  << std::endl;

        // debug_ceres_jacobis.debug_file << Output_M(jac_bg_j_copy) << " ";
      }
      if (jacobians[2]) {
        Eigen::Map<Eigen::Matrix<double, 6, 3, Eigen::RowMajor>> jac_ba_i(
            jacobians[2]);
        jac_ba_i.setZero();
        jac_ba_i.block<3, 3>(3, 0) = - Eigen::Matrix3d::Identity();
        jac_ba_i.applyOnTheLeft(sqrt_info_);

        Eigen::MatrixXd jac_ba_i_copy = jac_ba_i;
        // debug_ceres.debug_file << "J_ba_i = " << Output_M(jac_ba_i_copy) << "]\n";

        LOG(ERROR) << "jac_ba_i_copy = " << Output_M(jac_ba_i_copy)  << std::endl;

        // debug_ceres_jacobis.debug_file << Output_M(jac_ba_i_copy) << " ";
      }
      if (jacobians[3]) {
        Eigen::Map<Eigen::Matrix<double, 6, 3, Eigen::RowMajor>> jac_ba_j(
            jacobians[3]);
        jac_ba_j.setZero();
        jac_ba_j.block<3, 3>(3, 0) = Eigen::Matrix3d::Identity();
        jac_ba_j.applyOnTheLeft(sqrt_info_);

        Eigen::MatrixXd jac_ba_j_copy = jac_ba_j;
        // debug_ceres.debug_file << "J_ba_j = " << Output_M(jac_ba_j_copy) << "]\n";

        LOG(ERROR) << "jac_ba_j_copy = " << Output_M(jac_ba_j_copy)  << std::endl;

        // debug_ceres_jacobis.debug_file << Output_M(jac_ba_j_copy) << "]\n";
      }
    }
    else
    {
      LOG(ERROR) << "Jacobi is not exist " << std::endl; 
    }

    debug_ceres.Close();


    // debug_ceres_jacobis.Close();
    return true;
  }

 private:
  Eigen::Vector3d acc_i_, acc_j_;
  Eigen::Vector3d gyr_i_, gyr_j_;
  Eigen::Matrix<double, 6, 6> sqrt_info_;
};



class BodyLocalVelocityFactor : public ceres::CostFunction, SplitSpineView{
 public:
 EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  using SO3View = So3SplineView;
  using R3View = RdSplineView;
  using SplitView = SplitSpineView;

  using Vec3d = Eigen::Matrix<double, 3, 1>;
  using Vec6d = Eigen::Matrix<double, 6, 1>;
  using Mat3d = Eigen::Matrix<double, 3, 3>;
  using Quatd = Eigen::Quaterniond;
  using SO3d = Sophus::SO3<double>;

  BodyLocalVelocityFactor(int64_t time_ns, const Eigen::Vector3d local_vel,
            // const Eigen::Vector3d linear_bias,
            const SplineMeta<SplineOrder>& spline_segment_meta,
            double weight, double w_weight = 0.05, double R_weight = 0.05) // const Vec6d& info_vec)
      : time_ns_(time_ns),local_vel_(local_vel),
        // linear_bias_(linear_bias),
        spline_meta_(spline_segment_meta),
        weight_(weight), w_weight_(w_weight), R_weight_(R_weight)
        {
          set_num_residuals(3);           // 定义残差值的大小(事件角速度残差)

          size_t knot_num = this->spline_meta_.NumParameters();
          // LOG(ERROR) << "knot_num = " << knot_num << std::endl;

          // TODO: 需要提供采样曲线的多普勒速度和偏执                   
          for (size_t i = 0; i < knot_num; ++i) {             
            mutable_parameter_block_sizes()->push_back(4);   // rotation
          }
          for (size_t i = 0; i < knot_num; ++i) {
            mutable_parameter_block_sizes()->push_back(3);   // position
          }
          mutable_parameter_block_sizes()->push_back(3);    // linear bias // 1 -11 修改
        
        }

  virtual bool Evaluate(double const* const* parameters, double* residuals,
                          double** jacobians) const {
        typename SO3View::JacobianStruct J_R;
        typename SO3View::JacobianStruct J_w;
        typename R3View::JacobianStruct J_v;

        LOG(ERROR) << "Evaluate BodyLocalVelocityFactor " << std::endl;
        // debug_ceres.Open();
        // debug_ceres.debug_file << std::endl;
        // debug_ceres.debug_file << "    ---------------------- Evaluate BodyLocalVelocityFactor ------------------    " << std::endl;

        size_t knot_num = spline_meta_.NumParameters();
        Eigen::Map<Vec3d const> linear_bias(parameters[2 * knot_num]);

        double time_offset_in_ns = 0; // parameters[t_offset_index][0];
        int64_t t_corrected = time_ns_ + (int64_t)time_offset_in_ns;
        // t_corrected = std::min(t_corrected, spline_meta_.segments.at(0).MaxTimeNs() - 1);
        // t_corrected = std::max(t_corrected, spline_meta_.segments.at(0).MinTimeNs() + 1);

        Eigen::Vector3d gyro_, rot_accel, v_inG;
        SO3d S_ItoG;
        if (jacobians){
          S_ItoG = SO3View::EvaluateRp(t_corrected, spline_meta_.segments.at(0),
                          parameters, &J_R);

          // gyro_ = SO3View::VelocityBody(t_corrected,
          //                                     spline_meta_.segments.at(0),
          //                                     parameters, &J_w);

          // rot_accel = SO3View::accelerationBody(t_corrected,  
          //                                       spline_meta_.segments.at(0), 
          //                                       parameters);

          v_inG = R3View::velocity(t_corrected,
                                        spline_meta_.segments.at(0),
                                        parameters + knot_num, &J_v);   
        }else{
          S_ItoG = SO3View::EvaluateRp(t_corrected, spline_meta_.segments.at(0),
                          parameters, nullptr);
                                
          // gyro_ = SO3View::VelocityBody(t_corrected,
          //                               spline_meta_.segments.at(0),
          //                               parameters, nullptr);

          // rot_accel = SO3View::accelerationBody(t_corrected,  
          //                               spline_meta_.segments.at(0), 
          //                               parameters);
        
          v_inG = R3View::velocity(t_corrected,
                                spline_meta_.segments.at(0),
                                parameters + knot_num, nullptr);   
        }
        // LOG(ERROR) << "Set Jacobi Matrix " << std::endl;

        // debug_ceres.debug_file << "BodyLocalVelocity Details: weight_ = " << weight_ 
                                      // << "\n w_weight_ = [" <<  w_weight_ << "] "
                                      // << "\n S_ItoG = [" <<  Output_M(S_ItoG.matrix()) << "] "
                                      // << "\n local_vel_= [" <<  Output(local_vel_) << "] "
                                      // << "\n v_inG = [" <<  Output(v_inG.transpose()) << "] "
                                      // << "\n linear_bias = [" <<  Output(linear_bias) << "] " << std::endl;

        LOG(ERROR) << "BodyLocalVelocity Details: weight_ = " << weight_ 
                                      << "\n w_weight_ = [" <<  w_weight_ << "] "
                                      << "\n S_ItoG = [" <<  Output_M(S_ItoG.matrix()) << "] "
                                      << "\n local_vel_= [" <<  Output(local_vel_) << "] "
                                      << "\n v_inG = [" <<  Output(v_inG.transpose()) << "] "
                                      << "\n linear_bias = [" <<  Output(linear_bias) << "] " << std::endl;

        Eigen::Matrix3d R_ItoG = S_ItoG.matrix();   // 提取旋转矩阵

        Eigen::Map<Eigen::Matrix3d> R_GtoI_map(R_ItoG.data());
        Eigen::Matrix3d R_GtoI = R_GtoI_map.transpose();

        Eigen::Map<Eigen::Vector3d> residual(residuals);
        residual.setZero();
        // Eigen::Vector3d residual_temp = R_GtoI * v_inG - local_vel_ - linear_bias;
        // Eigen::Vector3d residual_temp = v_inG - S_ItoG * local_vel_ - S_ItoG * linear_bias;
                      // S_ItoG.matrix().transpose() * v_inG - local_vel_ - linear_bias;



        LOG(ERROR) << "R_GtoI = " << Output_M(R_GtoI) << std::endl;
        // HAO TODO: 定义在机体上
        Eigen::Vector3d residual_temp = v_inG - local_vel_ - linear_bias;

        residual = residual_temp;
        // residual = (w_weight_ * weight_ * residual).eval();
        residual = (w_weight_ * weight_ * residual_temp).eval();
        // debug_ceres.debug_file << "local velocity residuals = " << Output(residual) << std::endl;

        LOG(ERROR) << "local velocity residuals = " << Output(residual) << std::endl;

        // // 技术问题,Eigen无法赋值,手动处理
        // residual[0] = residual(0);
        // residual[1] = residual(1);
        // residual[2] = residual(2);

        // debug_ceres.debug_file << "local velocity origin = " << residuals[0] << ", " << residuals[1] << ", " << residuals[2] << std::endl;
        // LOG(ERROR) << "residuals done" << std::endl;

        // 不评估雅可比就返回
        if (!jacobians) {
          // LOG(ERROR) << "BodyLocalVelocityFactor No Jacobi!" << std::endl;
          // debug_ceres.debug_file << "BodyLocalVelocityFactor No Jacobi!" << std::endl;
          // debug_ceres.Close();

          // // debug_ceres_jacobis.Close();
          return true;
        }
        // debug_ceres_jacobis.Open();
        // debug_ceres_jacobis.debug_file << "Evaluate BodyLocalVelocityFactor Res J_R_0 J_R_1 J_R_2 J_R_3 J_p_0 J_p_1 J_p_2 J_p_3 J_linear_bias J_angular_bias J_t \n";
        // debug_ceres_jacobis.debug_file << Output(residual_temp.transpose()) << std::endl;
        

        // [1] 对 kont_R 的 雅可比
        Eigen::Matrix3d jac_lhs_R;
        // jac_lhs_R = - S_ItoG.matrix().transpose() * Skew(v_inG);


        // debug_ceres.debug_file << "R_GtoI = " << Output_M(R_GtoI) << std::endl;

        // Eigen::Matrix3d R_GtoI = R_ItoG.transpose().eval();
        Eigen::Matrix3d skew_v = Skew(v_inG);

        jac_lhs_R = -(R_GtoI * skew_v);
        LOG(ERROR) << "jac_lhs_R = " << jac_lhs_R << std::endl;

        // jac_lhs_R = (- R_ItoG.transpose() * skew_v).eval();

        Eigen::Matrix<double, 12, 4> Jac_R;
        for (size_t i = 0; i < knot_num; i++) {
          size_t idx = i;
          if (jacobians[idx]) {
            Eigen::Map<Eigen::Matrix<double, 3, 4, Eigen::RowMajor>> jac_kont_R(
                jacobians[idx]);
            jac_kont_R.setZero();

            jac_kont_R.block<3,3>(0,0) += (jac_lhs_R * J_R.d_val_d_knot[i]);
            jac_kont_R = (R_weight_ * jac_kont_R).eval(); // weight_ * 

            Eigen::Matrix<double, 3, 4> jac_kont_R_copy = jac_kont_R;
            // debug_ceres.debug_file << "jac_lhs_R = [" << Output_M(jac_lhs_R) << "]\n";
            // debug_ceres.debug_file << "J_R.d_val_d_knot[" << i << "] = [" << Output_M(J_R.d_val_d_knot[i]) << "]\n";
            // debug_ceres.debug_file << "jac_lhs_R_" << i << " = [" << Output_M(jac_kont_R_copy) << "]\n";

            Jac_R.block<3,4>(3 * i, 0) = jac_kont_R_copy;

            // debug_ceres_jacobis.debug_file <<  Output_M(jac_kont_R_copy) << std::endl;

          }
        }
        // LOG(ERROR) << "Add jacobians for Rotation control point " << std::endl;   

        // [2] 对 kont_p 的 雅可比
        Eigen::Matrix<double, 12, 3> Jac_p;
        // Eigen::Matrix3d Jac_p_temp;
 
        // // debug_ceres.debug_file << "R_ItoG = [" << Output_M(R_ItoG) << "]\n";
        // Jac_p_temp = R_ItoG.transpose();
        // // debug_ceres.debug_file << "R_ItoG.transpose() = [" << Output_M(Jac_p_temp) << "]\n";
        for (size_t i = 0; i < knot_num; i++) {
          size_t idx = i + knot_num;
          if (jacobians[idx]) {
            Eigen::Map<Eigen::Matrix3d> jac_kont_p(
                jacobians[idx]);
            jac_kont_p.setZero();

            // jac_kont_p += (Jac_p_temp * J_v.d_val_d_knot[i]);
            LOG(ERROR) << "J_v.d_val_d_knot["<< i << "] = " << J_v.d_val_d_knot[i] << std::endl;
            // Eigen::Matrix3d Jac_p_temp = R_GtoI * J_v.d_val_d_knot[i];

            // Eigen::Matrix3d Jac_p_temp;
            // Jac_p_temp.setIdentity();
            // HAO TODO: 定义在机体系
            Eigen::Matrix3d Jac_p_temp = Eigen::Matrix3d::Identity() * J_v.d_val_d_knot[i]; // = (R_GtoI * J_v.d_val_d_knot[i]);
            // Jac_p_temp.setIdentity();

            jac_kont_p = (weight_ * Jac_p_temp).eval();

            Eigen::Matrix3d jac_kont_p_copy = jac_kont_p;
            // debug_ceres.debug_file << "J_v.d_val_d_knot[" << i << "] = [" << J_v.d_val_d_knot[i]<< "]\n";
            // // debug_ceres.debug_file << "Jac_p_temp = [" << Output_M(Jac_p_temp) << "]\n";

            // debug_ceres.debug_file << "jac_kont_p_copy = [" << Output_M(jac_kont_p_copy) << "]\n";

            // // debug_ceres_jacobis.debug_file << Output_M(jac_kont_p_copy) << " ";

            Jac_p.block<3,3>(3 * i, 0) = jac_kont_p_copy;

            // debug_ceres_jacobis.debug_file <<  Output_M(jac_kont_p_copy) << std::endl;

          }
        }
        // LOG(ERROR) << "Add jacobians for Position control point " << std::endl;   


        // [3] 对 linear_bias 的 雅可比
        Eigen::Matrix3d Jac_v_bias;
        Eigen::Matrix3d jac_lhs_bias;
        jac_lhs_bias.setIdentity();
        if(jacobians[2 * knot_num])
        {
          // Eigen::Map<Eigen::Matrix<double, 1, 3>> J_linear_bias_(jacobians[2 * knot_num]); 
          Eigen::Map<Eigen::Matrix<double, 3, 3>> J_linear_bias_(jacobians[2 * knot_num]); 
          J_linear_bias_.setZero();

          // J_linear_bias_.setIdentity();

          // J_linear_bias_ = (-1.0 * weight_ * J_linear_bias_).eval();

          J_linear_bias_ = (-1.0 * weight_ * jac_lhs_bias).eval();

          // J_linear_bias_ = (-1.0 * weight_ * S_ItoG.matrix() * jac_lhs_bias).eval();
            
          Eigen::Matrix3d jac_linear_copy = J_linear_bias_;

          // debug_ceres.debug_file << "J_linear_bias_ = " << Output_M(jac_linear_copy) << "]\n";
        
          Jac_v_bias = jac_linear_copy;
          // debug_ceres_jacobis.debug_file << Output_M(Jac_v_bias) << std::endl;
        }
        // LOG(ERROR) << "Add jacobians for Linear velocity bias " << std::endl;  
    
        LOG(ERROR) << "All Jacobi for BodyLocalVelocity factor" << std::endl;
        // debug_ceres.debug_file << "All Jacobi for Event factor" << std::endl;
        // debug_ceres.debug_file << "Jac_R_ = " << Output_M(Jac_R) << std::endl;
        // debug_ceres.debug_file << "Jac_p_ = " << Output_M(Jac_p) << std::endl;
        // debug_ceres.debug_file << "Jac_linear_bias_ = " << Output_M(Jac_v_bias) << std::endl;

        LOG(ERROR) << "Jac_R_ = " << Output_M(Jac_R) << std::endl;
        LOG(ERROR) << "Jac_p_ = " << Output_M(Jac_p) << std::endl;
        LOG(ERROR) << "Jac_linear_bias_ = " << Output_M(Jac_v_bias) << std::endl;

        // debug_ceres.Close();
        // debug_ceres_jacobis.Close();
        return true;
    }

private:
    int64_t time_ns_;
    SplineMeta<SplineOrder> spline_meta_;
    Eigen::Vector3d local_vel_;
    // Eigen::Vector3d linear_bias_;
    double weight_;
    double w_weight_;
    double R_weight_;
};



class BodyLocalAngularVelocityFactor : public ceres::CostFunction, SplitSpineView{
 public:
 EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  using SO3View = So3SplineView;
  using R3View = RdSplineView;
  using SplitView = SplitSpineView;

  using Vec3d = Eigen::Matrix<double, 3, 1>;
  using Vec6d = Eigen::Matrix<double, 6, 1>;
  using Mat3d = Eigen::Matrix<double, 3, 3>;
  using Quatd = Eigen::Quaterniond;
  using SO3d = Sophus::SO3<double>;

  BodyLocalAngularVelocityFactor(int64_t time_ns, const Eigen::Vector3d local_angular_vel,
            // const Eigen::Vector3d angular_bias,
            const SplineMeta<SplineOrder>& spline_segment_meta,
            double weight, double w_weight = 0.05) // const Vec6d& info_vec)
      : time_ns_(time_ns),local_angular_vel_(local_angular_vel),
      // angular_bias_(angular_bias),
        spline_meta_(spline_segment_meta),
        weight_(weight), w_weight_(w_weight)
        {
          set_num_residuals(3);           // 定义残差值的大小(事件角速度残差)

          size_t knot_num = this->spline_meta_.NumParameters();
          // LOG(ERROR) << "knot_num = " << knot_num << std::endl;

          // TODO: 需要提供采样曲线的多普勒速度和偏执                   
          for (size_t i = 0; i < knot_num; ++i) {             
            mutable_parameter_block_sizes()->push_back(4);   // rotation
          }
          for (size_t i = 0; i < knot_num; ++i) {
            mutable_parameter_block_sizes()->push_back(3);   // position
          }
          mutable_parameter_block_sizes()->push_back(3);    // angular bias // 1 -11 修改
        
        }

  virtual bool Evaluate(double const* const* parameters, double* residuals,
                          double** jacobians) const {
        typename SO3View::JacobianStruct J_R;
        typename SO3View::JacobianStruct J_w;
        typename R3View::JacobianStruct J_v;

        LOG(ERROR) << "Evaluate BodyLocalAngularVelocityFactor " << std::endl;
        // debug_ceres.Open();
        // debug_ceres.debug_file << std::endl;
        // debug_ceres.debug_file << "    ---------------------- Evaluate BodyLocalAngularVelocityFactor ------------------    " << std::endl;

        size_t knot_num = spline_meta_.NumParameters();
        Eigen::Map<Vec3d const> angular_bias(parameters[2 * knot_num]);

        // double time_offset_in_ns = parameters[t_offset_index][0];
        double time_offset_in_ns = 0;
        int64_t t_corrected = time_ns_ + (int64_t)time_offset_in_ns;
        t_corrected = std::min(t_corrected, spline_meta_.segments.at(0).MaxTimeNs() - 1);
        t_corrected = std::max(t_corrected, spline_meta_.segments.at(0).MinTimeNs() + 1);

        Eigen::Vector3d gyro_, rot_accel, v_inG;
        SO3d S_ItoG;
        if (jacobians){
          // S_ItoG = SO3View::EvaluateRp(t_corrected, spline_meta_.segments.at(0),
          //                 parameters, &J_R);

          gyro_ = SO3View::VelocityBody(t_corrected,
                                              spline_meta_.segments.at(0),
                                              parameters, &J_w);

          // rot_accel = SO3View::accelerationBody(t_corrected,  
          //                                       spline_meta_.segments.at(0), 
          //                                       parameters);

          // v_inG = R3View::velocity(t_corrected,
          //                               spline_meta_.segments.at(0),
          //                               parameters + knot_num, &J_v);   
        }else{
          // S_ItoG = SO3View::EvaluateRp(t_corrected, spline_meta_.segments.at(0),
          //                 parameters, nullptr);
                                
          gyro_ = SO3View::VelocityBody(t_corrected,
                                          spline_meta_.segments.at(0),
                                          parameters, nullptr);

          // rot_accel = SO3View::accelerationBody(t_corrected,  
          //                               spline_meta_.segments.at(0), 
          //                               parameters);
        
          // v_inG = R3View::velocity(t_corrected,
          //                       spline_meta_.segments.at(0),
          //                       parameters + knot_num, nullptr);   
        }
        // LOG(ERROR) << "Set Jacobi Matrix " << std::endl;



        // debug_ceres.debug_file << "Event Details: weight_ = " << weight_ 
                                      // << "\n w_weight_ = [" <<  w_weight_
                                      // // << "\n S_ItoG = [" <<  Output_M(S_ItoG)
                                      // << "\n local_angular_vel_= [" <<  Output(local_angular_vel_) << "] "
                                      // << "\n gyro_ = [" <<  Output(gyro_.transpose()) << "] "
                                      // << "\n angular_bias = [" <<  Output(angular_bias) << "] " << std::endl;

        // LOG(ERROR) << "Event Details: weight_ = " << weight_ 
        //                               << "\n w_weight_ = [" <<  w_weight_
        //                               // << "\n S_ItoG = [" <<  Output_M(S_ItoG)
        //                               << "\n local_angular_vel_= [" <<  Output(local_angular_vel_) << "] "
        //                               << "\n gyro_ = [" <<  Output(gyro_.transpose()) << "] "
        //                               << "\n angular_bias = [" <<  Output(angular_bias) << "] " << std::endl;


        Eigen::Map<Eigen::Vector3d> residual(residuals);
        residual.setZero();
        Eigen::Vector3d residual_temp = gyro_ - local_angular_vel_ - angular_bias;
        residual = residual_temp;
        residual = (weight_ * w_weight_ * residual).eval();
        // debug_ceres.debug_file << "local angular velocity residuals = " 
                                // << Output(residual) << std::endl;

        LOG(ERROR) << "local angular velocity residuals = " 
                                << Output(residual) << std::endl;

        // // 技术问题,Eigen无法赋值,手动处理
        // residual[0] = residual(0);
        // residual[1] = residual(1);
        // residual[2] = residual(2);

        // debug_ceres.debug_file << "local angular velocity origin = " << residuals[0] << ", " << residuals[1] << ", " << residuals[2] << std::endl;
        // LOG(ERROR) << "residuals done" << std::endl;

        // 不评估雅可比就返回
        if (!jacobians) {
          // LOG(ERROR) << "BodyLocalAngularVelocityFactor No Jacobi!" << std::endl;
          // debug_ceres.debug_file << "BodyLocalAngularVelocityFactor No Jacobi!" << std::endl;
          // debug_ceres.Close();

          // // debug_ceres_jacobis.Close();
          return true;
        }
        // debug_ceres_jacobis.Open();
        // debug_ceres_jacobis.debug_file << "Evaluate EventAgularFactor Res J_R_0 J_R_1 J_R_2 J_R_3 J_p_0 J_p_1 J_p_2 J_p_3 J_linear_bias J_angular_bias J_t \n";
        // debug_ceres_jacobis.debug_file << Output(residual_temp.transpose()) << std::endl;
        

        // [1] 对 kont_R 的 雅可比
        Eigen::Matrix<double, 12, 4> Jac_R;
        for (size_t i = 0; i < knot_num; i++) {
          size_t idx = i;
          if (jacobians[idx]) {
            Eigen::Map<Eigen::Matrix<double, 3, 4, Eigen::RowMajor>> jac_kont_R(
                jacobians[idx]);
            jac_kont_R.setZero();

            jac_kont_R.block<3,3>(0,0) += J_w.d_val_d_knot[i];
            jac_kont_R = (weight_ * jac_kont_R).eval();

            // LOG(ERROR) << "J_w.d_val_d_knot["<< i << "] = " << J_w.d_val_d_knot[i] << std::endl;

            Eigen::Matrix<double, 3, 4> jac_kont_R_copy = jac_kont_R;
            // debug_ceres.debug_file << "J_w.d_val_d_knot[" << i << "] = [" << Output_M(J_w.d_val_d_knot[i]) << "]\n";
            // // debug_ceres.debug_file << "jac_lhs_R_" << i << " = [" << Output_M(jac_kont_R_copy) << "]\n";
            
            // debug_ceres.debug_file << "jac_kont_R_copy = [" << Output_M(jac_kont_R_copy) << "]\n";
            Jac_R.block<3,4>(3 * i, 0) = jac_kont_R_copy;

            // debug_ceres_jacobis.debug_file << Output_M(jac_kont_R_copy)  << std::endl;

          }
        }
        // LOG(ERROR) << "Add jacobians for Rotation control point " << std::endl;   

        // [2] 对 kont_p 的 雅可比
        Eigen::Matrix<double, 12, 3> Jac_p;
        for (size_t i = 0; i < knot_num; i++) {
          size_t idx = i + knot_num;
          if (jacobians[idx]) {
            Eigen::Map<Eigen::Matrix3d> jac_kont_p(
                jacobians[idx]);
            jac_kont_p.setZero();

            // jac_kont_p = S_ItoG.matrix().transpose() * J_v.d_val_d_knot[i];
            // jac_kont_p = (weight * jac_kont_p).eval();

            Eigen::Matrix3d jac_kont_p_copy = jac_kont_p;
            // // debug_ceres.debug_file << "J_v.d_val_d_knot[" << i << "] = [" << Output_M(J_v.d_val_d_knot[i]) << "]\n";
            // // debug_ceres.debug_file << "jac_lhs_R_" << i << " = [" << Output_M(jac_kont_p_copy) << "]\n";

            Jac_p.block<3,3>(3 * i, 0) = jac_kont_p_copy;

            // debug_ceres_jacobis.debug_file << Output_M(jac_kont_p)  << std::endl;

          }
        }
        // LOG(ERROR) << "Add jacobians for Position control point " << std::endl;  

        // [3] 对 angular_bias 的 雅可比
        Eigen::Matrix3d Jac_w_bias;
        if(jacobians[2 * knot_num])
        {
          // Eigen::Map<Eigen::Matrix<double, 1, 3>> J_linear_bias_(jacobians[2 * knot_num]); 
          Eigen::Map<Eigen::Matrix<double, 3, 3>> J_angular_bias_(jacobians[2 * knot_num]); 
          J_angular_bias_.setZero();

          J_angular_bias_.setIdentity();

          J_angular_bias_ = (-1.0 * weight_ * J_angular_bias_).eval();
            
          Eigen::Matrix3d jac_angular_copy = J_angular_bias_;

          Jac_w_bias = J_angular_bias_;

          // debug_ceres.debug_file << "J_angular_bias_ = " << Output_M(jac_angular_copy) << "]\n";


          // debug_ceres_jacobis.debug_file << Output_M(jac_angular_copy) << std::endl;
        }
        // LOG(ERROR) << "Add jacobians for Angular velocity bias " << std::endl;  


        // LOG(ERROR) << "All Jacobi for BodyLocalAngularVelocity factor" << std::endl;
        // debug_ceres.debug_file << "All Jacobi for BodyLocalAngularVelocity factor" << std::endl;
        // debug_ceres.debug_file << "Jac_R_ = " << Output_M(Jac_R) << std::endl;
        // debug_ceres.debug_file << "Jac_p_ = " << Output_M(Jac_p) << std::endl;
        // debug_ceres.debug_file << "J_angular_bias_ = " << Output_M(Jac_w_bias) << std::endl;

        // LOG(ERROR) << "Jac_R_ = " << Output_M(Jac_R) << std::endl;
        // LOG(ERROR) << "Jac_p_ = " << Output_M(Jac_p) << std::endl;
        // LOG(ERROR) << "J_angular_bias_ = " << Output_M(Jac_w_bias) << std::endl;


        // debug_ceres.Close();
        // debug_ceres_jacobis.Close();

        return true;
    }
    
private:
    int64_t time_ns_;
    SplineMeta<SplineOrder> spline_meta_;
    Eigen::Vector3d local_angular_vel_;
    // Eigen::Vector3d angular_bias_;
    double weight_;
    double w_weight_;
};

class BodyLocalAngularVelocityFactor2 : public ceres::CostFunction, SplitSpineView{
 public:
 EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  using SO3View = So3SplineView;
  using R3View = RdSplineView;
  using SplitView = SplitSpineView;

  using Vec3d = Eigen::Matrix<double, 3, 1>;
  using Vec6d = Eigen::Matrix<double, 6, 1>;
  using Mat3d = Eigen::Matrix<double, 3, 3>;
  using Quatd = Eigen::Quaterniond;
  using SO3d = Sophus::SO3<double>;

  BodyLocalAngularVelocityFactor2(int64_t time_ns, const Eigen::Vector3d local_angular_vel,
            // const Eigen::Vector3d angular_bias,
            const SplineMeta<SplineOrder>& spline_segment_meta,
            double weight, double w_weight = 0.05) // const Vec6d& info_vec)
      : time_ns_(time_ns),local_angular_vel_(local_angular_vel),
      // angular_bias_(angular_bias),
        spline_meta_(spline_segment_meta),
        weight_(weight), w_weight_(w_weight)
        {
          set_num_residuals(3);           // 定义残差值的大小(事件角速度残差)

          size_t knot_num = this->spline_meta_.NumParameters();
          // LOG(ERROR) << "knot_num = " << knot_num << std::endl;

          // TODO: 需要提供采样曲线的多普勒速度和偏执                   
          // for (size_t i = 0; i < knot_num; ++i) {             
          //   mutable_parameter_block_sizes()->push_back(4);   // rotation
          // }
          for (size_t i = 0; i < knot_num; ++i) {
            mutable_parameter_block_sizes()->push_back(3);   // position
          }
          mutable_parameter_block_sizes()->push_back(3);    // angular bias // 1 -11 修改
        
        }

  virtual bool Evaluate(double const* const* parameters, double* residuals,
                          double** jacobians) const {
        // typename SO3View::JacobianStruct J_R;
        // typename SO3View::JacobianStruct J_w;
        typename R3View::JacobianStruct J_w;
        typename R3View::JacobianStruct J_v;

        LOG(ERROR) << "Evaluate BodyLocalAngularVelocityFactor " << std::endl;
        // debug_ceres.Open();
        // debug_ceres.debug_file << std::endl;
        // debug_ceres.debug_file << "    ---------------------- Evaluate BodyLocalAngularVelocityFactor ------------------    " << std::endl;

        size_t knot_num = spline_meta_.NumParameters();
        // Eigen::Map<Vec3d const> angular_bias(parameters[2 * knot_num]);
        Eigen::Map<Vec3d const> angular_bias(parameters[knot_num]);

        // double time_offset_in_ns = parameters[t_offset_index][0];
        double time_offset_in_ns = 0;
        int64_t t_corrected = time_ns_ + (int64_t)time_offset_in_ns;
        t_corrected = std::min(t_corrected, spline_meta_.segments.at(0).MaxTimeNs() - 1);
        t_corrected = std::max(t_corrected, spline_meta_.segments.at(0).MinTimeNs() + 1);

        Eigen::Vector3d gyro_, rot_accel, v_inG;
        SO3d S_ItoG;
        if (jacobians){
          // S_ItoG = SO3View::EvaluateRp(t_corrected, spline_meta_.segments.at(0),
          //                 parameters, &J_R);

          // gyro_ = SO3View::VelocityBody(t_corrected,
          //                                     spline_meta_.segments.at(0),
          //                                     parameters, &J_w);

          gyro_ = R3View::evaluate(t_corrected,
                                              spline_meta_.segments.at(0),
                                              parameters, &J_w);

          // rot_accel = SO3View::accelerationBody(t_corrected,  
          //                                       spline_meta_.segments.at(0), 
          //                                       parameters);

          // v_inG = R3View::velocity(t_corrected,
          //                               spline_meta_.segments.at(0),
          //                               parameters + knot_num, &J_v);   
        }else{
          // S_ItoG = SO3View::EvaluateRp(t_corrected, spline_meta_.segments.at(0),
          //                 parameters, nullptr);
                                
          // gyro_ = SO3View::VelocityBody(t_corrected,
          //                                 spline_meta_.segments.at(0),
          //                                 parameters, nullptr);

          gyro_ = R3View::evaluate(t_corrected,
                                              spline_meta_.segments.at(0),
                                              parameters, nullptr);

          // rot_accel = SO3View::accelerationBody(t_corrected,  
          //                               spline_meta_.segments.at(0), 
          //                               parameters);
        
          // v_inG = R3View::velocity(t_corrected,
          //                       spline_meta_.segments.at(0),
          //                       parameters + knot_num, nullptr);   
        }
        // LOG(ERROR) << "Set Jacobi Matrix " << std::endl;



        // debug_ceres.debug_file << "Event Details: weight_ = " << weight_ 
                                      // << "\n w_weight_ = [" <<  w_weight_
                                      // // << "\n S_ItoG = [" <<  Output_M(S_ItoG)
                                      // << "\n local_angular_vel_= [" <<  Output(local_angular_vel_) << "] "
                                      // << "\n gyro_ = [" <<  Output(gyro_.transpose()) << "] "
                                      // << "\n angular_bias = [" <<  Output(angular_bias) << "] " << std::endl;

        // LOG(ERROR) << "Event Details: weight_ = " << weight_ 
        //                               << "\n w_weight_ = [" <<  w_weight_
        //                               // << "\n S_ItoG = [" <<  Output_M(S_ItoG)
        //                               << "\n local_angular_vel_= [" <<  Output(local_angular_vel_) << "] "
        //                               << "\n gyro_ = [" <<  Output(gyro_.transpose()) << "] "
        //                               << "\n angular_bias = [" <<  Output(angular_bias) << "] " << std::endl;

        LOG(ERROR) << "Event Details: weight_ = " << weight_ 
                                      << "\n w_weight_ = [" <<  w_weight_
                                      << "\n local_angular_vel_= [" <<  Output(local_angular_vel_) << "] "
                                      << "\n gyro_ = [" <<  Output(gyro_.transpose()) << "] "
                                      << "\n angular_bias = [" <<  Output(angular_bias) << "] " << std::endl;

        Eigen::Map<Eigen::Vector3d> residual(residuals);
        residual.setZero();
        Eigen::Vector3d residual_temp = gyro_ - local_angular_vel_ - angular_bias;
        residual = residual_temp;
        residual = (weight_ * w_weight_ * residual).eval();
        // debug_ceres.debug_file << "local angular velocity residuals = " 
                                // << Output(residual) << std::endl;

        LOG(ERROR) << "local angular velocity residuals = " 
                                << Output(residual) << std::endl;

        // // 技术问题,Eigen无法赋值,手动处理
        // residual[0] = residual(0);
        // residual[1] = residual(1);
        // residual[2] = residual(2);

        // debug_ceres.debug_file << "local angular velocity origin = " << residuals[0] << ", " << residuals[1] << ", " << residuals[2] << std::endl;
        // LOG(ERROR) << "residuals done" << std::endl;

        // 不评估雅可比就返回
        if (!jacobians) {
          // LOG(ERROR) << "BodyLocalAngularVelocityFactor No Jacobi!" << std::endl;
          // debug_ceres.debug_file << "BodyLocalAngularVelocityFactor No Jacobi!" << std::endl;
          // debug_ceres.Close();

          // // debug_ceres_jacobis.Close();
          return true;
        }
        // debug_ceres_jacobis.Open();
        // debug_ceres_jacobis.debug_file << "Evaluate EventAgularFactor Res J_R_0 J_R_1 J_R_2 J_R_3 J_p_0 J_p_1 J_p_2 J_p_3 J_linear_bias J_angular_bias J_t \n";
        // debug_ceres_jacobis.debug_file << Output(residual_temp.transpose()) << std::endl;
        

        // [1] 对 kont_R 的 雅可比
        /*Eigen::Matrix<double, 12, 4> Jac_R;
        for (size_t i = 0; i < knot_num; i++) {
          size_t idx = i;
          if (jacobians[idx]) {
            Eigen::Map<Eigen::Matrix<double, 3, 4, Eigen::RowMajor>> jac_kont_R(
                jacobians[idx]);
            jac_kont_R.setZero();

            jac_kont_R.block<3,3>(0,0) += J_w.d_val_d_knot[i];
            jac_kont_R = (weight_ * jac_kont_R).eval();

            // LOG(ERROR) << "J_w.d_val_d_knot["<< i << "] = " << J_w.d_val_d_knot[i] << std::endl;

            Eigen::Matrix<double, 3, 4> jac_kont_R_copy = jac_kont_R;
            // debug_ceres.debug_file << "J_w.d_val_d_knot[" << i << "] = [" << Output_M(J_w.d_val_d_knot[i]) << "]\n";
            // // debug_ceres.debug_file << "jac_lhs_R_" << i << " = [" << Output_M(jac_kont_R_copy) << "]\n";
            
            // debug_ceres.debug_file << "jac_kont_R_copy = [" << Output_M(jac_kont_R_copy) << "]\n";
            Jac_R.block<3,4>(3 * i, 0) = jac_kont_R_copy;

            // debug_ceres_jacobis.debug_file << Output_M(jac_kont_R_copy)  << std::endl;

          }
        }*/
        // LOG(ERROR) << "Add jacobians for Rotation control point " << std::endl;   

        // [2] 对 kont_p 的 雅可比
        Eigen::Matrix<double, 12, 3> Jac_p;
        for (size_t i = 0; i < knot_num; i++) {
          size_t idx = i;
          if (jacobians[idx]) {
            Eigen::Map<Eigen::Matrix3d> jac_kont_p(
                jacobians[idx]);
            jac_kont_p.setIdentity();

            // jac_kont_p = S_ItoG.matrix().transpose() * J_v.d_val_d_knot[i];
            // jac_kont_p = (weight * jac_kont_p).eval();

            jac_kont_p = (weight_ * jac_kont_p * J_w.d_val_d_knot[i]).eval();

            Eigen::Matrix3d jac_kont_p_copy = jac_kont_p;
            // // debug_ceres.debug_file << "J_v.d_val_d_knot[" << i << "] = [" << Output_M(J_v.d_val_d_knot[i]) << "]\n";
            // // debug_ceres.debug_file << "jac_lhs_R_" << i << " = [" << Output_M(jac_kont_p_copy) << "]\n";

            Jac_p.block<3,3>(3 * i, 0) = jac_kont_p_copy;

            // debug_ceres_jacobis.debug_file << Output_M(jac_kont_p)  << std::endl;

          }
        }
        // LOG(ERROR) << "Add jacobians for Position control point " << std::endl;  

        // [3] 对 angular_bias 的 雅可比
        Eigen::Matrix3d Jac_w_bias;
        if(jacobians[knot_num])
        {
          // Eigen::Map<Eigen::Matrix<double, 1, 3>> J_linear_bias_(jacobians[2 * knot_num]); 
          Eigen::Map<Eigen::Matrix<double, 3, 3>> J_angular_bias_(jacobians[knot_num]); 
          J_angular_bias_.setZero();

          J_angular_bias_.setIdentity();

          J_angular_bias_ = (-1.0 * weight_ * J_angular_bias_).eval();
            
          Eigen::Matrix3d jac_angular_copy = J_angular_bias_;

          Jac_w_bias = J_angular_bias_;

          // debug_ceres.debug_file << "J_angular_bias_ = " << Output_M(jac_angular_copy) << "]\n";


          // debug_ceres_jacobis.debug_file << Output_M(jac_angular_copy) << std::endl;
        }
        // LOG(ERROR) << "Add jacobians for Angular velocity bias " << std::endl;  


        // LOG(ERROR) << "All Jacobi for BodyLocalAngularVelocity factor" << std::endl;
        // debug_ceres.debug_file << "All Jacobi for BodyLocalAngularVelocity factor" << std::endl;
        // debug_ceres.debug_file << "Jac_R_ = " << Output_M(Jac_R) << std::endl;
        // debug_ceres.debug_file << "Jac_p_ = " << Output_M(Jac_p) << std::endl;
        // debug_ceres.debug_file << "J_angular_bias_ = " << Output_M(Jac_w_bias) << std::endl;

        // LOG(ERROR) << "Jac_R_ = " << Output_M(Jac_R) << std::endl;
        LOG(ERROR) << "Jac_w_ = " << Output_M(Jac_p) << std::endl;
        LOG(ERROR) << "J_angular_bias_ = " << Output_M(Jac_w_bias) << std::endl;


        // debug_ceres.Close();
        // debug_ceres_jacobis.Close();

        return true;
    }
    
private:
    int64_t time_ns_;
    SplineMeta<SplineOrder> spline_meta_;
    Eigen::Vector3d local_angular_vel_;
    // Eigen::Vector3d angular_bias_;
    double weight_;
    double w_weight_;
};

  
// pixel_cord R_e_r v_r t_r_e normal_flow normal_norm
// linear_bias gyro_ omega_bias
class EventAgularFactor2 : public ceres::CostFunction, SplitSpineView{
 public:
 EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  using SO3View = So3SplineView;
  using R3View = RdSplineView;
  using SplitView = SplitSpineView;

  using Vec3d = Eigen::Matrix<double, 3, 1>;
  using Vec6d = Eigen::Matrix<double, 6, 1>;
  using Mat3d = Eigen::Matrix<double, 3, 3>;
  using Quatd = Eigen::Quaterniond;
  using SO3d = Sophus::SO3<double>;

  EventAgularFactor2(int64_t time_ns, const Eigen::Vector3d pt, const event_flow_velocity flow, 
            const Eigen::Vector3d doppler_velocity,
            const Eigen::Quaterniond q_e_r, const Eigen::Vector3d t_e_r,
            const SplineMeta<SplineOrder>& spline_segment_meta,
            double weight, double w_weight = 0.05) // const Vec6d& info_vec)
      : time_ns_(time_ns),
        pixel_cord(pt), doppler_velocity_(doppler_velocity),
        q_e_r(q_e_r), t_e_r(t_e_r),
        spline_meta_(spline_segment_meta),
        lock_extrincs(true),      // HAO TODO: 暂时不优化外参
        weight_(weight), w_weight_(w_weight)
        {
          set_num_residuals(3);           // 定义残差值的大小(事件角速度残差)

          size_t knot_num = this->spline_meta_.NumParameters();
          // LOG(ERROR) << "knot_num = " << knot_num << std::endl;
          flow_ << flow.x, flow.y, 0.0;
          // TODO: 需要角速度曲线              
          for (size_t i = 0; i < knot_num; ++i) {
            mutable_parameter_block_sizes()->push_back(3);   // angular velocity
          }

          // mutable_parameter_block_sizes()->push_back(3);    // linear bias // 1 -11 修改
          
          // mutable_parameter_block_sizes()->push_back(3);    // angular bias // 1 -11 修改
        }

  virtual bool Evaluate(double const* const* parameters, double* residuals,
                          double** jacobians) const {
        typename R3View::JacobianStruct J_w_;

        LOG(ERROR) << "Evaluate EventAgularFactor2 " << std::endl;
        // debug_ceres.Open();
        // debug_ceres.debug_file << std::endl;
        // debug_ceres.debug_file << "    ---------------------- Evaluate BodyLocalAngularVelocityFactor ------------------    " << std::endl;

        size_t knot_num = spline_meta_.NumParameters();

        // double time_offset_in_ns = parameters[t_offset_index][0];
        double time_offset_in_ns = 0;
        int64_t t_corrected = time_ns_ + (int64_t)time_offset_in_ns;
        t_corrected = std::min(t_corrected, spline_meta_.segments.at(0).MaxTimeNs() - 1);
        t_corrected = std::max(t_corrected, spline_meta_.segments.at(0).MinTimeNs() + 1);
  
        Eigen::Vector3d gyro_, rot_accel;
        if (jacobians){
          // S_ItoG = SO3View::EvaluateRp(t_corrected, spline_meta_.segments.at(0),
          //                 parameters, &J_R);

          // gyro_ = R3View::velocity(t_corrected,
          //                               spline_meta_.segments.at(0),
          //                               parameters, &J_w_);   

          gyro_ = R3View::evaluate(t_corrected,
                                        spline_meta_.segments.at(0),
                                        parameters, &J_w_);   

          // rot_accel = SO3View::accelerationBody(t_corrected,  
          //                                       spline_meta_.segments.at(0), 
          //                                       parameters);

          // v_inG = R3View::velocity(t_corrected,
          //                               spline_meta_.segments.at(0),
          //                               parameters + knot_num, &J_v);   
        }else{
          // S_ItoG = SO3View::EvaluateRp(t_corrected, spline_meta_.segments.at(0),
          //                 parameters, nullptr);

          // gyro_ = R3View::velocity(t_corrected,
          //                     spline_meta_.segments.at(0),
          //                     parameters, nullptr);  
                                
          gyro_ = R3View::evaluate(t_corrected,
                                        spline_meta_.segments.at(0),
                                        parameters, nullptr);  

          // rot_accel = SO3View::accelerationBody(t_corrected,  
          //                               spline_meta_.segments.at(0), 
          //                               parameters);
        
          // v_inG = R3View::velocity(t_corrected,
          //                       spline_meta_.segments.at(0),
          //                       parameters + knot_num, nullptr);   
        }
        // LOG(ERROR) << "Set Jacobi Matrix " << std::endl;



        // debug_ceres.debug_file << "Event Details: weight_ = " << weight_ 
                                      // << "\n w_weight_ = [" <<  w_weight_
                                      // // << "\n S_ItoG = [" <<  Output_M(S_ItoG)
                                      // << "\n local_angular_vel_= [" <<  Output(local_angular_vel_) << "] "
                                      // << "\n gyro_ = [" <<  Output(gyro_.transpose()) << "] "
                                      // << "\n angular_bias = [" <<  Output(angular_bias) << "] " << std::endl;

        // LOG(ERROR) << "Event Details: weight_ = " << weight_ 
        //                               << "\n w_weight_ = [" <<  w_weight_
        //                               // << "\n S_ItoG = [" <<  Output_M(S_ItoG)
        //                               << "\n local_angular_vel_= [" <<  Output(local_angular_vel_) << "] "
        //                               << "\n gyro_ = [" <<  Output(gyro_.transpose()) << "] "
        //                               << "\n angular_bias = [" <<  Output(angular_bias) << "] " << std::endl;

      // 3-18 修改 法向光流的计算
      Eigen::Vector3d grad;
      grad << -1.0 / flow_(0), -1.0 / flow_(1), 0.0;
      double normal_norm = 1.0 / grad.norm();
      Eigen::Vector3d normal_flow = grad * normal_norm;

      // Eigen::Map<Vec3d const> linear_bias(parameters[knot_num]);
      // Eigen::Map<Vec3d const> omega_bias(parameters[knot_num + 1]);

      Eigen::Matrix3d R_e_r = q_e_r.toRotationMatrix(); 
      Eigen::Vector3d t_r_e_temp = R_e_r.transpose() * t_e_r;
      Eigen::Vector3d t_r_e = -1.0 * t_r_e_temp;
      

      // Eigen::Vector3d pre_vec = Skew(pixel_cord) * (R_e_r.transpose() * ((doppler_velocity_ + linear_bias) + (gyro_ + R_e_r * omega_bias).cross(t_r_e)));      
      // // use for transport
      // double post_vec = normal_norm + normal_flow.transpose() * Skew(pixel_cord) * (R_e_r.transpose() * gyro_ + omega_bias);

      // 去bias
      Eigen::Vector3d pre_vec = Skew(pixel_cord) * (R_e_r.transpose() * ((doppler_velocity_) + (gyro_).cross(t_r_e)));      
      // use for transport
      double post_vec = normal_norm + normal_flow.transpose() * Skew(pixel_cord) * (R_e_r.transpose() * gyro_);


      LOG(ERROR) << "Event Details: weight_ = " << weight_ 
                   << "\n w_weight_ = " <<  w_weight_
                   << "\n pixel_cord = " <<  Output_M(pixel_cord)
                   << "\n flow_ = " <<  Output(flow_)
                   << "\n doppler_velocity_ = " <<  Output(doppler_velocity_)
                  //  << "\n linear_bias = " <<  Output(linear_bias)
                   << "\n R_e_r = " <<  Output_M(R_e_r)
                   << "\n t_r_e = " << Output(t_r_e.transpose()) 
                   << "\n gyro_ = " <<  Output(gyro_) 
                   << "\n normal_norm = " <<  normal_norm 
                   << "\n normal_flow = " <<  Output(normal_flow) 
                  //  << "\n omega_bias = " <<  Output(omega_bias) 
                   << std::endl;
      LOG(ERROR) << "pre_vec = " << Output(pre_vec)
                 << "\n post_vec = " << post_vec << std::endl;

        Eigen::Map<Eigen::Vector3d> residual(residuals);
        residual.setZero();
        Eigen::Vector3d residual_temp = pre_vec * post_vec;
        residual = residual_temp;
        residual = (weight_ * w_weight_ * residual).eval();

        LOG(ERROR) << "local angular velocity residuals = " 
                                << Output(residual) << std::endl;

        // 不评估雅可比就返回
        if (!jacobians) {
          // LOG(ERROR) << "BodyLocalAngularVelocityFactor No J" << std::endl;
          // debug_ceres.debug_file << "BodyLocalAngularVelocityFactor No Jacobi!" << std::endl;
          // debug_ceres.Close();

          // // debug_ceres_jacobis.Close();
          return true;
        }
        // debug_ceres_jacobis.Open();
        // debug_ceres_jacobis.debug_file << "Evaluate EventAgularFactor Res J_R_0 J_R_1 J_R_2 J_R_3 J_p_0 J_p_1 J_p_2 J_p_3 J_linear_bias J_angular_bias J_t \n";
        // debug_ceres_jacobis.debug_file << Output(residual_temp.transposacobi!e()) << std::endl;
 
        // [1] 对 kont_w 的 雅可比
        Eigen::Matrix3d Jac_gyro_pre = - Skew(pixel_cord) * R_e_r.transpose() * Skew(t_r_e);
        Eigen::RowVector3d Jac_gyro_post = normal_flow.transpose() * Skew(pixel_cord) * R_e_r.transpose();
        // LOG(ERROR) << "Jac_gyro_pre = " << Output_M(Jac_gyro_pre) << std::endl;
        // LOG(ERROR) << "Jac_gyro_post = " << Output_M(Jac_gyro_post) << std::endl;

        Eigen::Matrix<double, 12, 3> Jac_w;
        for (size_t i = 0; i < knot_num; i++) {
          size_t idx = i;
          if (jacobians[idx]) {
            Eigen::Map<Eigen::Matrix3d> jac_kont_w(
                jacobians[idx]);
            jac_kont_w.setZero();

            // jac_kont_p = S_ItoG.matrix().transpose() * J_v.d_val_d_knot[i];
            // jac_kont_p = (weight * jac_kont_p).eval();
            Eigen::Matrix3d Jac_gyro_ = (Jac_gyro_pre * J_w_.d_val_d_knot[i]) * post_vec + pre_vec * Jac_gyro_post * J_w_.d_val_d_knot[i];
            jac_kont_w = (weight_ * Jac_gyro_).eval();

            Eigen::Matrix3d jac_kont_w_copy = jac_kont_w;

            // LOG(ERROR) << "J_w_.d_val_d_knot[" << i << "] = [" << J_w_.d_val_d_knot[i] << "]\n";
            // // debug_ceres.debug_file << "J_v.d_val_d_knot[" << i << "] = [" << Output_M(J_v.d_val_d_knot[i]) << "]\n";
            // // debug_ceres.debug_file << "jac_lhs_R_" << i << " = [" << Output_M(jac_kont_p_copy) << "]\n";

            Jac_w.block<3,3>(3 * i, 0) = jac_kont_w_copy;

            // debug_ceres_jacobis.debug_file << Output_M(jac_kont_p)  << std::endl;

          }
        }
        // LOG(ERROR) << "Add jacobians for Position control point " << std::endl;  

        /*
        // [2] 对 linear_bias 的 雅可比
        Eigen::Matrix3d Jac_linear_bias_pre_ = Skew(pixel_cord) * R_e_r.transpose();
        Eigen::Matrix3d Jac_linear_bias_ = Jac_linear_bias_pre_ * post_vec;
        // LOG(ERROR) << "Jac_linear_bias_pre_ = " << Output_M(Jac_linear_bias_pre_) << std::endl;
        // LOG(ERROR) << "Jac_linear_bias_ = " << Output_M(Jac_linear_bias_) << std::endl;
        Eigen::Matrix3d J_linear_bias_copy;
        if(jacobians[knot_num])
        {
          // Eigen::Map<Eigen::Matrix<double, 1, 3>> J_linear_bias_(jacobians[2 * knot_num]); 
          Eigen::Map<Eigen::Matrix<double, 3, 3>> J_linear_bias_(jacobians[knot_num]); 
          J_linear_bias_.setZero();

          J_linear_bias_ = (weight_ * 0.01 * Jac_linear_bias_).eval();

          J_linear_bias_copy = J_linear_bias_;

          // debug_ceres.debug_file << "J_angular_bias_ = " << Output_M(jac_angular_copy) << "]\n";


          // debug_ceres_jacobis.debug_file << Output_M(jac_angular_copy) << std::endl;
        }

        // [3] 对 angular_bias 的 雅可比
        Eigen::Matrix3d Jac_omega_bias_pre_ = - Skew(pixel_cord) * R_e_r.transpose() * Skew(t_r_e) * R_e_r;
        Eigen::RowVector3d Jac_omega_bias_post_ = normal_flow.transpose() * Skew(pixel_cord);
        Eigen::Matrix3d Jac_omega_bias_ = Jac_omega_bias_pre_ * post_vec + pre_vec * Jac_omega_bias_post_;
        // LOG(ERROR) << "Jac_omega_bias_pre_ = " << Output_M(Jac_omega_bias_pre_) << std::endl;
        // LOG(ERROR) << "Jac_omega_bias_post_ = " << Output(Jac_omega_bias_post_) << std::endl;
        // LOG(ERROR) << "Jac_omega_bias_ = " << Output_M(Jac_omega_bias_) << std::endl;
        Eigen::Matrix3d J_angular_bias_copy;
        if(jacobians[knot_num + 1])
        {
          // Eigen::Map<Eigen::Matrix<double, 1, 3>> J_linear_bias_(jacobians[2 * knot_num]); 
          Eigen::Map<Eigen::Matrix<double, 3, 3>> J_angular_bias_(jacobians[knot_num + 1]); 
          J_angular_bias_.setZero();

          J_angular_bias_ = (weight_ * 0.01 * Jac_omega_bias_).eval();

          // debug_ceres.debug_file << "J_angular_bias_ = " << Output_M(jac_angular_copy) << "]\n";
          J_angular_bias_copy = J_angular_bias_;

          // debug_ceres_jacobis.debug_file << Output_M(jac_angular_copy) << std::endl;
        }
        // LOG(ERROR) << "Add jacobians for Angular velocity bias " << std::endl;  
        */

        // LOG(ERROR) << "All Jacobi for BodyLocalAngularVelocity factor" << std::endl;
        // debug_ceres.debug_file << "All Jacobi for BodyLocalAngularVelocity factor" << std::endl;
        // debug_ceres.debug_file << "Jac_R_ = " << Output_M(Jac_R) << std::endl;
        // debug_ceres.debug_file << "Jac_p_ = " << Output_M(Jac_p) << std::endl;
        // debug_ceres.debug_file << "J_angular_bias_ = " << Output_M(Jac_w_bias) << std::endl;

        LOG(ERROR) << "Jac_w = " << Output_M(Jac_w) << std::endl;
        // LOG(ERROR) << "J_linear_bias_ = " << Output_M(J_linear_bias_copy) << std::endl;
        // LOG(ERROR) << "J_angular_bias_ = " << Output_M(J_angular_bias_copy) << std::endl;


        // debug_ceres.Close();
        // debug_ceres_jacobis.Close();

        return true;
    }
  
private:
    int64_t time_ns_;
    Eigen::Vector3d pixel_cord;
    Eigen::Vector3d flow_;
    Eigen::Vector3d doppler_velocity_;
    Eigen::Quaterniond q_e_r;
    Eigen::Vector3d t_e_r;
    SplineMeta<SplineOrder> spline_meta_;
    // Vec6d info_vec_;
    Eigen::Vector3d angular_bias;
    bool lock_extrincs;
    double weight_;
    // 某些参数的弱优化权重
    double w_weight_;
};


class EventAgularFactor3 : public ceres::CostFunction, SplitSpineView{
 public:
 EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  using SO3View = So3SplineView;
  using R3View = RdSplineView;
  using SplitView = SplitSpineView;

  using Vec3d = Eigen::Matrix<double, 3, 1>;
  using Vec6d = Eigen::Matrix<double, 6, 1>;
  using Mat3d = Eigen::Matrix<double, 3, 3>;
  using Quatd = Eigen::Quaterniond;
  using SO3d = Sophus::SO3<double>;

  EventAgularFactor3(int64_t time_ns, const Eigen::Vector3d pt, const event_flow_velocity flow, 
            const Eigen::Vector3d doppler_velocity,
            const Eigen::Quaterniond q_e_r, const Eigen::Vector3d t_e_r,
            const SplineMeta<SplineOrder>& spline_segment_meta,
            double weight, double w_weight = 0.05) // const Vec6d& info_vec)
      : time_ns_(time_ns),
        pixel_cord(pt), doppler_velocity_(doppler_velocity),
        q_e_r(q_e_r), t_e_r(t_e_r),
        spline_meta_(spline_segment_meta),
        lock_extrincs(true),      // HAO TODO: 暂时不优化外参
        weight_(weight), w_weight_(w_weight)
        {
          // set_num_residuals(1);           // 定义残差值的大小(事件角速度残差)
          set_num_residuals(3);           // 定义残差值的大小(事件角速度残差)

          size_t knot_num = this->spline_meta_.NumParameters();
          // LOG(ERROR) << "knot_num = " << knot_num << std::endl;
          flow_ << flow.x, flow.y, 0.0;
          // TODO: 需要角速度曲线              
          for (size_t i = 0; i < knot_num; ++i) {
            mutable_parameter_block_sizes()->push_back(3);   // angular velocity
          }

          // mutable_parameter_block_sizes()->push_back(3);    // linear bias // 1 -11 修改
          
          // mutable_parameter_block_sizes()->push_back(3);    // angular bias // 1 -11 修改
        }

  virtual bool Evaluate(double const* const* parameters, double* residuals,
                          double** jacobians) const {
        typename R3View::JacobianStruct J_w_;

        LOG(ERROR) << "Evaluate EventAgularFactor3 " << std::endl;
        // debug_ceres.Open();
        // debug_ceres.debug_file << std::endl;
        // debug_ceres.debug_file << "    ---------------------- Evaluate BodyLocalAngularVelocityFactor ------------------    " << std::endl;

        size_t knot_num = spline_meta_.NumParameters();

        // double time_offset_in_ns = parameters[t_offset_index][0];
        double time_offset_in_ns = 0;
        int64_t t_corrected = time_ns_ + (int64_t)time_offset_in_ns;
        t_corrected = std::min(t_corrected, spline_meta_.segments.at(0).MaxTimeNs() - 1);
        t_corrected = std::max(t_corrected, spline_meta_.segments.at(0).MinTimeNs() + 1);
  
        Eigen::Vector3d gyro_, rot_accel;
        if (jacobians){
          // S_ItoG = SO3View::EvaluateRp(t_corrected, spline_meta_.segments.at(0),
          //                 parameters, &J_R);

          // gyro_ = R3View::velocity(t_corrected,
          //                               spline_meta_.segments.at(0),
          //                               parameters, &J_w_);   

          gyro_ = R3View::evaluate(t_corrected,
                                        spline_meta_.segments.at(0),
                                        parameters, &J_w_);   

          // rot_accel = SO3View::accelerationBody(t_corrected,  
          //                                       spline_meta_.segments.at(0), 
          //                                       parameters);

          // v_inG = R3View::velocity(t_corrected,
          //                               spline_meta_.segments.at(0),
          //                               parameters + knot_num, &J_v);   
        }else{
          // S_ItoG = SO3View::EvaluateRp(t_corrected, spline_meta_.segments.at(0),
          //                 parameters, nullptr);

          // gyro_ = R3View::velocity(t_corrected,
          //                     spline_meta_.segments.at(0),
          //                     parameters, nullptr);  
                                
          gyro_ = R3View::evaluate(t_corrected,
                                        spline_meta_.segments.at(0),
                                        parameters, nullptr);  

          // rot_accel = SO3View::accelerationBody(t_corrected,  
          //                               spline_meta_.segments.at(0), 
          //                               parameters);
        
          // v_inG = R3View::velocity(t_corrected,
          //                       spline_meta_.segments.at(0),
          //                       parameters + knot_num, nullptr);   
        }
        // LOG(ERROR) << "Set Jacobi Matrix " << std::endl;



        // debug_ceres.debug_file << "Event Details: weight_ = " << weight_ 
                                      // << "\n w_weight_ = [" <<  w_weight_
                                      // // << "\n S_ItoG = [" <<  Output_M(S_ItoG)
                                      // << "\n local_angular_vel_= [" <<  Output(local_angular_vel_) << "] "
                                      // << "\n gyro_ = [" <<  Output(gyro_.transpose()) << "] "
                                      // << "\n angular_bias = [" <<  Output(angular_bias) << "] " << std::endl;

        // LOG(ERROR) << "Event Details: weight_ = " << weight_ 
        //                               << "\n w_weight_ = [" <<  w_weight_
        //                               // << "\n S_ItoG = [" <<  Output_M(S_ItoG)
        //                               << "\n local_angular_vel_= [" <<  Output(local_angular_vel_) << "] "
        //                               << "\n gyro_ = [" <<  Output(gyro_.transpose()) << "] "
        //                               << "\n angular_bias = [" <<  Output(angular_bias) << "] " << std::endl;

      // 3-18 修改 法向光流的计算
      Eigen::Vector3d grad;
      grad << -1.0 / flow_(0), -1.0 / flow_(1), 0.0;
      double normal_norm = 1.0 / grad.norm();
      Eigen::Vector3d normal_flow = grad * normal_norm;

      // Eigen::Map<Vec3d const> linear_bias(parameters[knot_num]);
      // Eigen::Map<Vec3d const> omega_bias(parameters[knot_num + 1]);

      Eigen::Matrix3d R_e_r = q_e_r.toRotationMatrix(); 
      Eigen::Vector3d t_r_e_temp = R_e_r.transpose() * t_e_r;
      Eigen::Vector3d t_r_e = -1.0 * t_r_e_temp;
      
      // 保留bias的优化
      // Eigen::Vector3d pre_vec = Skew(pixel_cord) * (R_e_r.transpose() * ((doppler_velocity_ + linear_bias) + (gyro_ + R_e_r * omega_bias).cross(t_r_e)));      
      // // use for transport
      // double post_vec = normal_norm + normal_flow.transpose() * Skew(pixel_cord) * (R_e_r.transpose() * gyro_ + omega_bias);

      // 去bias
      Eigen::Vector3d pre_vec = Skew(pixel_cord) * (R_e_r.transpose() * ((doppler_velocity_) + (gyro_).cross(t_r_e)));      
      pre_vec.normalize();
      double doppler_scale_aid = pre_vec.norm();
      // use for transport
      double post_vec = normal_norm + normal_flow.transpose() * Skew(pixel_cord) * (R_e_r.transpose() * gyro_);

      /*
      修改问题,非线性优化 -> 线性
      ref f(x) |-> log(1 + f(x))   f(x)|->0
      
      */


      LOG(ERROR) << "Event Details: weight_ = " << weight_ 
                   << "\n w_weight_ = " <<  w_weight_
                   << "\n pixel_cord = " <<  Output_M(pixel_cord)
                   << "\n flow_ = " <<  Output(flow_)
                   << "\n doppler_velocity_ = " <<  Output(doppler_velocity_)
                  //  << "\n linear_bias = " <<  Output(linear_bias)
                   << "\n R_e_r = " <<  Output_M(R_e_r)
                   << "\n t_r_e = " << Output(t_r_e.transpose()) 
                   << "\n gyro_ = " <<  Output(gyro_) 
                   << "\n normal_norm = " <<  normal_norm 
                   << "\n normal_flow = " <<  Output(normal_flow) 
                  //  << "\n omega_bias = " <<  Output(omega_bias) 
                   << std::endl;
      LOG(ERROR) << "pre_vec = " << Output(pre_vec)
                 << "\n post_vec = " << post_vec << std::endl;

      // LOG(ERROR) << "post_vec = " << post_vec << std::endl;

        Eigen::Map<Eigen::Vector3d> residual(residuals);
        residual.setZero();
        Eigen::Vector3d residual_temp = pre_vec * post_vec;
        residual = residual_temp;
        // residual = (weight_ * w_weight_ * residual).eval();
        residual = (w_weight_ * residual).eval();

        // double residual = post_vec;
        // residuals[0] = residual;


        // LOG(ERROR) << "local angular velocity residuals = " 
        //                         << (residual) << std::endl;

        LOG(ERROR) << "local angular velocity residuals = " 
                                << Output_M(residual) << std::endl;

        // 不评估雅可比就返回
        if (!jacobians) {
          // LOG(ERROR) << "BodyLocalAngularVelocityFactor No J" << std::endl;
          // debug_ceres.debug_file << "BodyLocalAngularVelocityFactor No Jacobi!" << std::endl;
          // debug_ceres.Close();

          // // debug_ceres_jacobis.Close();
          return true;
        }
        // debug_ceres_jacobis.Open();
        // debug_ceres_jacobis.debug_file << "Evaluate EventAgularFactor Res J_R_0 J_R_1 J_R_2 J_R_3 J_p_0 J_p_1 J_p_2 J_p_3 J_linear_bias J_angular_bias J_t \n";
        // debug_ceres_jacobis.debug_file << Output(residual_temp.transposacobi!e()) << std::endl;
 
        // [1] 对 kont_w 的 雅可比
        Eigen::Matrix3d Jac_gyro_pre = - Skew(pixel_cord) * R_e_r.transpose() * Skew(t_r_e);
        // // 前一项固定,后一项优化
        // Eigen::Matrix3d Jac_gyro_pre;
        // Jac_gyro_pre.setZero();
        Eigen::RowVector3d Jac_gyro_post = normal_flow.transpose() * Skew(pixel_cord) * R_e_r.transpose();
        // LOG(ERROR) << "Jac_gyro_pre = " << Output_M(Jac_gyro_pre) << std::endl;
        // LOG(ERROR) << "Jac_gyro_post = " << Output_M(Jac_gyro_post) << std::endl;

        // 只要最后一项的优化
        // Eigen::Matrix<double, 4, 3> Jac_w;
        // for (size_t i = 0; i < knot_num; i++) {
        //   size_t idx = i;
        //   if (jacobians[idx]) {
        //     Eigen::Map<Eigen::RowVector3d> jac_kont_w(
        //         jacobians[idx]);
        //     jac_kont_w.setZero();

        //     // jac_kont_p = S_ItoG.matrix().transpose() * J_v.d_val_d_knot[i];
        //     // jac_kont_p = (weight * jac_kont_p).eval();
        //     Eigen::RowVector3d Jac_gyro_ = Jac_gyro_post * J_w_.d_val_d_knot[i];
        //     jac_kont_w = (weight_ * Jac_gyro_).eval();

        //     Eigen::RowVector3d jac_kont_w_copy = jac_kont_w;

        //     // LOG(ERROR) << "J_w_.d_val_d_knot[" << i << "] = [" << J_w_.d_val_d_knot[i] << "]\n";
        //     // // debug_ceres.debug_file << "J_v.d_val_d_knot[" << i << "] = [" << Output_M(J_v.d_val_d_knot[i]) << "]\n";
        //     // // debug_ceres.debug_file << "jac_lhs_R_" << i << " = [" << Output_M(jac_kont_p_copy) << "]\n";

        //     Jac_w.block<1,3>(i, 0) = jac_kont_w_copy;

        //     // debug_ceres_jacobis.debug_file << Output_M(jac_kont_p)  << std::endl;

        //   }
        // }
        // LOG(ERROR) << "Add jacobians for Position control point " << std::endl;  

        Eigen::Matrix<double, 12, 3> Jac_w;
        for (size_t i = 0; i < knot_num; i++) {
          size_t idx = i;
          if (jacobians[idx]) {
            Eigen::Map<Eigen::Matrix3d> jac_kont_w(
                jacobians[idx]);
            jac_kont_w.setZero();

            // jac_kont_p = S_ItoG.matrix().transpose() * J_v.d_val_d_knot[i];
            // jac_kont_p = (weight * jac_kont_p).eval();
            Eigen::Matrix3d Jac_gyro_ = Jac_gyro_pre * J_w_.d_val_d_knot[i] * post_vec 
                                          + pre_vec * Jac_gyro_post * J_w_.d_val_d_knot[i];
            jac_kont_w = (weight_ * Jac_gyro_).eval();

            Eigen::Matrix3d jac_kont_w_copy = jac_kont_w;

            // LOG(ERROR) << "J_w_.d_val_d_knot[" << i << "] = [" << J_w_.d_val_d_knot[i] << "]\n";
            // // debug_ceres.debug_file << "J_v.d_val_d_knot[" << i << "] = [" << Output_M(J_v.d_val_d_knot[i]) << "]\n";
            // // debug_ceres.debug_file << "jac_lhs_R_" << i << " = [" << Output_M(jac_kont_p_copy) << "]\n";

            Jac_w.block<3,3>(3 * i, 0) = jac_kont_w_copy;

            // debug_ceres_jacobis.debug_file << Output_M(jac_kont_p)  << std::endl;

          }
        }

        /*
        // [2] 对 linear_bias 的 雅可比
        Eigen::Matrix3d Jac_linear_bias_pre_ = Skew(pixel_cord) * R_e_r.transpose();
        Eigen::Matrix3d Jac_linear_bias_ = Jac_linear_bias_pre_ * post_vec;
        // LOG(ERROR) << "Jac_linear_bias_pre_ = " << Output_M(Jac_linear_bias_pre_) << std::endl;
        // LOG(ERROR) << "Jac_linear_bias_ = " << Output_M(Jac_linear_bias_) << std::endl;
        Eigen::Matrix3d J_linear_bias_copy;
        if(jacobians[knot_num])
        {
          // Eigen::Map<Eigen::Matrix<double, 1, 3>> J_linear_bias_(jacobians[2 * knot_num]); 
          Eigen::Map<Eigen::Matrix<double, 3, 3>> J_linear_bias_(jacobians[knot_num]); 
          J_linear_bias_.setZero();

          J_linear_bias_ = (weight_ * 0.01 * Jac_linear_bias_).eval();

          J_linear_bias_copy = J_linear_bias_;

          // debug_ceres.debug_file << "J_angular_bias_ = " << Output_M(jac_angular_copy) << "]\n";


          // debug_ceres_jacobis.debug_file << Output_M(jac_angular_copy) << std::endl;
        }

        // [3] 对 angular_bias 的 雅可比
        Eigen::Matrix3d Jac_omega_bias_pre_ = - Skew(pixel_cord) * R_e_r.transpose() * Skew(t_r_e) * R_e_r;
        Eigen::RowVector3d Jac_omega_bias_post_ = normal_flow.transpose() * Skew(pixel_cord);
        Eigen::Matrix3d Jac_omega_bias_ = Jac_omega_bias_pre_ * post_vec + pre_vec * Jac_omega_bias_post_;
        // LOG(ERROR) << "Jac_omega_bias_pre_ = " << Output_M(Jac_omega_bias_pre_) << std::endl;
        // LOG(ERROR) << "Jac_omega_bias_post_ = " << Output(Jac_omega_bias_post_) << std::endl;
        // LOG(ERROR) << "Jac_omega_bias_ = " << Output_M(Jac_omega_bias_) << std::endl;
        Eigen::Matrix3d J_angular_bias_copy;
        if(jacobians[knot_num + 1])
        {
          // Eigen::Map<Eigen::Matrix<double, 1, 3>> J_linear_bias_(jacobians[2 * knot_num]); 
          Eigen::Map<Eigen::Matrix<double, 3, 3>> J_angular_bias_(jacobians[knot_num + 1]); 
          J_angular_bias_.setZero();

          J_angular_bias_ = (weight_ * 0.01 * Jac_omega_bias_).eval();

          // debug_ceres.debug_file << "J_angular_bias_ = " << Output_M(jac_angular_copy) << "]\n";
          J_angular_bias_copy = J_angular_bias_;

          // debug_ceres_jacobis.debug_file << Output_M(jac_angular_copy) << std::endl;
        }
        // LOG(ERROR) << "Add jacobians for Angular velocity bias " << std::endl;  
        */

        // LOG(ERROR) << "All Jacobi for BodyLocalAngularVelocity factor" << std::endl;
        // debug_ceres.debug_file << "All Jacobi for BodyLocalAngularVelocity factor" << std::endl;
        // debug_ceres.debug_file << "Jac_R_ = " << Output_M(Jac_R) << std::endl;
        // debug_ceres.debug_file << "Jac_p_ = " << Output_M(Jac_p) << std::endl;
        // debug_ceres.debug_file << "J_angular_bias_ = " << Output_M(Jac_w_bias) << std::endl;

        LOG(ERROR) << "Jac_w = " << Output_M(Jac_w) << std::endl;
        // LOG(ERROR) << "J_linear_bias_ = " << Output_M(J_linear_bias_copy) << std::endl;
        // LOG(ERROR) << "J_angular_bias_ = " << Output_M(J_angular_bias_copy) << std::endl;


        // debug_ceres.Close();
        // debug_ceres_jacobis.Close();

        return true;
    }
  
private:
    int64_t time_ns_;
    Eigen::Vector3d pixel_cord;
    Eigen::Vector3d flow_;
    Eigen::Vector3d doppler_velocity_;
    Eigen::Quaterniond q_e_r;
    Eigen::Vector3d t_e_r;
    SplineMeta<SplineOrder> spline_meta_;
    // Vec6d info_vec_;
    Eigen::Vector3d angular_bias;
    bool lock_extrincs;
    double weight_;
    // 某些参数的弱优化权重
    double w_weight_;
};

class BodyLocalVelocityFactor2 : public ceres::CostFunction, SplitSpineView{
 public:
 EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  using SO3View = So3SplineView;
  using R3View = RdSplineView;
  using SplitView = SplitSpineView;

  using Vec3d = Eigen::Matrix<double, 3, 1>;
  using Vec6d = Eigen::Matrix<double, 6, 1>;
  using Mat3d = Eigen::Matrix<double, 3, 3>;
  using Quatd = Eigen::Quaterniond;
  using SO3d = Sophus::SO3<double>;

  BodyLocalVelocityFactor2(int64_t time_ns, const Eigen::Vector3d local_vel,
            // const Eigen::Vector3d linear_bias,
            const SplineMeta<SplineOrder>& spline_segment_meta,
            double weight, double w_weight = 0.05, double R_weight = 0.05) // const Vec6d& info_vec)
      : time_ns_(time_ns),local_vel_(local_vel),
        // linear_bias_(linear_bias),
        spline_meta_(spline_segment_meta),
        weight_(weight), w_weight_(w_weight), R_weight_(R_weight)
        {
          set_num_residuals(3);           // 定义残差值的大小(事件角速度残差)

          size_t knot_num = this->spline_meta_.NumParameters();
          // LOG(ERROR) << "knot_num = " << knot_num << std::endl;

          // // TODO: 需要提供采样曲线的多普勒速度和偏执                   
          // for (size_t i = 0; i < knot_num; ++i) {             
          //   mutable_parameter_block_sizes()->push_back(4);   // rotation
          // }
          for (size_t i = 0; i < knot_num; ++i) {
            mutable_parameter_block_sizes()->push_back(3);   // position
          }
          mutable_parameter_block_sizes()->push_back(3);    // linear bias // 1 -11 修改
          // LOG(ERROR) << "param.size = " << mutable_parameter_block_sizes()->size() << std::endl;
        }

  virtual bool Evaluate(double const* const* parameters, double* residuals,
                          double** jacobians) const {
        typename SO3View::JacobianStruct J_R;
        typename SO3View::JacobianStruct J_w;
        typename R3View::JacobianStruct J_v;

        LOG(ERROR) << "Evaluate BodyLocalVelocityFactor2 " << std::endl;
        // debug_ceres.Open();
        // debug_ceres.debug_file << std::-endl;
        // debug_ceres.debug_file << "    --------------------- Evaluate BodyLocalVelocityFactor ------------------    " << std::endl;

        size_t knot_num = spline_meta_.NumParameters();
        // LOG(ERROR) << "get knot_num = " << knot_num << std::endl;
        // for(int i = 0; i <= knot_num; i++)
        // {
        //   for(int j = 0; j < 3; j++)
        //   {
        //     LOG(ERROR) << "parameters[" << i << "][" << j << "] = " << parameters[i][j] << std::endl;
        //   }
        // }
        // Eigen::Map<Eigen::Vector3d const> linear_bias(parameters[knot_num]);
        // Eigen::Map<Vec3d const> linear_bias(parameters[knot_num]);

        Eigen::Vector3d linear_bias(parameters[knot_num][0], parameters[knot_num][1], parameters[knot_num][2]);

        // Eigen::Vector3d linear_bias
        // LOG(ERROR) << "linear_bias = [" <<  Output(linear_bias) << "] " << std::endl;

        double time_offset_in_ns = 0; // parameters[t_offset_index][0];
        int64_t t_corrected = time_ns_ + (int64_t)time_offset_in_ns;
        // t_corrected = std::min(t_corrected, spline_meta_.segments.at(0).MaxTimeNs() - 1);
        // t_corrected = std::max(t_corrected, spline_meta_.segments.at(0).MinTimeNs() + 1);

        Eigen::Vector3d gyro_, rot_accel, v_inG;
        SO3d S_ItoG;
        if (jacobians){
          // S_ItoG = SO3View::EvaluateRp(t_corrected, spline_meta_.segments.at(0),
          //                 parameters, &J_R);

          // gyro_ = SO3View::VelocityBody(t_corrected,
          //                                     spline_meta_.segments.at(0),
          //                                     parameters, &J_w);

          // rot_accel = SO3View::accelerationBody(t_corrected,  
          //                                       spline_meta_.segments.at(0), 
          //                                       parameters);

          v_inG = R3View::evaluate(t_corrected,
                                        spline_meta_.segments.at(0),
                                        parameters, &J_v);   

          // v_inG = R3View::velocity(t_corrected,
          //                               spline_meta_.segments.at(0),
          //                               parameters, &J_v);   
        }else{
          // S_ItoG = SO3View::EvaluateRp(t_corrected, spline_meta_.segments.at(0),
          //                 parameters, nullptr);
                                
          // gyro_ = SO3View::VelocityBody(t_corrected,
          //                               spline_meta_.segments.at(0),
          //                               parameters, nullptr);

          // rot_accel = SO3View::accelerationBody(t_corrected,  
          //                               spline_meta_.segments.at(0), 
          //                               parameters);
        
          v_inG = R3View::evaluate(t_corrected,
                                spline_meta_.segments.at(0),
                                parameters, nullptr);   

          // v_inG = R3View::velocity(t_corrected,
          //                       spline_meta_.segments.at(0),
          //                       parameters, nullptr);             
        }
        // LOG(ERROR) << "Set Jacobi Matrix " << std::endl;

        // debug_ceres.debug_file << "BodyLocalVelocity Details: weight_ = " << weight_ 
                                      // << "\n w_weight_ = [" <<  w_weight_ << "] "
                                      // << "\n S_ItoG = [" <<  Output_M(S_ItoG.matrix()) << "] "
                                      // << "\n local_vel_= [" <<  Output(local_vel_) << "] "
                                      // << "\n v_inG = [" <<  Output(v_inG.transpose()) << "] "
                                      // << "\n linear_bias = [" <<  Output(linear_bias) << "] " << std::endl;

        LOG(ERROR) << "BodyLocalVelocity Details: weight_ = " << weight_ 
                                      << "\n w_weight_ = [" <<  w_weight_ << "] "
                                      // << "\n S_ItoG = [" <<  Output_M(S_ItoG.matrix()) << "] "
                                      << "\n local_vel_= [" <<  Output(local_vel_) << "] "
                                      << "\n v_inG = [" <<  Output(v_inG.transpose()) << "] " // << std::endl;
                                      << "\n linear_bias = [" <<  Output(linear_bias) << "] " << std::endl;

        // Eigen::Matrix3d R_ItoG = S_ItoG.matrix();   // 提取旋转矩阵

        // Eigen::Map<Eigen::Matrix3d> R_GtoI_map(R_ItoG.data());
        // Eigen::Matrix3d R_GtoI = R_GtoI_map.transpose();

        Eigen::Map<Eigen::Vector3d> residual(residuals);
        residual.setZero();
        // Eigen::Vector3d residual_temp = R_GtoI * v_inG - local_vel_ - linear_bias;
        // Eigen::Vector3d residual_temp = v_inG - S_ItoG * local_vel_ - S_ItoG * linear_bias;
                      // S_ItoG.matrix().transpose() * v_inG - local_vel_ - linear_bias;



        // LOG(ERROR) << "R_GtoI = " << Output_M(R_GtoI) << std::endl;
        // HAO TODO: 定义在机体上
        // Eigen::Vector3d residual_temp = v_inG - local_vel_ - 0 * linear_bias;
        Eigen::Vector3d residual_temp = v_inG - local_vel_ - linear_bias;

        // residual = residual_temp;
        // residual = (w_weight_ * weight_ * residual).eval();
        residual = (w_weight_ * weight_ * residual_temp).eval();
        // debug_ceres.debug_file << "local velocity residuals = " << Output(residual) << std::endl;

        LOG(ERROR) << "local velocity residuals = " << Output(residual) << std::endl;

        // // 技术问题,Eigen无法赋值,手动处理
        // residual[0] = residual(0);
        // residual[1] = residual(1);
        // residual[2] = residual(2);

        // debug_ceres.debug_file << "local velocity origin = " << residuals[0] << ", " << residuals[1] << ", " << residuals[2] << std::endl;
        // LOG(ERROR) << "residuals done" << std::endl;

        // 不评估雅可比就返回
        if (!jacobians) {
          LOG(ERROR) << "BodyLocalVelocityFactor No Jacobi!" << std::endl;
          // debug_ceres.debug_file << "BodyLocalVelocityFactor No Jacobi!" << std::endl;
          // debug_ceres.Close();

          // // debug_ceres_jacobis.Close();
          return true;
        }
        // debug_ceres_jacobis.Open();
        // debug_ceres_jacobis.debug_file << "Evaluate BodyLocalVelocityFactor Res J_R_0 J_R_1 J_R_2 J_R_3 J_p_0 J_p_1 J_p_2 J_p_3 J_linear_bias J_angular_bias J_t \n";
        // debug_ceres_jacobis.debug_file << Output(residual_temp.transpose()) << std::endl;
        

        // [1] 对 kont_R 的 雅可比
        Eigen::Matrix3d jac_lhs_R;
        // jac_lhs_R = - S_ItoG.matrix().transpose() * Skew(v_inG);


        // debug_ceres.debug_file << "R_GtoI = " << Output_M(R_GtoI) << std::endl;

        // Eigen::Matrix3d R_GtoI = R_ItoG.transpose().eval();
        // Eigen::Matrix3d skew_v = Skew(v_inG);

        // jac_lhs_R = -(R_GtoI * skew_v);
        // LOG(ERROR) << "jac_lhs_R = " << jac_lhs_R << std::endl;

        // jac_lhs_R = (- R_ItoG.transpose() * skew_v).eval();

        /*Eigen::Matrix<double, 12, 4> Jac_R;
        for (size_t i = 0; i < knot_num; i++) {
          size_t idx = i;
          if (jacobians[idx]) {
            Eigen::Map<Eigen::Matrix<double, 3, 4, Eigen::RowMajor>> jac_kont_R(
                jacobians[idx]);
            jac_kont_R.setZero();

            jac_kont_R.block<3,3>(0,0) += (jac_lhs_R * J_R.d_val_d_knot[i]);
            jac_kont_R = (R_weight_ * jac_kont_R).eval(); // weight_ * 

            Eigen::Matrix<double, 3, 4> jac_kont_R_copy = jac_kont_R;
            // debug_ceres.debug_file << "jac_lhs_R = [" << Output_M(jac_lhs_R) << "]\n";
            // debug_ceres.debug_file << "J_R.d_val_d_knot[" << i << "] = [" << Output_M(J_R.d_val_d_knot[i]) << "]\n";
            // debug_ceres.debug_file << "jac_lhs_R_" << i << " = [" << Output_M(jac_kont_R_copy) << "]\n";

            Jac_R.block<3,4>(3 * i, 0) = jac_kont_R_copy;

            // debug_ceres_jacobis.debug_file <<  Output_M(jac_kont_R_copy) << std::endl;

          }
        }*/
        // LOG(ERROR) << "Add jacobians for Rotation control point " << std::endl;   

        // [2] 对 kont_p 的 雅可比
        Eigen::Matrix<double, 12, 3> Jac_p;
        // Eigen::Matrix3d Jac_p_temp;
 
        // // debug_ceres.debug_file << "R_ItoG = [" << Output_M(R_ItoG) << "]\n";
        // Jac_p_temp = R_ItoG.transpose();
        // // debug_ceres.debug_file << "R_ItoG.transpose() = [" << Output_M(Jac_p_temp) << "]\n";
        for (size_t i = 0; i < knot_num; i++) {
          size_t idx = i;
          if (jacobians[idx]) {
            Eigen::Map<Eigen::Matrix3d> jac_kont_p(
                jacobians[idx]);
            jac_kont_p.setZero();

            // jac_kont_p += (Jac_p_temp * J_v.d_val_d_knot[i]);
            // LOG(ERROR) << "J_v.d_val_d_knot["<< i << "] = " << J_v.d_val_d_knot[i] << std::endl;
            // Eigen::Matrix3d Jac_p_temp = R_GtoI * J_v.d_val_d_knot[i];

            // Eigen::Matrix3d Jac_p_temp;
            // Jac_p_temp.setIdentity();
            // HAO TODO: 定义在机体系
            Eigen::Matrix3d Jac_p_temp = Eigen::Matrix3d::Identity() * J_v.d_val_d_knot[i]; // = (R_GtoI * J_v.d_val_d_knot[i]);
            // Jac_p_temp.setIdentity();

            jac_kont_p = (w_weight_ * Jac_p_temp).eval();

            Eigen::Matrix3d jac_kont_p_copy = jac_kont_p;
            // debug_ceres.debug_file << "J_v.d_val_d_knot[" << i << "] = [" << J_v.d_val_d_knot[i]<< "]\n";
            // // debug_ceres.debug_file << "Jac_p_temp = [" << Output_M(Jac_p_temp) << "]\n";

            // debug_ceres.debug_file << "jac_kont_p_copy = [" << Output_M(jac_kont_p_copy) << "]\n";

            // // debug_ceres_jacobis.debug_file << Output_M(jac_kont_p_copy) << " ";

            Jac_p.block<3,3>(3 * i, 0) = jac_kont_p_copy;

            // debug_ceres_jacobis.debug_file <<  Output_M(jac_kont_p_copy) << std::endl;

          }
          // else
          // {
          //   LOG(ERROR) << "missing jacobians [ " << idx << " ]" << std::endl;
          // }
        }
        // LOG(ERROR) << "Add jacobians for Position control point " << std::endl;   


        // [3] 对 linear_bias 的 雅可比
        Eigen::Matrix3d Jac_v_bias;
        Eigen::Matrix3d jac_lhs_bias;
        jac_lhs_bias.setIdentity();
        if(jacobians[knot_num])
        {
          // Eigen::Map<Eigen::Matrix<double, 1, 3>> J_linear_bias_(jacobians[2 * knot_num]); 
          Eigen::Map<Eigen::Matrix<double, 3, 3>> J_linear_bias_(jacobians[knot_num]); 
          J_linear_bias_.setZero();

          // J_linear_bias_.setIdentity();

          // J_linear_bias_ = (-1.0 * weight_ * J_linear_bias_).eval();

          J_linear_bias_ = (-1.0 * w_weight_ * jac_lhs_bias).eval();
          // J_linear_bias_ = (-0.0 * weight_ * jac_lhs_bias).eval();

          // J_linear_bias_ = (-1.0 * weight_ * S_ItoG.matrix() * jac_lhs_bias).eval();
            
          Eigen::Matrix3d jac_linear_copy = J_linear_bias_;

          // debug_ceres.debug_file << "J_linear_bias_ = " << Output_M(jac_linear_copy) << "]\n";
        
          Jac_v_bias = jac_linear_copy;
          // debug_ceres_jacobis.debug_file << Output_M(Jac_v_bias) << std::endl;
        }
        
        // LOG(ERROR) << "Add jacobians for Linear velocity bias " << std::endl;  
    
        LOG(ERROR) << "All Jacobi for BodyLocalVelocity factor" << std::endl;
        // debug_ceres.debug_file << "All Jacobi for Event factor" << std::endl;
        // debug_ceres.debug_file << "Jac_R_ = " << Output_M(Jac_R) << std::endl;
        // debug_ceres.debug_file << "Jac_p_ = " << Output_M(Jac_p) << std::endl;
        // debug_ceres.debug_file << "Jac_linear_bias_ = " << Output_M(Jac_v_bias) << std::endl;

        // LOG(ERROR) << "Jac_R_ = " << Output_M(Jac_R) << std::endl;

        LOG(ERROR) << "Jac_p_ = " << Output_M(Jac_p) << std::endl;
        // LOG(ERROR) << "Jac_linear_bias_ = " << Output_M(Jac_v_bias) << std::endl;

        // debug_ceres.Close();
        // debug_ceres_jacobis.Close();
        return true;
    }

private:
    int64_t time_ns_;
    SplineMeta<SplineOrder> spline_meta_;
    Eigen::Vector3d local_vel_;
    // Eigen::Vector3d linear_bias_;
    double weight_;
    double w_weight_;
    double R_weight_;
};


// HAO TODO: 以雷达的时间为准
class NewDopplerFactor : public ceres::CostFunction, SplitSpineView{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  // using SO3View = So3SplineView;
  using R3View = RdSplineView;
  // using SplitView = SplitSpineView;

  using Vec3d = Eigen::Matrix<double, 3, 1>;
  using Vec6d = Eigen::Matrix<double, 6, 1>;
  using Mat3d = Eigen::Matrix<double, 3, 3>;
  // using SO3d = Sophus::SO3<double>;

  NewDopplerFactor(int64_t time_ns, const Eigen::Vector3d& pt, 
            const double& doppler, 
            const SplineMeta<SplineOrder>& spline_segment_meta, // FEJ_STATE* global_fej_state_, no nned for fej
            std::shared_ptr<FEJ_STATE> global_fej_state, bool use_fej,
            double weight, double w_weight = 0.05)// const Vec6d& info_vec)
      : time_ns_(time_ns),
        pt_(pt), doppler_(doppler),
        linear_spline_meta_(spline_segment_meta),
        global_fej_state_(global_fej_state),
        use_fej_(use_fej),
        weight_(weight), w_weight_(w_weight)
        // info_vec_(info_vec) 
        {
          set_num_residuals(1);           // 定义残差值的大小(doppler速度残差)

          size_t knot_num = linear_spline_meta_.NumParameters();

          // // TODO: 需要提供采样曲线的多普勒速度和偏执
          // for (size_t i = 0; i < knot_num; ++i) {             
          //   mutable_parameter_block_sizes()->push_back(4);    // HAO TODO:
          // }
          for (size_t i = 0; i < knot_num; ++i) {
            mutable_parameter_block_sizes()->push_back(3);   // linear splines
          }

          mutable_parameter_block_sizes()->push_back(3);  // linear bias

          // mutable_parameter_block_sizes()->push_back(3);  // angular bias

          // mutable_parameter_block_sizes()->push_back(1);  // time_offset 雷达时间作为系统时间,不存在时间偏移
        }

   virtual bool Evaluate(double const* const* parameters, double* residuals,
                        double** jacobians) const {
    // typename SO3View::JacobianStruct J_R;
    // typename SO3View::JacobianStruct J_rot_a;
    typename R3View::JacobianStruct J_v;

    std::chrono::time_point<std::chrono::high_resolution_clock> time1;

    LOG(ERROR) << "Evaluate NewDopplerFactor " << std::endl;

    // 解析状态参数 parameters
    int knot_num = this->linear_spline_meta_.NumParameters();

    int64_t t_corrected = time_ns_; //  + (int64_t)time_offset_in_ns;


    // 估计的速度
    Eigen::Vector3d v_inG;
    SO3d S_ItoG;
    if (jacobians){
      // LOG(ERROR) << std::setprecision(20) << " t_corrected = " << t_corrected << std::endl;

      // HAO TODO:
      // S_ItoG = SO3View::EvaluateRp(t_corrected, spline_meta_.segments.at(0),
      //                         parameters, &J_R);

      // LOG(ERROR) << "first J_R.start_idx = " <<  J_R.start_idx << std::endl;

      // v_inG = R3View::velocity(t_corrected,
      //                         spline_meta_.segments.at(0),
      //                         parameters + knot_num, &J_v);

      v_inG = R3View::evaluate(t_corrected,
                              linear_spline_meta_.segments.at(0),
                              parameters, &J_v);                          
    }else{
      // gyro_ = SO3View::VelocityBody(t_corrected,
      //                               spline_meta_.segments.at(0),
      //                               parameters + R_offset[0], nullptr);

      // HAO TODO:
      // S_ItoG = SO3View::EvaluateRp(t_corrected, spline_meta_.segments.at(0),
      //                         parameters, nullptr);
      // v_inG = R3View::velocity(t_corrected,
      //                         spline_meta_.segments.at(0),
      //                         parameters + knot_num, nullptr);
      v_inG = R3View::evaluate(t_corrected,
                        linear_spline_meta_.segments.at(0),
                        parameters, nullptr);    
    }

    // LOG(ERROR) << "Interpolation velocity = " << Output(v_inG.transpose()) << std::endl;

    // LOG(ERROR) << "Pointer to linear_bias: " << parameters[2 * knot_num] << std::endl;
    // Eigen::Map<Vec3d const> linear_bias(parameters[2 * knot_num]);          // 速度偏置
    Eigen::Map<Vec3d const> linear_bias(parameters[knot_num]);          // 速度偏置
    

    // residuals[0] = weight_ * (doppler_ - pt_.normalized().transpose() * S_ItoG.matrix().inverse() * (v_inG + linear_bias).eval());

    // Eigen::Vector3d corrected_velocity = v_inG + linear_bias;      // 注意这里的 linear_bias 也是定义在 
    // double doppler_actual = pt_.normalized().transpose() * S_ItoG.matrix().inverse() * corrected_velocity;
    // residuals[0] = doppler_ - doppler_actual;
    // residuals[0] *= weight_;

    // 1-10 TODO: 修正 偏置应该在各自的传感器上 linear_bias 应该在radar系上
    // Eigen::Vector3d corrected_velocity = v_inG;      // 注意这里的 linear_bias 也是定义在 
    // double doppler_actual = pt_.normalized().transpose() * (S_ItoG.matrix().inverse() * v_inG + linear_bias); // 这里的I定位为雷达系,S_ItoG.matrix().inverse() * v_inG 则是雷达系的参考速度
    // residuals[0] = doppler_ - doppler_actual;
    // residuals[0] *= weight_;

    // 1-24 check:
    // double doppler_actual = pt_.normalized().transpose() * (S_ItoG.matrix().inverse() * v_inG + linear_bias); // 这里的I定位为雷达系,S_ItoG.matrix().inverse() * v_inG 则是雷达系的参考速度
    // residuals[0] = doppler_ - doppler_actual;

    // residuals[0] = (weight_ * (doppler_ - pt_.normalized().transpose() * (S_ItoG.matrix().transpose() * v_inG + linear_bias)));    
    // Eigen::Matrix3d temp = S_ItoG.matrix();
    // Eigen::Map<Eigen::Matrix3d> S_GtoI_map(temp.data());
    // Eigen::Matrix3d S_GtoI = S_GtoI_map.transpose();

    // Eigen::Vector3d v_inI_compen = (S_GtoI * v_inG + linear_bias);

    // HAO TODO: 4-6 修改
    // Eigen::Vector3d v_inI_compen = v_inG + linear_bias;
    // LOG(ERROR) << "linear_bias = " << Output(linear_bias.transpose()) << std::endl;
    // LOG(ERROR) << "v_inG = " << Output(v_inG.transpose()) << std::endl;
    // LOG(ERROR) << "v_inI_compen = " << v_inI_compen << std::endl;
    // LOG(ERROR) << "doppler_ = " << doppler_ << std::endl;
    // LOG(ERROR) << "pt_.normalized().transpose() = " << Output(pt_.normalized().transpose()) << std::endl;
    // double temp_residuals = (- pt_.normalized().transpose() * v_inI_compen).eval();
    // double temp_residuals = (- pt_.normalized().transpose() * v_inI_compen).value();
    // double temp_residuals = (doppler_ - pt_.normalized().dot(v_inI_compen));

    // LOG(ERROR) << "temp_residuals = " << temp_residuals << std::endl;
    // residuals[0] = (doppler_ - pt_.normalized().dot(v_inI_compen));
    // LOG(ERROR) << "Doppler residuals = " << residuals[0] << "weight = " << weight_ << std::endl;
    // residuals[0] = weight_ * residuals[0];
    // LOG(ERROR) << "weight * Doppler residuals = " << residuals[0] << std::endl;

    std::chrono::time_point<std::chrono::high_resolution_clock> time2;


    // double residuals_temp = weight_ * (doppler_ - pt_.normalized().dot(global_fej_state_->linear_velocity_ 
    //                           + global_fej_state_->linear_bias_));
    double residuals_temp = weight_ * (doppler_ - pt_.normalized().dot(v_inG + linear_bias));
    residuals[0] = residuals_temp; // v_inG is in body

    // LOG(ERROR) << "Doppler residuals = " << residuals_temp << std::endl;

    if(global_fej_state_ && use_fej_)
    {
      residuals_temp = weight_ * (doppler_ - pt_.normalized().dot(global_fej_state_->linear_velocity_ 
                          + global_fej_state_->linear_bias_));

      LOG(ERROR) << "doppler evaluation use fej:\n"
                << "linear_velocity = " << Output(global_fej_state_->linear_velocity_) << "\n" 
                << "linear_bias = " << Output(global_fej_state_->linear_bias_) << "\n"  
                << std::endl;      
    }

    LOG(ERROR) << "Doppler residuals = " << residuals_temp << "\n"
               << "weight_ = " << weight_ << "\n"
               << "w_weight_ = " << w_weight_<< "\n"
               << "doppler_ = " << doppler_ << "\n"
               << "pt_.normalized() = " << Output(pt_.normalized()) << "\n"
               << "v_inG = " << Output(v_inG) << "\n" 
               << "linear_bias = " << Output(linear_bias) << "\n"  
               << std::endl;

    // LOG(ERROR) << "Doppler Loss: " << ((residuals_temp > 0.3)? "True": "False") << std::endl;          

    /*std::fstream linear_file("/home/hao/Desktop/twist_ws/src/TwistEstimator/output/linear_res.txt",
                              std::ios::out | std::ios::app);
    linear_file << residuals_temp << std::endl;
    linear_file.close();*/

    std::chrono::time_point<std::chrono::high_resolution_clock> time3;

    // debug_ceres.Open();
    // if(!clear_debug)
    // {
    //   clear_debug = true;
    //   debug_ceres.Close();

    //   // debug_ceres.debug_file.open(debug_ceres.file_path, std::ios::trunc);
    //   // // debug_ceres.debug_file.clear();
    //   // debug_ceres.debug_file.close();

    //   debug_ceres.Open();
    //   // debug_ceres.debug_file << "Start to Record \n";
    // }
    // debug_ceres.debug_file << std::endl;
    // debug_ceres.debug_file << "    ---------------------- Evaluate DopplerFactor ------------------    " << std::endl;
    // LOG(ERROR) << "open debug file: " << debug_ceres.file_path << std::endl;
    // debug_ceres.debug_file << "Doppler residuals = " << std::setprecision(20) << residuals[0] << std::endl;
    // LOG(ERROR) << "residuals done" << std::endl;

    // debug
    // Eigen::RowVector3d pt_row_vec = pt_.transpose();

    // Eigen::RowVector3d linear_bias_row_vec = linear_bias.transpose(); //.eval();

    // assert(parameters[2 * knot_num] != nullptr && "parameters[2 * knot_num] is nullptr");
    // // debug_ceres.debug_file << "Details: weight_ = " << std::setprecision(5) << weight_ 
    
    // debug_ceres.debug_file << "Doppler Details: weight_ = " <<  weight_ << std::endl;
    // debug_ceres.debug_file << "Doppler Details: w_weight_ = " <<  w_weight_ << std::endl;
    // debug_ceres.debug_file << "Doppler residuals = " << std::setprecision(20) <<  residuals[0];
    // // debug_ceres.debug_file << "\n pt_row_vec = [" <<  pt_row_vec << "] ";
    // debug_ceres.debug_file << "\n pt_row_vec = [" <<  Output(pt_row_vec) << "] ";
    // debug_ceres.debug_file << "\n pt_row_vec.norm = [" <<  Output(pt_row_vec.normalized()) << "] "
                                  // << "\nS_GtoI = [" << Output_M(S_GtoI) << "] "
                                  // << "\ndoppler_ = [" << doppler_ << "] "
                                  // << "\nS_ItoG = [" << Output_M(S_ItoG.matrix()) << "] "
                                  // << "\nv_inG = [" << Output(v_inG.transpose()) << "] " 
                                  // << "\nlinear_bias = [" << Output(linear_bias_row_vec) << "] " << std::endl;

    // 不评估雅可比就返回
    // if (!jacobians) {
    //   LOG(ERROR) << "Doppler No Jacobians " << std::endl;
    //   // debug_ceres.debug_file << "No Jacobians" << std::endl;
    //   debug_ceres.Close();
      
    //   return true;
    // }
    // else
    // {
    //   LOG(ERROR) << " Calculate Jacobians " << std::endl;
    // }

    if (!jacobians) {
      return true;
    }

    // debug_ceres_jacobis.Open();
    // debug_ceres_jacobis.debug_file << "Evaluate DopplerFactor Res J_R_0 J_R_1 J_R_2 J_R_3 J_p_0 J_p_1 J_p_2 J_p_3 J_linear_bias \n";
    // debug_ceres_jacobis.debug_file << residuals[0] << " ";
 
    // 位姿雅可比
    // Eigen::Matrix3d jac_lhs_R;
        // residuals[0] = 
    // (weight_ * (doppler_ - pt_.normalized().transpose() * (S_ItoG.matrix().transpose() * v_inG + linear_bias)));
    // Eigen::Vector3d jac_lhs_R = Skew(v_inG).transpose() * S_ItoG.matrix() * pt_.normalized();
    // jac_lhs_R = - weight_ * pt_.normalized().transpose() * S_ItoG.matrix().inverse() *  SO3::hat(v_inG + linear_bias);

    // jac_lhs_R =  - weight_ * (v_inG + linear_bias) * pt_.normalized().transpose();

    // jac_lhs_R =  - weight_ * (pt_.normalized().transpose().cross((v_inG + linear_bias))).transpose();
    // jac_lhs_R = - weight_ * (pt_.normalized().cross((v_inG + linear_bias)));

    // 1-10 TODO: 修正
    // 修正的残差是
    // pt_.normalized().transpose() * (S_ItoG.matrix().inverse() * v_inG + linear_bias)

    // jac_lhs_R = - weight_ * (pt_.normalized() *  S_ItoG.matrix().inverse() * v_inG);
    // jac_lhs_R = (pt_.normalized().transpose() *  S_ItoG.matrix().inverse() *  Skew(v_inG)).transpose(); // weight_ 在下面乘了
  
    // 1-12 修改
    // jac_lhs_R = (Skew(pt_.normalized()).transpose() * S_ItoG.matrix().transpose() * v_inG);
    // jac_lhs_R += ((pt_.normalized().transpose() * S_ItoG.matrix().inverse() * Skew(v_inG)).transpose()).eval();
    // jac_lhs_R += ((pt_.normalized().transpose() * S_ItoG.matrix().inverse() * Skew(v_inG)).transpose());

    // jac_lhs_R = (jac_lhs_R);
    // LOG(ERROR) << " jac_lhs_R = " << Output(jac_lhs_R) << std::endl;
    // debug_ceres.debug_file << "jac_lhs_R = " << Output(jac_lhs_R) << std::endl;
    // // debug_ceres.debug_file << "details = [" << jac_lhs_R.transpose() << "]\n";

    // 速度雅可比 J_v_d_
    // Eigen::Vector3d J_v_d_ = - S_ItoG.matrix().inverse() * pt_.normalized();
    // 1-10 修改
    // Eigen::Vector3d J_v_d_ = - S_ItoG.matrix().inverse() * pt_.normalized();  // use with transpose()

    // 1-24 check:


    // residual = doppler_ - pt_.normalized().transpose() * (S_ItoG.matrix().inverse() * v_inG + linear_bias)
    // - pt_.normalized().transpose() * S_ItoG.transpose() * v_inG / S_ItoG
    // pt_.normalized().transpose() * (S_ItoG.transpose() - (S_ItoG * (I + skew(theta)).transpose()) * v_inG / theta
    // - pt_.normalized().transpose() * (S_ItoG * skew(theta)).transpose() * v_inG / theta
    // - pt_.normalized().transpose() * skew(theta).transpose() * S_ItoG.transpose() * v_inG / theta
    // - (skew(theta) * pt_.normalized()).transpose() * S_ItoG.transpose() * v_inG  / theta
    // - ( - skew(pt_.normalized()) * theta).transpose() * S_ItoG.transpose() * v_inG  / theta
    // theta.transpose() * skew(pt_.normalized()).transpose() * S_ItoG.transpose() * v_inG  / theta.transpose() .transpose()
    // (skew(pt_.normalized()).transpose() * S_ItoG.transpose() * v_inG).transpose()

    // 偏置雅可比 J_v_b_  
    // Eigen::Vector3d J_v_b_ = - S_ItoG.matrix().inverse() * pt_.normalized();

    // 1-10 修改
    // Eigen::Vector3d J_v_b_ = - pt_.normalized().transpose();
    // residuals[0] = 
    // (weight_ * (doppler_ - pt_.normalized().transpose() * (S_ItoG.matrix().transpose() * v_inG + linear_bias)));
    // Eigen::Vector3d J_v_b_ = - weight_ *  S_ItoG.matrix() * pt_.normalized();
    // Eigen::Vector3d J_v_b_ = - S_ItoG.matrix().inverse() * pt_.normalized();
    // LOG(ERROR) << "SplineOrder = " << SplineOrder << std::endl;

    // LOG(ERROR) << " start to calculate Rotation control points " << std::endl;

    /// Rotation control point
/*
    Eigen::Matrix<double, 4, 4> Jac_R;
    for (size_t i = 0; i < knot_num; i++) {

      size_t idx = i + J_R.start_idx;
      // LOG(ERROR) << " J_R.start_idx = " <<  J_R.start_idx << std::endl;
      // LOG(ERROR) << "idx = " << idx << std::endl;
      if (jacobians[idx]) {

        // for Debug
        // double* temp_test = jacobians[idx];
        // for(size_t j = 0;j < 12;j++)
        // {
        //   LOG(ERROR) << "j = " << j << std::endl;
        //   LOG(ERROR) << "jacobians[j] = " << temp_test[j] << std::endl;
        // }
      
        Eigen::Map<Eigen::Matrix<double, 1, 4, Eigen::RowMajor>> jac_kont_R(
            jacobians[idx]);
        jac_kont_R.setZero();
       
      //  jac_kont_R.setZero();

       // 在雷达多普勒中不优化旋转    
       {
        Eigen::RowVector3d jac_kont_R_pre = jac_lhs_R.transpose() * J_R.d_val_d_knot[i];

        // debug_ceres.debug_file << "J_R.d_val_d_knot[" << i << "] = [" <<  Output_M(J_R.d_val_d_knot[i]) << "]\n";

        // debug_ceres.debug_file << "J_R_pre.d_val_d_knot[" << i << "] = [" << Output(jac_kont_R_pre) << "]" << std::endl;
        jac_kont_R.block<1, 3>(0, 0) += (w_weight_ * weight_ * jac_kont_R_pre).eval();

        LOG(ERROR) << " J_R.d_val_d_knot[ " << i << "]" << std::endl;    

        Eigen::RowVectorXd jac_kont_R_copy = jac_kont_R;

        LOG(ERROR) << "J_R_" << i << " = [" << Output(jac_kont_R_copy) << std::endl;
        // // debug_ceres.debug_file << "J_R_" << i << " = [" << jac_kont_R_copy.transpose() << "]\n";
        // debug_ceres.debug_file << "J_R_" << i << " = [" << Output(jac_kont_R_copy) << "]\n";
      
        Jac_R.block<1,4>(i, 0) = jac_kont_R_copy;

        // // debug_ceres_jacobis.debug_file << jac_kont_R_copy.transpose() << " ";
        // debug_ceres_jacobis.debug_file << Output(jac_kont_R_copy) << " ";
      }
        // Eigen::RowVectorXd jac_kont_R_copy = jac_kont_R;
        // Jac_R.block<1,4>(i, 0) = jac_kont_R_copy;
      
      }
    }
    // LOG(ERROR) << "Calculate Rotation Control Jacobbi " << std::endl;
*/
    // 检查 I:
    // debug_ceres.debug_file << "Jac_R = " << Output_M(Jac_R) << std::endl;

    // LOG(ERROR) << "Jacobi for position J_v." << std::endl;
      // for (size_t i = 0; i < knot_num; ++i) {
      //     LOG(ERROR) << "Mat3[" << i << "] =\n" << J_v.d_val_d_knot[i] << "\n\n";
      // }

    Eigen::Vector3d J_v_b_;
     
    // HAO TODO: 4-6修改
    // J_v_b_ = (- S_ItoG.matrix() * pt_.normalized());

    J_v_b_ = - pt_.normalized();
    // J_v_b_ = (- weight_ * pt_.normalized());
    // debug_ceres.debug_file << "- pt_.normalized.transpose = " << Output(- pt_.normalized().transpose()) << "\n";
    // debug_ceres.debug_file << "S_ItoG = " << Output_M(S_ItoG.matrix()) << "\n";
    // J_v_d_ += ((- pt_.normalized().transpose() * S_ItoG.matrix().inverse()).transpose()).eval();
    // LOG(ERROR) << " J_v_d_ = " << std::endl;
    // debug_ceres.debug_file << "J_v_b_ = " << Output(J_v_b_.transpose()) << "\n";

    Eigen::Matrix<double, 4, 3> Jac_p;
    /// position control point
    // for (size_t i = knot_num; i < 2 * knot_num; i++) {
    for (size_t i = 0; i < knot_num; i++) {
      size_t idx = i;   // [0, knot_num - 1]
      // LOG(ERROR) << "idx = " << idx << std::endl;
      if (jacobians[idx]) {
        Eigen::Map<Eigen::RowVector3d> jac_kont_p(
            jacobians[idx]);
        jac_kont_p.setZero();
        
        // double d_val_d_knot = J_v.d_val_d_knot[i - knot_num];
        // Eigen::VectorXd temp_cal;
        // Eigen::RowVector3d temp_cal;
        // temp_cal.setZero();
        // temp_cal = (J_v_d_ * J_v.d_val_d_knot[i - knot_num]);
        // temp_cal = (d_val_d_knot * J_v_b_);
        // temp_cal = (J_v_b_ * d_val_d_knot);

        // double J_v_d_copy = J_v.d_val_d_knot[i - knot_num];
        // LOG(ERROR) << "J_v.d_val_d_knot[i - knot_num] = " << J_v_d_copy << std::endl;
        // debug_ceres.debug_file << "J_v.d_val_d_knot[" << i - knot_num << "] = " << J_v_d_copy << std::endl;


        // // debug_ceres.debug_file << "temp_cal = [" << temp_cal << "]"
        //                        << std::endl;

        // // debug_ceres.debug_file << "temp_cal = [" << J_v_d_ << "] "
        //                        << " * [" << J_v.d_val_d_knot[i - knot_num] << "]"
        //                        << " = 1: " << (J_v_d_ * J_v.d_val_d_knot[i - knot_num])
        //                        << "\n"
        //                        << " = 2: " << (J_v_d_ * J_v.d_val_d_knot[i - knot_num]).eval()
        //                        << std::endl;
        /// 1*1 1*3
        // jac_kont_p =  (J_v_d_ * J_v.d_val_d_knot[i - knot_num]).eval(); // TODO: i - knot_num 0-4 的范围
        // jac_kont_p =  (weight_ * J_v.d_val_d_knot[i - knot_num] * J_v_b_ ).eval(); // TODO: i - knot_num 0-4 的范围
        jac_kont_p = (w_weight_ * J_v_b_ * J_v.d_val_d_knot[i]).eval(); // TODO: i - knot_num 0-4 的范围s
        // jac_kont_p = (weight_ * jac_kont_p).eval();

        Eigen::RowVector3d jac_kont_p_copy; // = jac_kont_p;
        jac_kont_p_copy = jac_kont_p;
        // // debug_ceres.debug_file << "J_p_" << i - knot_num << " = [" << jac_kont_p_copy.transpose() << "]\n";

        // LOG(ERROR) << "J_v_d_" << i - knot_num << " = \n[" << J_v_d_ << "]\n";
        // LOG(ERROR) << "J_v_b_ " << std::endl;
        // // debug_ceres.debug_file << "J_v_d_" << i - knot_num << " = [" << J_v_d_.transpose() << "]\n";
        // debug_ceres.debug_file << "J_v_b_ = [" << Output(J_v_b_.transpose()) << "]\n";
        // // debug_ceres.debug_file << "J_v_d_" << i - knot_num << " = \n[" << J_v_d_.transpose() << "]\n";
        // LOG(ERROR) << "J_v_.d_val_d_knot_" << std::endl;
        // // debug_ceres.debug_file << "J_v_.d_val_d_knot_" << i - knot_num << " = [" << (J_v.d_val_d_knot[i - knot_num]) << "]\n";
        // debug_ceres.debug_file << "J_v_.d_val_d_knot_" << i - knot_num << " = [" << (J_v_d_copy) << "]\n";

        // Eigen::Matrix<double, 1, 3, Eigen::RowMajor> jac_kont_p_copy = jac_kont_p;
        // Eigen::VectorXd jac_kont_p_copy = jac_kont_p;
        // LOG(ERROR) << "J_p_" << i - knot_num << " = \n[" << jac_kont_p_copy << "]\n";
        // // debug_ceres.debug_file << "J_p_" << i - knot_num << " = [" << jac_kont_p_copy.transpose() << "]\n";
        // LOG(ERROR) << "J_p_" << i - knot_num << " = [" << Output(jac_kont_p_copy) << std::endl;
        // debug_ceres.debug_file << "J_p_" << i - knot_num << " = [" << Output(jac_kont_p_copy) << "]\n";

        Jac_p.block<1,3>(i, 0) = jac_kont_p_copy;

        // // debug_ceres_jacobis.debug_file << jac_kont_p_copy.transpose() << " ";
        // LOG(ERROR) << "jac_kont_p_copy = " << " ";
        // debug_ceres_jacobis.debug_file <<  (jac_kont_p_copy) << " ";
      }
    }

    LOG(ERROR) << "Calculate Position Control Jacobbi " << std::endl;

    std::chrono::time_point<std::chrono::high_resolution_clock> time5;

    // // DEBUG:
    // LOG(ERROR) << "Parameters.size = " << this->parameter_block_sizes().size() << std::endl;

    // // LOG(ERROR) << "Parameters.size = " << mutable_parameter_block_sizes().size() << std::endl;
    // LOG(ERROR) << "Search for = " << 2 * knot_num << std::endl;

    // for (int i = 0; i < this->parameter_block_sizes().size(); ++i) {
    //     LOG(ERROR) << "Block size at index " << i << " = " << parameter_block_sizes()[i];
    // }
    // // 使用所有的参数块
    // for (int i = 0; i < this->parameter_block_sizes().size(); ++i) {
    //     LOG(ERROR) << "Parameter " << i << " = " << parameters[i][0];
    // }
    // LOG(ERROR) << "Parameter for last = " << parameters[8][0] 
    // << ", " << parameters[8][1] 
    // << ", " << parameters[8][2] << std::endl;

    // for (size_t i = 0; i < this->parameter_block_sizes().size(); ++i) {
    //     if (jacobians[i] != nullptr) {
    //         LOG(ERROR) << "Jacobian at index " << i << " is valid.";
    //     } else {
    //         LOG(ERROR) << "Jacobian at index " << i << " is nullptr.";
    //     }
    // }

    // [3] velocity_bias 的雅可比
    // residuals[0] = 
    // (weight_ * (doppler_ - pt_.normalized().transpose() * (S_ItoG.matrix().transpose() * v_inG + linear_bias)));    
    // Eigen::Vector3d J_v_b_ = - weight_ * pt_.normalized();
    Eigen::RowVector3d J_velocity_bias_copy_;
    Eigen::RowVector3d J_v_bias_ = - pt_.normalized();
    
    // if(jacobians[2 * knot_num])
    if(jacobians[knot_num])
    {
      // Eigen::Map<Eigen::Matrix<double, 1, 3>> J_velocity_bias_(jacobians[2 * knot_num]);
       
      // LOG(ERROR) << "J_velocity_bias_ = " << ((jacobians[2 * knot_num] == nullptr)? "nullptr": "exist") << std::endl;
      // Eigen::Map<Eigen::Matrix<double, 1, 3, Eigen::RowMajor>> J_velocity_bias_(jacobians[2 * knot_num]);
      Eigen::Map<Eigen::RowVector3d> J_velocity_bias_(jacobians[knot_num]);
      // LOG(ERROR) << "J_velocity_bias_ = " << J_velocity_bias_ << std::endl;

      // LOG(ERROR) << "J_velocity_rot_ after = " << ((jacobians[2 * knot_num - 1] == nullptr)? "nullptr": "exist") << std::endl;

      // J_velocity_bias_.setZero();
      // LOG(ERROR) << "J_v_b_ = " << J_v_b_.transpose() << std::endl;
      // J_velocity_bias_ = (J_v_b_.transpose()).eval();
      J_velocity_bias_ = J_v_bias_;
      J_velocity_bias_ = (w_weight_ * J_velocity_bias_).eval();
      // LOG(ERROR) << "J_v_b_ = " << jacobians[2 * knot_num][0] << ", " 
      //             << jacobians[2 * knot_num][1] << ", "
      //             << jacobians[2 * knot_num][2] << ", "<< std::endl;
      // LOG(ERROR) << "J_v_b_ = " << J_velocity_bias_ << std::endl;
      // LOG(ERROR) << "J_v_b_ = " << J_velocity_bias_ << std::endl;

      // Eigen::Matrix<double, 1, 3, Eigen::RowMajor> J_velocity_bias_copy_ = J_velocity_bias_;
      J_velocity_bias_copy_ = J_velocity_bias_;
      // LOG(ERROR) << "J_v_b_" << " = [\n" << J_velocity_bias_copy_ << "]\n";
      // LOG(ERROR) << "J_velocity_bias_copy_";
      // debug_ceres.debug_file << "J_v_bias_" << Output(J_v_bias_.transpose()) << std::endl;
      // debug_ceres.debug_file << "J_velocity_bias_ = " << Output(J_velocity_bias_) << std::endl;
      // debug_ceres_jacobis.debug_file << Output(J_velocity_bias_copy_) << "]\n";
    }
    else
    {
      LOG(ERROR) << "No Calculate Velocity Bias Jacobbi " << std::endl;
    }

    std::chrono::time_point<std::chrono::high_resolution_clock> time6;

    // LOG(ERROR) << "No Calculate Velocity Bias Jacobbi " << std::endl;

    // [4] timeoffset 本身无偏置,因此不用加入
    // Eigen::Map<Eigen::Matrix<double, 1, 3>> J_timeoffset_(jacobians[knot_num + 2]);
    // J_timeoffset_.setZero();
    // J_timeoffset_ = J_v_b_;    

    // LOG(ERROR) << "DopplerFactor Evaluation Done" << std::endl;

    // LOG(ERROR) << "All Jacobi for Doppler factor" << std::endl;
    // debug_ceres.debug_file << "All Jacobi for Doppler factor " << std::endl;
    // debug_ceres.debug_file << "Jac_R_ = " << Output_M(Jac_R) << std::endl;
    // debug_ceres.debug_file << "J_p_ = " << Output_M(Jac_p) << std::endl;
    // debug_ceres.debug_file << "J_velocity_bias_ = " << Output(J_velocity_bias_copy_) << std::endl;

    // LOG(ERROR) << "All Jacobi for Doppler factor" << std::endl;
    // // LOG(ERROR) << "Jac_R_ = " << Output_M(Jac_R) << std::endl;
    // LOG(ERROR) << "J_p_ = " << Output_M(Jac_p) << std::endl;
    // LOG(ERROR) << "J_velocity_bias_ = " << Output(J_velocity_bias_copy_) << std::endl;

    /*
    LOG(ERROR) << "Jac_p = " << Jac_p << std::endl;
    LOG(ERROR) << "J_velocity_bias_copy_ = " << J_velocity_bias_copy_ << std::endl;
    */

    std::chrono::time_point<std::chrono::high_resolution_clock> time7;

    // debug_ceres.debug_file << std::endl;
    // debug_ceres.Close();
    // debug_ceres_jacobis.Close();

    // LOG(ERROR) << "print close" << std::endl;

    /*{
      std::chrono::duration<double, std::milli> elapsed;
      elapsed = time7 - time1;
      LOG(ERROR) << "Total Time: " << elapsed.count() << std::endl;
      elapsed = time2 - time1;
      LOG(ERROR) << "Pre-Optimization: " << elapsed.count() << std::endl;
      elapsed = time3 - time2;
      LOG(ERROR) << "Evaluate-Residual: " << elapsed.count() << std::endl;
      elapsed = time5 - time3;
      LOG(ERROR) << "Jacobian Position: " << elapsed.count() << std::endl;
      elapsed = time6 - time5;
      LOG(ERROR) << "Jacobian Linear Bias: " << elapsed.count() << std::endl;
      elapsed = time7 - time6;
      LOG(ERROR) << "Output: " << elapsed.count() << std::endl;
    }*/


    return true;
  }

private:
    int64_t time_ns_;
    Eigen::Vector3d pt_;
    SplineMeta<SplineOrder> linear_spline_meta_;
    // Vec6d info_vec_;

    //TODO: gravity is not necessary
    // Eigen::Vector3d gravity;
    double doppler_;
    double weight_;

    // 某些参数的弱优化权重
    double w_weight_;

    std::shared_ptr<FEJ_STATE> global_fej_state_;
    bool use_fej_;

    // Eigen::Vector3d J_v_d_;
    // Eigen::Vector3d J_v_b_;
};


class EventAgularFactor4 : public ceres::CostFunction, SplitSpineView{
 public:
 EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  using SO3View = So3SplineView;
  using R3View = RdSplineView;
  using SplitView = SplitSpineView;

  using Vec3d = Eigen::Matrix<double, 3, 1>;
  using Vec6d = Eigen::Matrix<double, 6, 1>;
  using Mat3d = Eigen::Matrix<double, 3, 3>;
  using Quatd = Eigen::Quaterniond;
  using SO3d = Sophus::SO3<double>;

  EventAgularFactor4(int64_t time_ns, const Eigen::Vector3d pt, 
            // const event_flow_velocity flow,
            Eigen::Vector3d normal_flow,
            double normal_norm,
            const Eigen::Vector3d doppler_velocity,
            const Eigen::Quaterniond q_e_r, const Eigen::Vector3d t_e_r,
            const SplineMeta<SplineOrder>& linear_spline_segment_meta,
            const SplineMeta<SplineOrder>& angular_spline_segment_meta,
            std::shared_ptr<FEJ_STATE> global_fej_state, bool use_fej,
            // const Eigen::Vector3d linear_bias, const Eigen::Vector3d angular_bias,
            double weight, double w_weight = 0.05) // const Vec6d& info_vec)
      : time_ns_(time_ns),
        pixel_cord(pt), normal_norm_(normal_norm),
        normal_flow_(normal_flow),
        doppler_velocity_(doppler_velocity),
        q_e_r(q_e_r), t_e_r(t_e_r),
        linear_spline_meta_(linear_spline_segment_meta),
        angular_spline_meta_(angular_spline_segment_meta),
        global_fej_state_(global_fej_state),
        use_fej_(use_fej),
        // linear_bias_(linear_bias), omega_bias_(angular_bias),
        lock_extrincs(true),      // HAO TODO: 暂时不优化外参
        weight_(weight), w_weight_(w_weight)
        {
          // set_num_residuals(1);           // 定义残差值的大小(事件角速度残差)
          set_num_residuals(3);           // 定义残差值的大小(事件角速度残差)

          size_t knot_num = this->angular_spline_meta_.NumParameters();
          // LOG(ERROR) << "knot_num = " << knot_num << std::endl;
          // flow_ << flow.x, flow.y, 0.0;

          for (size_t i = 0; i < knot_num; ++i) {
            mutable_parameter_block_sizes()->push_back(3);   // linear velocity
          }

          // TODO: 需要角速度曲线              
          for (size_t i = 0; i < knot_num; ++i) {
            mutable_parameter_block_sizes()->push_back(3);   // angular velocity
          }

          mutable_parameter_block_sizes()->push_back(3);    // linear bias // 1 -11 修改
          
          mutable_parameter_block_sizes()->push_back(3);    // angular bias // 1 -11 修改
        }

  virtual bool Evaluate(double const* const* parameters, double* residuals,
                          double** jacobians) const {
        typename R3View::JacobianStruct J_v_;
        typename R3View::JacobianStruct J_w_;

        std::chrono::time_point<std::chrono::high_resolution_clock> time1;

        LOG(ERROR) << "Evaluate EventAgularFactor4 " << std::endl;
        // debug_ceres.Open();
        // debug_ceres.debug_file << std::endl;
        // debug_ceres.debug_file << "    ---------------------- Evaluate BodyLocalAngularVelocityFactor ------------------    " << std::endl;

        size_t knot_num = angular_spline_meta_.NumParameters();

        // double time_offset_in_ns = parameters[t_offset_index][0];
        double time_offset_in_ns = 0;
        int64_t t_corrected = time_ns_ + (int64_t)time_offset_in_ns;
        t_corrected = std::min(t_corrected, angular_spline_meta_.segments.at(0).MaxTimeNs() - 1);
        t_corrected = std::max(t_corrected, angular_spline_meta_.segments.at(0).MinTimeNs() + 1);
  
        Eigen::Vector3d vel_, gyro_, rot_accel;
        if (jacobians){
          // S_ItoG = SO3View::EvaluateRp(t_corrected, spline_meta_.segments.at(0),
          //                 parameters, &J_R);

          // gyro_ = R3View::velocity(t_corrected,
          //                               spline_meta_.segments.at(0),
          //                               parameters, &J_w_);   
          vel_ = R3View::evaluate(t_corrected,
                                        linear_spline_meta_.segments.at(0),
                                        parameters, &J_v_); 
          gyro_ = R3View::evaluate(t_corrected,
                                        angular_spline_meta_.segments.at(0),
                                        parameters + knot_num, &J_w_);   

          // rot_accel = SO3View::accelerationBody(t_corrected,  
          //                                       spline_meta_.segments.at(0), 
          //                                       parameters);

          // v_inG = R3View::velocity(t_corrected,
          //                               spline_meta_.segments.at(0),
          //                               parameters + knot_num, &J_v);   
        }else{
          // S_ItoG = SO3View::EvaluateRp(t_corrected, spline_meta_.segments.at(0),
          //                 parameters, nullptr);

          // gyro_ = R3View::velocity(t_corrected,
          //                     spline_meta_.segments.at(0),
          //                     parameters, nullptr);  
          vel_ = R3View::evaluate(t_corrected,
                                        linear_spline_meta_.segments.at(0),
                                        parameters, nullptr);                                  
          gyro_ = R3View::evaluate(t_corrected,
                                        angular_spline_meta_.segments.at(0),
                                        parameters + knot_num, nullptr);  

          // rot_accel = SO3View::accelerationBody(t_corrected,  
          //                               spline_meta_.segments.at(0), 
          //                               parameters);
        
          // v_inG = R3View::velocity(t_corrected,
          //                       spline_meta_.segments.at(0),
          //                       parameters + knot_num, nullptr);   
        }
        // LOG(ERROR) << "Set Jacobi Matrix " << std::endl;



        // debug_ceres.debug_file << "Event Details: weight_ = " << weight_ 
                                      // << "\n w_weight_ = [" <<  w_weight_
                                      // // << "\n S_ItoG = [" <<  Output_M(S_ItoG)
                                      // << "\n local_angular_vel_= [" <<  Output(local_angular_vel_) << "] "
                                      // << "\n gyro_ = [" <<  Output(gyro_.transpose()) << "] "
                                      // << "\n angular_bias = [" <<  Output(angular_bias) << "] " << std::endl;

        // LOG(ERROR) << "Event Details: weight_ = " << weight_ 
        //                               << "\n w_weight_ = [" <<  w_weight_
        //                               // << "\n S_ItoG = [" <<  Output_M(S_ItoG)
        //                               << "\n local_angular_vel_= [" <<  Output(local_angular_vel_) << "] "
        //                               << "\n gyro_ = [" <<  Output(gyro_.transpose()) << "] "
        //                               << "\n angular_bias = [" <<  Output(angular_bias) << "] " << std::endl;

      // 3-18 修改 法向光流的计算
      // Eigen::Vector3d grad;
      // grad << -1.0 / flow_(0), -1.0 / flow_(1), 0.0;
      // double normal_norm = 1.0 / grad.norm();
      // Eigen::Vector3d normal_flow = grad * normal_norm;

      // 10-8 修改 法向光流的计算
      // Eigen::Vector3d normal_flow;
      // double normal_norm;
      /*
      {
        const auto plane = plane_;
        
        // double norm_grad = sqrt(plane(0) * plane(0) + plane(1) * plane(1)); //  a* a + b* b
        normal_flow <<  plane(0), plane(1), 0.0;
        double norm_grad = normal_flow.norm();
        normal_flow = normal_flow / norm_grad;     // grad / grad.norm();
        normal_norm = - plane(2) / norm_grad;
        Eigen::Vector3d flow = normal_norm * normal_flow;

        Eigen::Matrix3d K_inv = K.inverse();
        // 上面的像素光流需要转到相机系下光流
        double focal_len_inv = (K_inv(0,0) + K_inv(1,1)) / 2;
        LOG(ERROR) << "focal_len_inv = " << focal_len_inv << std::endl;
        normal_norm *= focal_len_inv;
        LOG(ERROR) << "normal_norm = " << normal_norm << std::endl;
      }*/
      
      // normal_flow = flow_.cwiseAbs2() / flow_.squaredNorm();
      // normal_norm = normal_flow.norm();
      // normal_flow = normal_flow / normal_norm;


      Eigen::Map<Vec3d const> linear_bias_(parameters[2 * knot_num]);
      Eigen::Map<Vec3d const> omega_bias_(parameters[2 * knot_num + 1]);

      Eigen::Matrix3d R_e_r = q_e_r.toRotationMatrix(); 
      Eigen::Vector3d t_r_e_temp = R_e_r.transpose() * t_e_r;
      Eigen::Vector3d t_r_e = -1.0 * t_r_e_temp;

      // 保留线速度估计 + bias的优化
      Eigen::Vector3d pre_vec = Skew(pixel_cord) * (R_e_r.transpose() * ((vel_ + linear_bias_) + (gyro_ + R_e_r * omega_bias_).cross(t_r_e)));      
      // use for transport
      // double post_vec = normal_norm + normal_flow.transpose() * Skew(pixel_cord) * (R_e_r.transpose() * gyro_ + omega_bias_);
      double post_vec = normal_norm_ + normal_flow_.transpose() * Skew(pixel_cord) * (R_e_r.transpose() * gyro_ + omega_bias_);

      if(pre_vec.norm() < 1e-3)
        pre_vec = Eigen::Vector3d::Identity();

      // 保留bias的优化
      // Eigen::Vector3d pre_vec = Skew(pixel_cord) * (R_e_r.transpose() * ((doppler_velocity_ + linear_bias) + (gyro_ + R_e_r * omega_bias).cross(t_r_e)));      
      // // use for transport
      // double post_vec = normal_norm + normal_flow.transpose() * Skew(pixel_cord) * (R_e_r.transpose() * gyro_ + omega_bias);

      // // 去bias
      // Eigen::Vector3d pre_vec = Skew(pixel_cord) * (R_e_r.transpose() * ((doppler_velocity_) + (gyro_).cross(t_r_e)));      
      // pre_vec.normalize();
      // double doppler_scale_aid = pre_vec.norm();
      // // use for transport
      // double post_vec = normal_norm + normal_flow.transpose() * Skew(pixel_cord) * (R_e_r.transpose() * gyro_);

      /*
      修改问题,非线性优化 -> 线性
      ref f(x) |-> log(1 + f(x))   f(x)|->0 
      */


      // LOG(ERROR) << "post_vec = " << post_vec << std::endl;

        std::chrono::time_point<std::chrono::high_resolution_clock> time2;

        Eigen::Map<Eigen::Vector3d> residual(residuals);
        residual.setZero();
        Eigen::Vector3d residual_temp = pre_vec * post_vec;
        residual = residual_temp;
        // residual = (weight_ * w_weight_ * residual).eval();
        residual = (weight_ * residual).eval();

        // double residual = post_vec;
        // residuals[0] = residual;


        // LOG(ERROR) << "local angular velocity residuals = " 
        //                         << (residual) << std::endl;

        LOG(ERROR) << "Event Details: weight_ = " << weight_ 
                    << "\n w_weight_ = " <<  w_weight_
                    << "\n pixel_cord = " <<  Output_M(pixel_cord)
                    // << "\n flow_ = " <<  Output(flow_)
                    << "\n doppler_velocity_ = " <<  Output(doppler_velocity_)
                    << "\n vel_ = " <<  Output(vel_)
                    //  << "\n linear_bias = " <<  Output(linear_bias)
                    << "\n R_e_r = " <<  Output_M(R_e_r)
                    << "\n t_r_e = " << Output(t_r_e.transpose()) 
                    << "\n gyro_ = " <<  Output(gyro_) 
                    << "\n normal_norm_ = " <<  normal_norm_ 
                    << "\n normal_flow_ = " <<  Output(normal_flow_) 
                    << "\n linear_bias_ = " <<  Output(linear_bias_) 
                    << "\n omega_bias_ = " <<  Output(omega_bias_) 
                    //  << "\n omega_bias = " <<  Output(omega_bias) 
                    << std::endl;
        LOG(ERROR) << "pre_vec = " << Output(pre_vec)
                  << "\n post_vec = " << post_vec << std::endl;

        LOG(ERROR) << "local angular velocity residuals = " 
                                << Output_M(residual) << std::endl;
        LOG(ERROR) << "local angular velocity residuals norm = " 
                                << residual.norm() << std::endl;
        // LOG(ERROR) << "Event Loss: " << ((residual.norm() > 0.3)? "True": "False") << std::endl; 

        /*std::fstream angular_file("/home/hao/Desktop/twist_ws/src/TwistEstimator/output/angular_res.txt",
                                  std::ios::out | std::ios::app);*/
        // angular_file << Output_M(residual) << std::endl;
        // angular_file << residual.norm() << std::endl;
        // angular_file.close();

        /*if(global_fej_state_)
        {
          LOG(ERROR) << "Use fej state = \n"
                 << "linear_velocity_ = "<< global_fej_state_->linear_velocity_ << "\n"
                << "doppler_ = " << doppler_ << "\n"
                << "pt_.normalized() = " << Output(pt_.normalized()) << "\n"
                << "v_inG = " << Output(v_inG) << "\n" 
                << "linear_bias = " << Output(global_fej_state_->linear_bias_) << "\n"  
                << std::endl;
        }*/

        std::chrono::time_point<std::chrono::high_resolution_clock> time3;

        // 不评估雅可比就返回
        if (!jacobians) {
          // LOG(ERROR) << "BodyLocalAngularVelocityFactor No J" << std::endl;
          // debug_ceres.debug_file << "BodyLocalAngularVelocityFactor No Jacobi!" << std::endl;
          // debug_ceres.Close();

          // // debug_ceres_jacobis.Close();
          return true;
        }
        // debug_ceres_jacobis.Open();
        // debug_ceres_jacobis.debug_file << "Evaluate EventAgularFactor Res J_R_0 J_R_1 J_R_2 J_R_3 J_p_0 J_p_1 J_p_2 J_p_3 J_linear_bias J_angular_bias J_t \n";
        // debug_ceres_jacobis.debug_file << Output(residual_temp.transposacobi!e()) << std::endl;

        /*
          Eigen::Vector3d linear_velocity_;
          Eigen::Vector3d angular_velocity_;
          Eigen::Vector3d linear_bias_;
          Eigen::Vector3d angular_bias_;    
        */

        // 加入FEJ状态
        if(global_fej_state_ && use_fej_)
        {
          pre_vec = Skew(pixel_cord) * (R_e_r.transpose() * ((global_fej_state_->linear_velocity_
                     + global_fej_state_->linear_bias_) + (global_fej_state_->angular_velocity_ 
                     + R_e_r * global_fej_state_->angular_bias_).cross(t_r_e)));      
          post_vec = normal_norm_ + normal_flow_.transpose() * Skew(pixel_cord) * (R_e_r.transpose() 
                      * global_fej_state_->angular_velocity_ + global_fej_state_->angular_bias_);

          LOG(ERROR) << "FEJ STATE:\n linear velocity = " << Output(global_fej_state_->linear_velocity_)
                  << "\n linear bias = " << Output(global_fej_state_->linear_bias_) 
                  << "\n angular velocity = " << Output(global_fej_state_->angular_velocity_) 
                  << "\n angular bias = " << Output(global_fej_state_->angular_bias_) 
                  << std::endl;          
          LOG(ERROR) << "FEJ: pre_vec = " << Output(pre_vec)
                  << "\n FEJ: post_vec = " << post_vec << std::endl;
        }

        // [0] 对 kont_v 的 雅可比
        Eigen::Matrix3d Jac_vel_pre = Skew(pixel_cord) * R_e_r.transpose();
        Eigen::Matrix<double, 12, 3> Jac_v;
        for (size_t i = 0; i < knot_num; i++) {
          size_t idx = i;
          if (jacobians[idx]) {
            Eigen::Map<Eigen::Matrix3d> jac_kont_v(
                jacobians[idx]);
            jac_kont_v.setZero();

            // jac_kont_p = S_ItoG.matrix().transpose() * J_v.d_val_d_knot[i];
            // jac_kont_p = (weight * jac_kont_p).eval();
            Eigen::Matrix3d Jac_vel_ = Jac_vel_pre * J_v_.d_val_d_knot[i] * post_vec;
            jac_kont_v = (0.0 * w_weight_ * Jac_vel_).eval();

            Eigen::Matrix3d jac_kont_v_copy = jac_kont_v;

            // LOG(ERROR) << "Jac_vel_pre = " << Output_M(Jac_vel_pre) << std::endl;
            // LOG(ERROR) << "J_v_.d_val_d_knot[i] = " << J_v_.d_val_d_knot[i] << std::endl;
            // LOG(ERROR) << "post_vec = " << post_vec << std::endl;
            // LOG(ERROR) << "jac_kont_v_copy = " << Output_M(jac_kont_v_copy) << std::endl;

            // LOG(ERROR) << "J_w_.d_val_d_knot[" << i << "] = [" << J_w_.d_val_d_knot[i] << "]\n";
            // // debug_ceres.debug_file << "J_v.d_val_d_knot[" << i << "] = [" << Output_M(J_v.d_val_d_knot[i]) << "]\n";
            // // debug_ceres.debug_file << "jac_lhs_R_" << i << " = [" << Output_M(jac_kont_p_copy) << "]\n";

            Jac_v.block<3,3>(3 * i, 0) = jac_kont_v_copy;

            // debug_ceres_jacobis.debug_file << Output_M(jac_kont_p)  << std::endl;

          }
        }

        std::chrono::time_point<std::chrono::high_resolution_clock> time4;
        
        
        // [1] 对 kont_w 的 雅可比
        // Eigen::Matrix3d Jac_gyro_pre = - Skew(pixel_cord) * R_e_r.transpose() * Skew(t_r_e);
        // // 前一项固定,后一项优化
        // Eigen::Matrix3d Jac_gyro_pre;
        // Jac_gyro_pre.setZero();
        // Eigen::RowVector3d Jac_gyro_post = normal_flow.transpose() * Skew(pixel_cord) * R_e_r.transpose();
        // LOG(ERROR) << "Jac_gyro_pre = " << Output_M(Jac_gyro_pre) << std::endl;
        // LOG(ERROR) << "Jac_gyro_post = " << Output_M(Jac_gyro_post) << std::endl;

        // 只要最后一项的优化
        // Eigen::Matrix<double, 4, 3> Jac_w;
        // for (size_t i = 0; i < knot_num; i++) {
        //   size_t idx = i;
        //   if (jacobians[idx]) {
        //     Eigen::Map<Eigen::RowVector3d> jac_kont_w(
        //         jacobians[idx]);
        //     jac_kont_w.setZero();

        //     // jac_kont_p = S_ItoG.matrix().transpose() * J_v.d_val_d_knot[i];
        //     // jac_kont_p = (weight * jac_kont_p).eval();
        //     Eigen::RowVector3d Jac_gyro_ = Jac_gyro_post * J_w_.d_val_d_knot[i];
        //     jac_kont_w = (weight_ * Jac_gyro_).eval();

        //     Eigen::RowVector3d jac_kont_w_copy = jac_kont_w;

        //     // LOG(ERROR) << "J_w_.d_val_d_knot[" << i << "] = [" << J_w_.d_val_d_knot[i] << "]\n";
        //     // // debug_ceres.debug_file << "J_v.d_val_d_knot[" << i << "] = [" << Output_M(J_v.d_val_d_knot[i]) << "]\n";
        //     // // debug_ceres.debug_file << "jac_lhs_R_" << i << " = [" << Output_M(jac_kont_p_copy) << "]\n";

        //     Jac_w.block<1,3>(i, 0) = jac_kont_w_copy;

        //     // debug_ceres_jacobis.debug_file << Output_M(jac_kont_p)  << std::endl;

        //   }
        // }
        // LOG(ERROR) << "Add jacobians for Position control point " << std::endl;  
 
        Eigen::Matrix3d Jac_gyro_pre = - Skew(pixel_cord) * R_e_r.transpose() * Skew(t_r_e);
        Eigen::RowVector3d Jac_gyro_post = normal_flow_.transpose() * Skew(pixel_cord) * R_e_r.transpose();
        Eigen::Matrix<double, 12, 3> Jac_w;
        for (size_t i = 0; i < knot_num; i++) {
          size_t idx = i + knot_num;
          if (jacobians[idx]) {
            Eigen::Map<Eigen::Matrix3d> jac_kont_w(
                jacobians[idx]);
            jac_kont_w.setZero();

            // jac_kont_p = S_ItoG.matrix().transpose() * J_v.d_val_d_knot[i];
            // jac_kont_p = (weight * jac_kont_p).eval();
            // Eigen::Matrix3d Jac_gyro_ = Jac_gyro_pre * J_w_.d_val_d_knot[i] * post_vec 
            //                               + pre_vec * Jac_gyro_post * J_w_.d_val_d_knot[i];
            Eigen::Matrix3d Jac_gyro_ = pre_vec * Jac_gyro_post * J_w_.d_val_d_knot[i];

                      
            jac_kont_w = (w_weight_ * Jac_gyro_).eval();

            Eigen::Matrix3d jac_kont_w_copy = jac_kont_w;

            // LOG(ERROR) << "Jac_gyro_pre = " << Output_M(Jac_gyro_pre) << std::endl;
            // LOG(ERROR) << "J_w_.d_val_d_knot[i] = " << J_w_.d_val_d_knot[i] << std::endl;
            // LOG(ERROR) << "Jac_gyro_post = " << Output(Jac_gyro_post) << std::endl;
            // LOG(ERROR) << "jac_kont_w = " << Output_M(Jac_gyro_post) << std::endl;

            // LOG(ERROR) << "J_w_.d_val_d_knot[" << i << "] = [" << J_w_.d_val_d_knot[i] << "]\n";
            // // debug_ceres.debug_file << "J_v.d_val_d_knot[" << i << "] = [" << Output_M(J_v.d_val_d_knot[i]) << "]\n";
            // // debug_ceres.debug_file << "jac_lhs_R_" << i << " = [" << Output_M(jac_kont_p_copy) << "]\n";

            Jac_w.block<3,3>(3 * i, 0) = jac_kont_w_copy;

            // debug_ceres_jacobis.debug_file << Output_M(jac_kont_p)  << std::endl;

          }
        }

        std::chrono::time_point<std::chrono::high_resolution_clock> time5;

        // [2] 对 linear_bias 的 雅可比
        Eigen::Matrix3d Jac_linear_bias_pre_ = Skew(pixel_cord) * R_e_r.transpose();
        Eigen::Matrix3d Jac_linear_bias_ = Jac_linear_bias_pre_ * post_vec;
        // LOG(ERROR) << "Jac_linear_bias_pre_ = " << Output_M(Jac_linear_bias_pre_) << std::endl;
        // LOG(ERROR) << "Jac_linear_bias_ = " << Output_M(Jac_linear_bias_) << std::endl;
        Eigen::Matrix3d J_linear_bias_copy;
        if(jacobians[2 * knot_num])
        {
          // Eigen::Map<Eigen::Matrix<double, 1, 3>> J_linear_bias_(jacobians[2 * knot_num]); 
          Eigen::Map<Eigen::Matrix<double, 3, 3>> J_linear_bias_(jacobians[2 * knot_num]); 
          J_linear_bias_.setZero();

          // J_linear_bias_ = (weight_ * 0.01 * Jac_linear_bias_).eval();
          J_linear_bias_ = (w_weight_ * 0.0 * Jac_linear_bias_).eval();

          J_linear_bias_copy = J_linear_bias_;

          // debug_ceres.debug_file << "J_angular_bias_ = " << Output_M(jac_angular_copy) << "]\n";


          // debug_ceres_jacobis.debug_file << Output_M(jac_angular_copy) << std::endl;
        }

        std::chrono::time_point<std::chrono::high_resolution_clock> time6;

        // [3] 对 angular_bias 的 雅可比
        Eigen::Matrix3d Jac_omega_bias_pre_ = - Skew(pixel_cord) * R_e_r.transpose() * Skew(t_r_e) * R_e_r;
        Eigen::RowVector3d Jac_omega_bias_post_ = normal_flow_.transpose() * Skew(pixel_cord);

        // Eigen::Matrix3d Jac_omega_bias_ = Jac_omega_bias_pre_ * post_vec + pre_vec * Jac_omega_bias_post_;
        Eigen::Matrix3d Jac_omega_bias_ = pre_vec * Jac_omega_bias_post_;

        // LOG(ERROR) << "Jac_omega_bias_pre_ = " << Output_M(Jac_omega_bias_pre_) << std::endl;
        // LOG(ERROR) << "Jac_omega_bias_post_ = " << Output(Jac_omega_bias_post_) << std::endl;
        // LOG(ERROR) << "Jac_omega_bias_ = " << Output_M(Jac_omega_bias_) << std::endl;
        Eigen::Matrix3d J_angular_bias_copy;
        if(jacobians[2 * knot_num + 1])
        {
          // Eigen::Map<Eigen::Matrix<double, 1, 3>> J_linear_bias_(jacobians[2 * knot_num]); 
          Eigen::Map<Eigen::Matrix<double, 3, 3>> J_angular_bias_(jacobians[2 * knot_num + 1]); 
          J_angular_bias_.setZero();

          // J_angular_bias_ = (weight_ * 0.01 * Jac_omega_bias_).eval();
          J_angular_bias_ = (w_weight_ * Jac_omega_bias_).eval();

          // debug_ceres.debug_file << "J_angular_bias_ = " << Output_M(jac_angular_copy) << "]\n";
          J_angular_bias_copy = J_angular_bias_;

          // debug_ceres_jacobis.debug_file << Output_M(jac_angular_copy) << std::endl;
        }
        // LOG(ERROR) << "Add jacobians for Angular velocity bias " << std::endl;  
        
        std::chrono::time_point<std::chrono::high_resolution_clock> time7;

        // LOG(ERROR) << "All Jacobi for BodyLocalAngularVelocity factor" << std::endl;
        // debug_ceres.debug_file << "All Jacobi for BodyLocalAngularVelocity factor" << std::endl;
        // debug_ceres.debug_file << "Jac_R_ = " << Output_M(Jac_R) << std::endl;
        // debug_ceres.debug_file << "Jac_p_ = " << Output_M(Jac_p) << std::endl;
        // debug_ceres.debug_file << "J_angular_bias_ = " << Output_M(Jac_w_bias) << std::endl;

        // LOG(ERROR) << "All Jacobi for EventAngular4 factor" << std::endl;
        // LOG(ERROR) << "Jac_v = " << Output_M(Jac_v) << std::endl;
        // LOG(ERROR) << "Jac_w = " << Output_M(Jac_w) << std::endl;
        // LOG(ERROR) << "J_linear_bias_ = " << Output_M(J_linear_bias_copy) << std::endl;
        // LOG(ERROR) << "J_angular_bias_ = " << Output_M(J_angular_bias_copy) << std::endl;
        

        std::chrono::time_point<std::chrono::high_resolution_clock> time8;

        // debug_ceres.Close();
        // debug_ceres_jacobis.Close();
        
        return true;
    }
  
private:
    int64_t time_ns_;
    Eigen::Vector3d pixel_cord;
    // Eigen::Vector3d flow_;
    Eigen::Vector3d normal_flow_;
    double normal_norm_;
    Eigen::Vector3d doppler_velocity_;
    Eigen::Quaterniond q_e_r;
    Eigen::Vector3d t_e_r;
    SplineMeta<SplineOrder> linear_spline_meta_;
    SplineMeta<SplineOrder> angular_spline_meta_;
    // Vec6d info_vec_;
    // Eigen::Vector3d linear_bias_;
    // Eigen::Vector3d omega_bias_;
    bool lock_extrincs;
    std::shared_ptr<FEJ_STATE> global_fej_state_;
    bool use_fej_;
    double weight_;
    // 某些参数的弱优化权重
    double w_weight_;
};



class EventAgularFactor5 : public ceres::CostFunction, SplitSpineView{
 public:
 EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  using SO3View = So3SplineView;
  using R3View = RdSplineView;
  using SplitView = SplitSpineView;

  using Vec3d = Eigen::Matrix<double, 3, 1>;
  using Vec6d = Eigen::Matrix<double, 6, 1>;
  using Mat3d = Eigen::Matrix<double, 3, 3>;
  using Quatd = Eigen::Quaterniond;
  using SO3d = Sophus::SO3<double>;

  EventAgularFactor5(int64_t time_ns, const Eigen::Vector3d pt, 
            const event_flow_velocity flow, 
            const Eigen::Vector3d doppler_velocity,
            const Eigen::Quaterniond q_e_r, const Eigen::Vector3d t_e_r,
            const SplineMeta<SplineOrder>& linear_spline_segment_meta,
            const SplineMeta<SplineOrder>& angular_spline_segment_meta,
            // const Eigen::Vector3d linear_bias, const Eigen::Vector3d angular_bias,
            double weight, double w_weight = 0.05) // const Vec6d& info_vec)
      : time_ns_(time_ns),
        pixel_cord(pt), doppler_velocity_(doppler_velocity),
        q_e_r(q_e_r), t_e_r(t_e_r),
        linear_spline_meta_(linear_spline_segment_meta),
        angular_spline_meta_(angular_spline_segment_meta),
        // linear_bias_(linear_bias), omega_bias_(angular_bias),
        lock_extrincs(true),      // HAO TODO: 暂时不优化外参
        weight_(weight), w_weight_(w_weight)
        {
          // set_num_residuals(1);           // 定义残差值的大小(事件角速度残差)
          set_num_residuals(3);           // 定义残差值的大小(事件角速度残差)

          size_t knot_num = this->angular_spline_meta_.NumParameters();
          // LOG(ERROR) << "knot_num = " << knot_num << std::endl;
          flow_ << flow.x, flow.y, 0.0;

          for (size_t i = 0; i < knot_num; ++i) {
            mutable_parameter_block_sizes()->push_back(3);   // linear velocity
          }

          // TODO: 需要角速度曲线              
          for (size_t i = 0; i < knot_num; ++i) {
            mutable_parameter_block_sizes()->push_back(3);   // angular velocity
          }

          mutable_parameter_block_sizes()->push_back(3);    // linear bias // 1 -11 修改
          
          mutable_parameter_block_sizes()->push_back(3);    // angular bias // 1 -11 修改
        }

  virtual bool Evaluate(double const* const* parameters, double* residuals,
                          double** jacobians) const {
        typename R3View::JacobianStruct J_v_;
        typename R3View::JacobianStruct J_w_;

        LOG(ERROR) << "Evaluate EventAgularFactor3 " << std::endl;
        // debug_ceres.Open();
        // debug_ceres.debug_file << std::endl;
        // debug_ceres.debug_file << "    ---------------------- Evaluate BodyLocalAngularVelocityFactor ------------------    " << std::endl;

        size_t knot_num = angular_spline_meta_.NumParameters();

        // double time_offset_in_ns = parameters[t_offset_index][0];
        double time_offset_in_ns = 0;
        int64_t t_corrected = time_ns_ + (int64_t)time_offset_in_ns;
        t_corrected = std::min(t_corrected, angular_spline_meta_.segments.at(0).MaxTimeNs() - 1);
        t_corrected = std::max(t_corrected, angular_spline_meta_.segments.at(0).MinTimeNs() + 1);
  
        Eigen::Vector3d vel_, gyro_, rot_accel;
        if (jacobians){
          // S_ItoG = SO3View::EvaluateRp(t_corrected, spline_meta_.segments.at(0),
          //                 parameters, &J_R);

          // gyro_ = R3View::velocity(t_corrected,
          //                               spline_meta_.segments.at(0),
          //                               parameters, &J_w_);   
          vel_ = R3View::evaluate(t_corrected,
                                        linear_spline_meta_.segments.at(0),
                                        parameters, &J_v_); 
          gyro_ = R3View::evaluate(t_corrected,
                                        angular_spline_meta_.segments.at(0),
                                        parameters + knot_num, &J_w_);   

          // rot_accel = SO3View::accelerationBody(t_corrected,  
          //                                       spline_meta_.segments.at(0), 
          //                                       parameters);

          // v_inG = R3View::velocity(t_corrected,
          //                               spline_meta_.segments.at(0),
          //                               parameters + knot_num, &J_v);   
        }else{
          // S_ItoG = SO3View::EvaluateRp(t_corrected, spline_meta_.segments.at(0),
          //                 parameters, nullptr);

          // gyro_ = R3View::velocity(t_corrected,
          //                     spline_meta_.segments.at(0),
          //                     parameters, nullptr);  
          vel_ = R3View::evaluate(t_corrected,
                                        linear_spline_meta_.segments.at(0),
                                        parameters, nullptr);                                  
          gyro_ = R3View::evaluate(t_corrected,
                                        angular_spline_meta_.segments.at(0),
                                        parameters + knot_num, nullptr);  

          // rot_accel = SO3View::accelerationBody(t_corrected,  
          //                               spline_meta_.segments.at(0), 
          //                               parameters);
        
          // v_inG = R3View::velocity(t_corrected,
          //                       spline_meta_.segments.at(0),
          //                       parameters + knot_num, nullptr);   
        }
        // LOG(ERROR) << "Set Jacobi Matrix " << std::endl;



        // debug_ceres.debug_file << "Event Details: weight_ = " << weight_ 
                                      // << "\n w_weight_ = [" <<  w_weight_
                                      // // << "\n S_ItoG = [" <<  Output_M(S_ItoG)
                                      // << "\n local_angular_vel_= [" <<  Output(local_angular_vel_) << "] "
                                      // << "\n gyro_ = [" <<  Output(gyro_.transpose()) << "] "
                                      // << "\n angular_bias = [" <<  Output(angular_bias) << "] " << std::endl;

        // LOG(ERROR) << "Event Details: weight_ = " << weight_ 
        //                               << "\n w_weight_ = [" <<  w_weight_
        //                               // << "\n S_ItoG = [" <<  Output_M(S_ItoG)
        //                               << "\n local_angular_vel_= [" <<  Output(local_angular_vel_) << "] "
        //                               << "\n gyro_ = [" <<  Output(gyro_.transpose()) << "] "
        //                               << "\n angular_bias = [" <<  Output(angular_bias) << "] " << std::endl;

      // 3-18 修改 法向光流的计算
      Eigen::Vector3d grad;
      grad << -1.0 / flow_(0), -1.0 / flow_(1), 0.0;
      double normal_norm = 1.0 / grad.norm();
      Eigen::Vector3d normal_flow = grad * normal_norm;

      Eigen::Map<Vec3d const> linear_bias_(parameters[2 * knot_num]);
      Eigen::Map<Vec3d const> omega_bias_(parameters[2 * knot_num + 1]);

      Eigen::Matrix3d R_e_r = q_e_r.toRotationMatrix(); 
      Eigen::Vector3d t_r_e_temp = R_e_r.transpose() * t_e_r;
      Eigen::Vector3d t_r_e = -1.0 * t_r_e_temp;

      // 保留线速度估计 + bias的优化
      Eigen::Vector3d pre_vec = Skew(pixel_cord) * (R_e_r.transpose() * ((vel_ + linear_bias_) + (gyro_ + R_e_r * omega_bias_).cross(t_r_e)));      
      // use for transport
      double post_vec = normal_norm + normal_flow.transpose() * Skew(pixel_cord) * (R_e_r.transpose() * gyro_ + omega_bias_);

      // 保留bias的优化
      // Eigen::Vector3d pre_vec = Skew(pixel_cord) * (R_e_r.transpose() * ((doppler_velocity_ + linear_bias) + (gyro_ + R_e_r * omega_bias).cross(t_r_e)));      
      // // use for transport
      // double post_vec = normal_norm + normal_flow.transpose() * Skew(pixel_cord) * (R_e_r.transpose() * gyro_ + omega_bias);

      // // 去bias
      // Eigen::Vector3d pre_vec = Skew(pixel_cord) * (R_e_r.transpose() * ((doppler_velocity_) + (gyro_).cross(t_r_e)));      
      // pre_vec.normalize();
      // double doppler_scale_aid = pre_vec.norm();
      // // use for transport
      // double post_vec = normal_norm + normal_flow.transpose() * Skew(pixel_cord) * (R_e_r.transpose() * gyro_);

      // 非凸转凸
      // // [1] 使用log估计
      // Eigen::Vector3d log_pre_vec = pre_vec.unaryExpr([](double x) { return std::log(x); });
      // double log_post_vec = std::log(post_vec);

      // [2] 
      /*Skew(pixel_cord) * R_e_r.transpose() * (vel_ + linear_bias_) 
      
      Skew(pixel_cord) * R_e_r.transpose() * (gyro_ + R_e_r * omega_bias_).cross(t_r_e)
      - Skew(pixel_cord) * R_e_r.transpose() * Skew(t_r_e) * (gyro_ + R_e_r * omega_bias_)

      +
      - Skew(pixel_cord) * R_e_r.transpose() * Skew(t_r_e) * gyro_

      +
      - Skew(pixel_cord) * R_e_r.transpose() * Skew(t_r_e) * R_e_r * omega_bias_

      normal_norm 

      + 
      normal_flow.transpose() * Skew(pixel_cord) * (R_e_r.transpose() * gyro_ + omega_bias_)

      normal_flow.transpose() * Skew(pixel_cord) * R_e_r.transpose() * gyro_
      
      +
      normal_flow.transpose() * Skew(pixel_cord) * omega_bias_*/

      /*
      修改问题,非线性优化 -> 线性
      ref f(x) |-> log(1 + f(x))   f(x)|->0 
      */


      LOG(ERROR) << "Event Details: weight_ = " << weight_ 
                   << "\n w_weight_ = " <<  w_weight_
                   << "\n pixel_cord = " <<  Output_M(pixel_cord)
                   << "\n flow_ = " <<  Output(flow_)
                   << "\n doppler_velocity_ = " <<  Output(doppler_velocity_)
                   << "\n vel_ = " <<  Output(vel_)
                  //  << "\n linear_bias = " <<  Output(linear_bias)
                   << "\n R_e_r = " <<  Output_M(R_e_r)
                   << "\n t_r_e = " << Output(t_r_e.transpose()) 
                   << "\n gyro_ = " <<  Output(gyro_) 
                   << "\n normal_norm = " <<  normal_norm 
                   << "\n normal_flow = " <<  Output(normal_flow) 
                   << "\n linear_bias_ = " <<  Output(linear_bias_) 
                   << "\n omega_bias_ = " <<  Output(omega_bias_) 
                  //  << "\n omega_bias = " <<  Output(omega_bias) 
                   << std::endl;
      LOG(ERROR) << "pre_vec = " << Output(pre_vec)
                 << "\n post_vec = " << post_vec << std::endl;

      // LOG(ERROR) << "post_vec = " << post_vec << std::endl;

        Eigen::Map<Eigen::Vector3d> residual(residuals);
        residual.setZero();
        Eigen::Vector3d residual_temp = pre_vec * post_vec;
        residual = residual_temp;
        // residual = (weight_ * w_weight_ * residual).eval();
        residual = (w_weight_ * residual).eval();

        // double residual = post_vec;
        // residuals[0] = residual;


        // LOG(ERROR) << "local angular velocity residuals = " 
        //                         << (residual) << std::endl;

        LOG(ERROR) << "local angular velocity residuals = " 
                                << Output_M(residual) << std::endl;

        // 不评估雅可比就返回
        if (!jacobians) {
          // LOG(ERROR) << "BodyLocalAngularVelocityFactor No J" << std::endl;
          // debug_ceres.debug_file << "BodyLocalAngularVelocityFactor No Jacobi!" << std::endl;
          // debug_ceres.Close();

          // // debug_ceres_jacobis.Close();
          return true;
        }
        // debug_ceres_jacobis.Open();
        // debug_ceres_jacobis.debug_file << "Evaluate EventAgularFactor Res J_R_0 J_R_1 J_R_2 J_R_3 J_p_0 J_p_1 J_p_2 J_p_3 J_linear_bias J_angular_bias J_t \n";
        // debug_ceres_jacobis.debug_file << Output(residual_temp.transposacobi!e()) << std::endl;

        // [0] 对 kont_v 的 雅可比
        Eigen::Matrix3d Jac_vel_pre = Skew(pixel_cord) * R_e_r.transpose();
        Eigen::Matrix<double, 12, 3> Jac_v;
        for (size_t i = 0; i < knot_num; i++) {
          size_t idx = i;
          if (jacobians[idx]) {
            Eigen::Map<Eigen::Matrix3d> jac_kont_v(
                jacobians[idx]);
            jac_kont_v.setZero();

            // jac_kont_p = S_ItoG.matrix().transpose() * J_v.d_val_d_knot[i];
            // jac_kont_p = (weight * jac_kont_p).eval();
            Eigen::Matrix3d Jac_vel_ = Jac_vel_pre * J_v_.d_val_d_knot[i] * post_vec;
            jac_kont_v = (weight_ * Jac_vel_).eval();

            Eigen::Matrix3d jac_kont_v_copy = jac_kont_v;

            LOG(ERROR) << "Jac_vel_pre = " << Output_M(Jac_vel_pre) << std::endl;
            LOG(ERROR) << "J_v_.d_val_d_knot[i] = " << J_v_.d_val_d_knot[i] << std::endl;
            LOG(ERROR) << "post_vec = " << post_vec << std::endl;
            LOG(ERROR) << "jac_kont_v_copy = " << Output_M(jac_kont_v_copy) << std::endl;

            // LOG(ERROR) << "J_w_.d_val_d_knot[" << i << "] = [" << J_w_.d_val_d_knot[i] << "]\n";
            // // debug_ceres.debug_file << "J_v.d_val_d_knot[" << i << "] = [" << Output_M(J_v.d_val_d_knot[i]) << "]\n";
            // // debug_ceres.debug_file << "jac_lhs_R_" << i << " = [" << Output_M(jac_kont_p_copy) << "]\n";

            Jac_v.block<3,3>(3 * i, 0) = jac_kont_v_copy;

            // debug_ceres_jacobis.debug_file << Output_M(jac_kont_p)  << std::endl;

          }
        }


        // [1] 对 kont_w 的 雅可比
        // Eigen::Matrix3d Jac_gyro_pre = - Skew(pixel_cord) * R_e_r.transpose() * Skew(t_r_e);
        // // 前一项固定,后一项优化
        // Eigen::Matrix3d Jac_gyro_pre;
        // Jac_gyro_pre.setZero();
        // Eigen::RowVector3d Jac_gyro_post = normal_flow.transpose() * Skew(pixel_cord) * R_e_r.transpose();
        // LOG(ERROR) << "Jac_gyro_pre = " << Output_M(Jac_gyro_pre) << std::endl;
        // LOG(ERROR) << "Jac_gyro_post = " << Output_M(Jac_gyro_post) << std::endl;

        // 只要最后一项的优化
        // Eigen::Matrix<double, 4, 3> Jac_w;
        // for (size_t i = 0; i < knot_num; i++) {
        //   size_t idx = i;
        //   if (jacobians[idx]) {
        //     Eigen::Map<Eigen::RowVector3d> jac_kont_w(
        //         jacobians[idx]);
        //     jac_kont_w.setZero();

        //     // jac_kont_p = S_ItoG.matrix().transpose() * J_v.d_val_d_knot[i];
        //     // jac_kont_p = (weight * jac_kont_p).eval();
        //     Eigen::RowVector3d Jac_gyro_ = Jac_gyro_post * J_w_.d_val_d_knot[i];
        //     jac_kont_w = (weight_ * Jac_gyro_).eval();

        //     Eigen::RowVector3d jac_kont_w_copy = jac_kont_w;

        //     // LOG(ERROR) << "J_w_.d_val_d_knot[" << i << "] = [" << J_w_.d_val_d_knot[i] << "]\n";
        //     // // debug_ceres.debug_file << "J_v.d_val_d_knot[" << i << "] = [" << Output_M(J_v.d_val_d_knot[i]) << "]\n";
        //     // // debug_ceres.debug_file << "jac_lhs_R_" << i << " = [" << Output_M(jac_kont_p_copy) << "]\n";

        //     Jac_w.block<1,3>(i, 0) = jac_kont_w_copy;

        //     // debug_ceres_jacobis.debug_file << Output_M(jac_kont_p)  << std::endl;

        //   }
        // }
        // LOG(ERROR) << "Add jacobians for Position control point " << std::endl;  
 
        Eigen::Matrix3d Jac_gyro_pre = - Skew(pixel_cord) * R_e_r.transpose() * Skew(t_r_e);
        Eigen::RowVector3d Jac_gyro_post = normal_flow.transpose() * Skew(pixel_cord) * R_e_r.transpose();
        Eigen::Matrix<double, 12, 3> Jac_w;
        for (size_t i = 0; i < knot_num; i++) {
          size_t idx = i + knot_num;
          if (jacobians[idx]) {
            Eigen::Map<Eigen::Matrix3d> jac_kont_w(
                jacobians[idx]);
            jac_kont_w.setZero();

            // jac_kont_p = S_ItoG.matrix().transpose() * J_v.d_val_d_knot[i];
            // jac_kont_p = (weight * jac_kont_p).eval();
            Eigen::Matrix3d Jac_gyro_ = Jac_gyro_pre * J_w_.d_val_d_knot[i] * post_vec 
                                          + pre_vec * Jac_gyro_post * J_w_.d_val_d_knot[i];
            jac_kont_w = (weight_ * Jac_gyro_).eval();

            Eigen::Matrix3d jac_kont_w_copy = jac_kont_w;

            LOG(ERROR) << "Jac_gyro_pre = " << Output_M(Jac_gyro_pre) << std::endl;
            LOG(ERROR) << "J_w_.d_val_d_knot[i] = " << J_w_.d_val_d_knot[i] << std::endl;
            LOG(ERROR) << "Jac_gyro_post = " << Output(Jac_gyro_post) << std::endl;
            LOG(ERROR) << "jac_kont_w = " << Output_M(Jac_gyro_post) << std::endl;

            // LOG(ERROR) << "J_w_.d_val_d_knot[" << i << "] = [" << J_w_.d_val_d_knot[i] << "]\n";
            // // debug_ceres.debug_file << "J_v.d_val_d_knot[" << i << "] = [" << Output_M(J_v.d_val_d_knot[i]) << "]\n";
            // // debug_ceres.debug_file << "jac_lhs_R_" << i << " = [" << Output_M(jac_kont_p_copy) << "]\n";

            Jac_w.block<3,3>(3 * i, 0) = jac_kont_w_copy;

            // debug_ceres_jacobis.debug_file << Output_M(jac_kont_p)  << std::endl;

          }
        }

        // [2] 对 linear_bias 的 雅可比
        Eigen::Matrix3d Jac_linear_bias_pre_ = Skew(pixel_cord) * R_e_r.transpose();
        Eigen::Matrix3d Jac_linear_bias_ = Jac_linear_bias_pre_ * post_vec;
        // LOG(ERROR) << "Jac_linear_bias_pre_ = " << Output_M(Jac_linear_bias_pre_) << std::endl;
        // LOG(ERROR) << "Jac_linear_bias_ = " << Output_M(Jac_linear_bias_) << std::endl;
        Eigen::Matrix3d J_linear_bias_copy;
        if(jacobians[2 * knot_num])
        {
          // Eigen::Map<Eigen::Matrix<double, 1, 3>> J_linear_bias_(jacobians[2 * knot_num]); 
          Eigen::Map<Eigen::Matrix<double, 3, 3>> J_linear_bias_(jacobians[2 * knot_num]); 
          J_linear_bias_.setZero();

          // J_linear_bias_ = (weight_ * 0.01 * Jac_linear_bias_).eval();
          J_linear_bias_ = (weight_ * 0.01 * Jac_linear_bias_).eval();

          J_linear_bias_copy = J_linear_bias_;

          // debug_ceres.debug_file << "J_angular_bias_ = " << Output_M(jac_angular_copy) << "]\n";


          // debug_ceres_jacobis.debug_file << Output_M(jac_angular_copy) << std::endl;
        }

        // [3] 对 angular_bias 的 雅可比
        Eigen::Matrix3d Jac_omega_bias_pre_ = - Skew(pixel_cord) * R_e_r.transpose() * Skew(t_r_e) * R_e_r;
        Eigen::RowVector3d Jac_omega_bias_post_ = normal_flow.transpose() * Skew(pixel_cord);
        Eigen::Matrix3d Jac_omega_bias_ = Jac_omega_bias_pre_ * post_vec + pre_vec * Jac_omega_bias_post_;
        // LOG(ERROR) << "Jac_omega_bias_pre_ = " << Output_M(Jac_omega_bias_pre_) << std::endl;
        // LOG(ERROR) << "Jac_omega_bias_post_ = " << Output(Jac_omega_bias_post_) << std::endl;
        // LOG(ERROR) << "Jac_omega_bias_ = " << Output_M(Jac_omega_bias_) << std::endl;
        Eigen::Matrix3d J_angular_bias_copy;
        if(jacobians[2 * knot_num + 1])
        {
          // Eigen::Map<Eigen::Matrix<double, 1, 3>> J_linear_bias_(jacobians[2 * knot_num]); 
          Eigen::Map<Eigen::Matrix<double, 3, 3>> J_angular_bias_(jacobians[2 * knot_num + 1]); 
          J_angular_bias_.setZero();

          // J_angular_bias_ = (weight_ * 0.01 * Jac_omega_bias_).eval();
          J_angular_bias_ = (weight_ * Jac_omega_bias_).eval();

          // debug_ceres.debug_file << "J_angular_bias_ = " << Output_M(jac_angular_copy) << "]\n";
          J_angular_bias_copy = J_angular_bias_;

          // debug_ceres_jacobis.debug_file << Output_M(jac_angular_copy) << std::endl;
        }
        // LOG(ERROR) << "Add jacobians for Angular velocity bias " << std::endl;  
        

        // LOG(ERROR) << "All Jacobi for BodyLocalAngularVelocity factor" << std::endl;
        // debug_ceres.debug_file << "All Jacobi for BodyLocalAngularVelocity factor" << std::endl;
        // debug_ceres.debug_file << "Jac_R_ = " << Output_M(Jac_R) << std::endl;
        // debug_ceres.debug_file << "Jac_p_ = " << Output_M(Jac_p) << std::endl;
        // debug_ceres.debug_file << "J_angular_bias_ = " << Output_M(Jac_w_bias) << std::endl;

        LOG(ERROR) << "Jac_v = " << Output_M(Jac_v) << std::endl;
        LOG(ERROR) << "Jac_w = " << Output_M(Jac_w) << std::endl;
        LOG(ERROR) << "J_linear_bias_ = " << Output_M(J_linear_bias_copy) << std::endl;
        LOG(ERROR) << "J_angular_bias_ = " << Output_M(J_angular_bias_copy) << std::endl;


        // debug_ceres.Close();
        // debug_ceres_jacobis.Close();

        return true;
    }
  
private:
    int64_t time_ns_;
    Eigen::Vector3d pixel_cord;
    Eigen::Vector3d flow_;
    Eigen::Vector3d doppler_velocity_;
    Eigen::Quaterniond q_e_r;
    Eigen::Vector3d t_e_r;
    SplineMeta<SplineOrder> linear_spline_meta_;
    SplineMeta<SplineOrder> angular_spline_meta_;
    // Vec6d info_vec_;
    // Eigen::Vector3d linear_bias_;
    // Eigen::Vector3d omega_bias_;
    bool lock_extrincs;
    double weight_;
    // 某些参数的弱优化权重
    double w_weight_;
};


class ImuFactor : public ceres::CostFunction, SplitSpineView{
 public:
 EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  using SO3View = So3SplineView;
  using R3View = RdSplineView;
  using SplitView = SplitSpineView;

  using Vec3d = Eigen::Matrix<double, 3, 1>;
  using Vec6d = Eigen::Matrix<double, 6, 1>;
  using Mat3d = Eigen::Matrix<double, 3, 3>;
  using Quatd = Eigen::Quaterniond;
  using SO3d = Sophus::SO3<double>;

  // Ri 旋转矩阵起点
  // 线速度
  // 角速度

  ImuFactor(int64_t time_ns, const Eigen::Matrix3d& Ri_in_world, 
            const Eigen::Vector3d& imu_acc,
            const Eigen::Vector3d& imu_gyr,
            // const Eigen::Vector3d& acc_bias,     // 与doppler bias 没有联系
            // const Eigen::Vector3d& gyro_bias,
            // const Eigen::Vector3d& gravity,
            const double* time_offset,
            const SplineMeta<SplineOrder>& linear_spline_segment_meta,
            const SplineMeta<SplineOrder>& angular_spline_segment_meta,
            std::shared_ptr<FEJ_STATE> global_fej_state, bool use_fej,
            // const Eigen::Vector3d linear_bias, const Eigen::Vector3d angular_bias,
            double weight, double w_weight = 0.05) // const Vec6d& info_vec)
      : time_ns_(time_ns), Ri_in_world_(Ri_in_world_),
        imu_acc_(imu_acc), imu_gyr_(imu_gyr),
        // acc_bias_(acc_bias), gyro_bias_(gyro_bias),
        // g_(gravity), 
        time_offset_(*time_offset),
        linear_spline_meta_(linear_spline_segment_meta),
        angular_spline_meta_(angular_spline_segment_meta),
        global_fej_state_(global_fej_state),
        use_fej_(use_fej),
        // linear_bias_(linear_bias), omega_bias_(angular_bias),
        lock_extrincs(true),      // HAO TODO: 暂时不优化外参
        weight_(weight), w_weight_(w_weight)
        {
          // set_num_residuals(1);           // 定义残差值的大小(事件角速度残差)
          set_num_residuals(6);           // 定义残差值的大小(事件角速度残差)

          size_t knot_num = this->angular_spline_meta_.NumParameters();

          for (size_t i = 0; i < knot_num; ++i) {
            mutable_parameter_block_sizes()->push_back(3);   // linear velocity
          }

          // TODO: 需要角速度曲线              
          for (size_t i = 0; i < knot_num; ++i) {
            mutable_parameter_block_sizes()->push_back(3);   // angular velocity
          }

          mutable_parameter_block_sizes()->push_back(3);    // acc bias
          
          mutable_parameter_block_sizes()->push_back(3);    // gyo bias

          mutable_parameter_block_sizes()->push_back(3);    // gravity
        }

  virtual bool Evaluate(double const* const* parameters, double* residuals,
                          double** jacobians) const {
        // typename R3View::JacobianStruct J_v_;
        typename R3View::JacobianStruct J_a_;
        typename R3View::JacobianStruct J_w_;

        std::chrono::time_point<std::chrono::high_resolution_clock> time1;

        LOG(ERROR) << "Evaluate Imu " << std::endl;
        // debug_ceres.Open();
        // debug_ceres.debug_file << std::endl;
        // debug_ceres.debug_file << "    ---------------------- Evaluate BodyLocalAngularVelocityFactor ------------------    " << std::endl;

        size_t knot_num = angular_spline_meta_.NumParameters();

        // double time_offset_in_ns = parameters[t_offset_index][0];
        double time_offset_in_ns = 0;
        int64_t t_corrected = time_ns_ + (int64_t)time_offset_in_ns;
        t_corrected = std::min(t_corrected, angular_spline_meta_.segments.at(0).MaxTimeNs() - 1);
        t_corrected = std::max(t_corrected, angular_spline_meta_.segments.at(0).MinTimeNs() + 1);
  
        // Eigen::Vector3d vel_, gyro_, rot_accel;
        Eigen::Vector3d acc_, gyro_, rot_accel;
        if (jacobians){
          acc_ = R3View::evaluate(t_corrected,
                                        linear_spline_meta_.segments.at(0),
                                        parameters, &J_a_); 

          gyro_ = R3View::evaluate(t_corrected,
                                        angular_spline_meta_.segments.at(0),
                                        parameters + knot_num, &J_w_);   
        }else{
          acc_ = R3View::evaluate(t_corrected,
                                        linear_spline_meta_.segments.at(0),
                                        parameters, nullptr); 

          gyro_ = R3View::evaluate(t_corrected,
                                        angular_spline_meta_.segments.at(0),
                                        parameters + knot_num, nullptr);  
        }

        Eigen::Map<Vec3d const> acc_bias_(parameters[2 * knot_num]);
        Eigen::Map<Vec3d const> gyro_bias_(parameters[2 * knot_num + 1]);
        Eigen::Map<Vec3d const> g_(parameters[2 * knot_num + 2]);

      Eigen::Map<Eigen::Matrix<double,6,1>> residual(residuals);
      residual.setZero();
      residual.block<3,1>(0,0) = acc_ - (Ri_in_world_ * (imu_acc_ - acc_bias_) - g_);
      residual.block<3,1>(3,0) = gyro_ - (imu_gyr_  - gyro_bias_);

      // Ri_in_world = Utility::deltaQ(un_gyr * dt).toRotationMatrix();

      residual = (weight_ * residual).eval();

      // 不评估雅可比就返回
      if (!jacobians) {
        // LOG(ERROR) << "BodyLocalAngularVelocityFactor No J" << std::endl;
        // debug_ceres.debug_file << "BodyLocalAngularVelocityFactor No Jacobi!" << std::endl;
        // debug_ceres.Close();

        // // debug_ceres_jacobis.Close();
        return true;
      }

      // 加入FEJ状态
      /*if(global_fej_state_ && use_fej_)
      {
        pre_vec = Skew(pixel_cord) * (R_e_r.transpose() * ((global_fej_state_->linear_velocity_
                    + global_fej_state_->linear_bias_) + (global_fej_state_->angular_velocity_ 
                    + R_e_r * global_fej_state_->angular_bias_).cross(t_r_e)));      
        post_vec = normal_norm + normal_flow.transpose() * Skew(pixel_cord) * (R_e_r.transpose() 
                    * global_fej_state_->angular_velocity_ + global_fej_state_->angular_bias_);
        
        LOG(ERROR) << "FEJ: pre_vec = " << Output(pre_vec)
                << "\n FEJ: post_vec = " << post_vec << std::endl;
      }  */
      /*if(global_fej_state_)
      {
        LOG(ERROR) << "Use fej state = \n"
                << << global_fej_state_-> << "\n"
              << "doppler_ = " << doppler_ << "\n"
              << "pt_.normalized() = " << Output(pt_.normalized()) << "\n"
              << "v_inG = " << Output(v_inG) << "\n" 
              << "linear_bias = " << Output(linear_bias) << "\n"  
              << std::endl;
      }*/

      LOG(ERROR) << "local imu residuals = " 
                              << Output_M(residual) << std::endl;
      LOG(ERROR) << "local imu residuals norm = " 
                              << residual.norm() << std::endl;
      // LOG(ERROR) << "Event Loss: " << ((residual.norm() > 0.3)? "True": "False") << std::endl; 

      /*std::fstream imu_file("/home/hao/Desktop/twist_ws/src/TwistEstimator/output/imu_res.txt",
                                std::ios::out | std::ios::app);*/
      // angular_file << Output_M(residual) << std::endl;
      // imu_file << residual.norm() << std::endl;
      // imu_file.close();


      std::chrono::time_point<std::chrono::high_resolution_clock> time3;

      Eigen::Matrix3d iden_matrix;
      iden_matrix.setIdentity();
      // 加速度
      Eigen::Matrix<double, 12, 3> Jac_a;
      for (size_t i = 0; i < knot_num; i++) {
        size_t idx = i;
        if (jacobians[idx]) {
          Eigen::Map<Eigen::Matrix3d> jac_kont_a(
              jacobians[idx]);
          jac_kont_a.setZero();
          Eigen::Matrix3d jac_a = iden_matrix * J_a_.d_val_d_knot[i];
          jac_kont_a = (weight_ * jac_a).eval();
        }
      }

      // 角速度
      for (size_t i = knot_num; i < 2 * knot_num; i++) {
        size_t idx = i;
        if (jacobians[idx]) {
          Eigen::Map<Eigen::Matrix3d> jac_kont_w(
              jacobians[idx]);
          jac_kont_w.setZero();
          Eigen::Matrix3d jac_w = iden_matrix * J_w_.d_val_d_knot[i - knot_num];
          jac_kont_w = (weight_ * jac_w).eval();
        }
      }
      
      // 加速度偏置
      if(jacobians[2 * knot_num])
      {
        Eigen::Map<Eigen::Matrix<double, 3, 3>> J_acc_bias_(jacobians[2 * knot_num]); 
        J_acc_bias_.setZero();
        J_acc_bias_ = (weight_ * Ri_in_world_).eval(); 
      }

      // 角速度偏置
      if(jacobians[2 * knot_num + 1])
      {
        Eigen::Map<Eigen::Matrix<double, 3, 3>> J_gyr_bias_(jacobians[2 * knot_num + 1]);
        J_gyr_bias_.setZero(); 

        J_gyr_bias_ = (weight_ * Ri_in_world_).eval(); 
      }

      // 重力
      if(jacobians[2 * knot_num + 2])
      {
        Eigen::Map<Eigen::Matrix<double, 3, 3>> J_grav_(jacobians[2 * knot_num + 2]);
        J_grav_.setIdentity(); 
      }

  
      return true;
    }
  
private:
    int64_t time_ns_;

    const Eigen::Matrix3d Ri_in_world_; 
    const Eigen::Vector3d imu_acc_;
    const Eigen::Vector3d imu_gyr_;
    // const Eigen::Vector3d acc_bias_;     // 与doppler bias 没有联系
    // const Eigen::Vector3d gyro_bias_;
    // const Eigen::Vector3d g_;
    const double time_offset_;

    SplineMeta<SplineOrder> linear_spline_meta_;
    SplineMeta<SplineOrder> angular_spline_meta_;
    // Vec6d info_vec_;
    // Eigen::Vector3d linear_bias_;
    // Eigen::Vector3d omega_bias_;
    bool lock_extrincs;
    std::shared_ptr<FEJ_STATE> global_fej_state_;
    bool use_fej_;
    double weight_;
    // 某些参数的弱优化权重
    double w_weight_;
};  // ImuFactor

