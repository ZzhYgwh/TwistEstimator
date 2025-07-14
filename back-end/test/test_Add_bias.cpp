
#include "estimator/TwistEstimator.h"



int main(int argc, char ** argv)
{
    double last_time = 0;
    double cur_time = 1;
    std::map<double, TwistBias> all_twist_bias_;
    TwistBias bias;
    bias.omega_bias << 1, 1, 1;
    bias.vel_bias << 1, 1, 1;

    all_twist_bias_[last_time] = bias;

    bias.omega_bias = 2.0 * bias.omega_bias;
    bias.vel_bias = 2.0 * bias.vel_bias;
    all_twist_bias_[cur_time] = bias;



    // 进行深拷贝，确保新内存空间
    Eigen::Vector3d last_omega_bias_copy = all_twist_bias_.at(last_time).omega_bias;
    Eigen::Vector3d cur_omega_bias_copy = all_twist_bias_.at(cur_time).omega_bias;

    Eigen::Vector3d last_vel_bias_copy = all_twist_bias_.at(last_time).vel_bias;
    Eigen::Vector3d cur_vel_bias_copy = all_twist_bias_.at(cur_time).vel_bias;

    // 或者你可以将 `copy` 版本的数据指向新的内存
    // 比如将 copy 后的数据存储到新的指针中
    double* bias_gyr_i = last_omega_bias_copy.data();
    double* bias_gyr_j = cur_omega_bias_copy.data();

    double* bias_acc_i = last_vel_bias_copy.data();
    double* bias_acc_j = cur_vel_bias_copy.data();


    LOG(ERROR) << "check bias parameters " << std::endl;

    if(bias_gyr_i == nullptr)
      LOG(ERROR) << "bias_gyr_i is empty" << std::endl;
    if(bias_gyr_j == nullptr)
      LOG(ERROR) << "bias_gyr_j is empty" << std::endl;
    if(bias_acc_i == nullptr)
      LOG(ERROR) << "bias_acc_i is empty" << std::endl;
    if(bias_acc_j == nullptr)
      LOG(ERROR) << "bias_acc_j is empty" << std::endl;

    LOG(ERROR) << "bias_gyr_i = [" << bias_gyr_i[0] << ", " << bias_gyr_i[1] << ", " << bias_gyr_i[2] << "] " << std::endl;
    LOG(ERROR) << "bias_gyr_j = [" << bias_gyr_j[0] << ", " << bias_gyr_j[1] << ", " << bias_gyr_j[2] << "] " << std::endl;
    LOG(ERROR) << "bias_acc_i = [" << bias_acc_i[0] << ", " << bias_acc_i[1] << ", " << bias_acc_i[2] << "] " << std::endl;
    LOG(ERROR) << "bias_acc_j = [" << bias_acc_j[0] << ", " << bias_acc_j[1] << ", " << bias_acc_j[2] << "] " << std::endl;
  
    Eigen::Map<Eigen::Vector3d const> bias_w_i(bias_gyr_i);
    Eigen::Map<Eigen::Vector3d const> bias_w_j(bias_gyr_j);
    Eigen::Map<Eigen::Vector3d const> bias_v_i(bias_acc_i);
    Eigen::Map<Eigen::Vector3d const> bias_v_j(bias_acc_j);

    // Eigen::Map<Eigen::Vector3d const> bias_w_i(vec[0]);
    // Eigen::Map<Eigen::Vector3d const> bias_w_j(vec[1]);
    // Eigen::Map<Eigen::Vector3d const> bias_v_i(vec[2]);
    // Eigen::Map<Eigen::Vector3d const> bias_v_j(vec[3]);

    LOG(ERROR) << "check bias parameters " << std::endl;

    // LOG(ERROR) << "Get All Bias Parameters " << std::endl;
    LOG(ERROR) << "bias_w_i: " << bias_w_i << std::endl;
    LOG(ERROR) << "bias_w_i: " << bias_w_i.transpose() << std::endl;
    LOG(ERROR) << "bias_w_j: " << bias_w_j.transpose() << std::endl;
    LOG(ERROR) << "bias_v_i: " << bias_v_i.transpose() << std::endl;
    LOG(ERROR) << "bias_v_j: " << bias_v_j.transpose() << std::endl;
    LOG(ERROR) << "bias_w_j - bias_w_i: " << (bias_w_j - bias_w_i) << std::endl;
    LOG(ERROR) << "bias_w_j - bias_w_i: " << (bias_w_j - bias_w_i).transpose();
    LOG(ERROR) << "bias_v_j - bias_v_i: " << (bias_v_j - bias_v_i).transpose();

    return 0;
}