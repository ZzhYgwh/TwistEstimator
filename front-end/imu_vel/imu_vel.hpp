
#include "imu_vel/integration_base.h"


struct IMUVel
{
    IMUVel(const Vector3d &linear_, 
                            const Vector3d &angular_)
    {
        linear = linear_;
        angular = angular_;
    }
    Eigen::Vector3d linear;
    Eigen::Vector3d angular;
}

std::vector<IMUVel> imu_vel_vec;
void Estimator::processIMU(double t, double dt, 
                            const Vector3d &acc_bias, 
                            const Vector3d &gyr_bias, 
                            const Vector3d &linear_acceleration, 
                            const Vector3d &angular_velocity, 
                            const Eigen::Matrix3d& Ri_in_world)
{
    if (!first_imu)
    {
        first_imu = true;
        acc_0 = linear_acceleration;
        gyr_0 = angular_velocity;
    }

    if (!pre_integrations[frame_count])
    {
        pre_integrations[frame_count] = new IntegrationBase{acc_0, gyr_0, acc_bias, gyr_bias};
    }
    if (frame_count != 0)
    {
        pre_integrations[frame_count]->push_back(dt, linear_acceleration, angular_velocity);
        //if(solver_flag != NON_LINEAR)
            tmp_pre_integration->push_back(dt, linear_acceleration, angular_velocity);

        dt_buf[frame_count].push_back(dt);
        linear_acceleration_buf[frame_count].push_back(linear_acceleration);
        angular_velocity_buf[frame_count].push_back(angular_velocity);

        int j = frame_count;         
        Vector3d un_acc_0 = Ri_in_world * (acc_0 - acc_bias) - g;
        Vector3d un_gyr = 0.5 * (gyr_0 + angular_velocity) - gyr_bias;
        Ri_in_world *= Utility::deltaQ(un_gyr * dt).toRotationMatrix();
        Eigen::Vector3d un_acc_1 = Ri_in_world * (linear_acceleration - acc_bias) - g;
        Eigen::Vector3d un_acc = 0.5 * (un_acc_0 + un_acc_1);  // 直接使用加速度误差
        // Ps[j] += dt * Vs[j] + 0.5 * dt * dt * un_acc;
        // Vs[j] += dt * un_acc;

        // output
        imu_vel_vec.push_back(IMUVel(un_acc, un_gyr));
    }
    acc_0 = linear_acceleration;
    gyr_0 = angular_velocity; 
}