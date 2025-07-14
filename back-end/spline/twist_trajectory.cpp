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

#include "twist_trajectory.h"
#include <fstream>


// void Twist_Trajectory::GetIMUState(double time, IMUState &imu_state) const {
//   SE3d pose = GetRadarPose(time);

//   imu_state.timestamp = time;
//   imu_state.q = pose.unit_quaternion();
//   imu_state.p = pose.translation();
//   imu_state.v = GetTransVelWorld(time);
//   // imu_state.bias;
//   // imu_state.g;
// }

// void Twist_Trajectory::UndistortScan(const PosCloud &scan_raw,


// void Twist_Trajectory::UndistortScanInG(const PosCloud &scan_raw,


Eigen::Matrix<double, 6, 1> Twist_Trajectory::GetSensorVel(const double timestamp,
                               const ExtrinsicParam &EP_StoI) const {
  double time_ns = timestamp * S_TO_NS + EP_StoI.t_offset_ns;

  if (!(time_ns >= this->minTimeNs() && time_ns < this->maxTimeNs())) {
    std::cout << time_ns << "; not in [" << this->minTimeNs() << ", "
              << this->maxTimeNs() << "]; "
              << "input time: " << timestamp
              << "[s]; t_offset: " << EP_StoI.t_offset_ns << " [ns]\n";
  }
  assert(time_ns >= this->minTimeNs() && time_ns < this->maxTimeNs() &&
         "[GetSensorPose] querry time not in range.");

  Eigen::Matrix<double, 6, 6> R_StoI;
  R_StoI.setZero();
  R_StoI.block<3,3>(0,0) = EP_StoI.Get_R();
  R_StoI.block<3,3>(3,3) = EP_StoI.Get_R();

  Eigen::Matrix<double, 6, 1> pose_I_to_G = this->poseNs(time_ns);
  Eigen::Matrix<double, 6, 1> pose_S_to_G = R_StoI * pose_I_to_G;
  return pose_S_to_G;
}

/*
void Twist_Trajectory::ToTUMTxt(std::string traj_path, double dt) {
  std::ofstream outfile;
  outfile.open(traj_path);
  outfile.setf(std::ios::fixed);

  double min_time = minTime(RadarSensor);
  double max_time = maxTime(RadarSensor);
  for (double t = min_time; t < max_time; t += dt) {
    SE3d pose = GetRadarPose(t);
    Eigen::Vector3d p = pose.translation();
    Eigen::Quaterniond q = pose.unit_quaternion();

    /// uncomment this line for VIRAL dataset to align with gt,  extrinsic is from leica_prism.yaml
    p = (q.toRotationMatrix() * Eigen::Vector3d(-0.293656, -0.012288, -0.273095) + p).eval();

    double relative_bag_time = data_start_time_ + t;
    outfile.precision(9);
    outfile << relative_bag_time << " ";
    outfile.precision(5);
    outfile << p(0) << " " << p(1) << " " << p(2) << " " << q.x() << " "
            << q.y() << " " << q.z() << " " << q.w() << "\n";
  }
  outfile.close();
  std::cout << "Save trajectory at " << traj_path << std::endl;
}

*/
