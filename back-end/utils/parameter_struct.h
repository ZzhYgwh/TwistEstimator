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

// #include <yaml-cpp/yaml.h>
#include <glog/logging.h>
#include "yaml_utils.h"
#include "sophus_utils.hpp"



#define RESET "\033[0m"
#define BLACK "\033[30m"   /* Black */
#define RED "\033[31m"     /* Red */
#define GREEN "\033[32m"   /* Green */
#define YELLOW "\033[33m"  /* Yellow */
#define BLUE "\033[34m"    /* Blue */
#define MAGENTA "\033[35m" /* Magenta */
#define CYAN "\033[36m"    /* Cyan */
#define WHITE "\033[37m"   /* White */

extern double GRAVITY_NORM;

enum MODE {
  Odometry_Offline = 1,  //
  Odometry_Online,       //
};

struct VecData {
  double timestamp;
  Eigen::Vector3d p;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

struct IMUData {
  double timestamp;
  Eigen::Vector3d gyro;
  Eigen::Vector3d accel;
  SO3d orientation;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

struct IMUBias {
  IMUBias()
      : gyro_bias(Eigen::Vector3d::Zero()),
        accel_bias(Eigen::Vector3d::Zero()) {}
  Eigen::Vector3d gyro_bias;
  Eigen::Vector3d accel_bias;
};

struct IMUState {
  IMUState()
      : timestamp(0),
        p(Eigen::Vector3d::Zero()),
        v(Eigen::Vector3d::Zero()),
        g(Eigen::Vector3d(0, 0, -9.8)) {}
  double timestamp;
  Eigen::Vector3d p;  // global frame
  Eigen::Vector3d v;  // global frame
  Eigen::Quaterniond q;
  IMUBias bias;
  Eigen::Vector3d g;
};

struct PoseData {
  PoseData() : timestamp(0), position(Eigen::Vector3d::Zero()) {}

  double timestamp;
  Eigen::Vector3d position;
  SO3d orientation;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

struct SystemState : public IMUState {
  SystemState() : name("") {}

  SystemState(const IMUState& imu) {
    timestamp = imu.timestamp;
    p = imu.p;
    v = imu.v;
    q = imu.q;
    bias = imu.bias;
    g = imu.g;

    name = "";
  }

  std::string name;
};

struct ExtrinsicParam {
  ExtrinsicParam()
      : p(Eigen::Vector3d::Zero()),
        q(Eigen::Quaterniond::Identity()),
        t_offset_ns(0) {}

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

  // node["Extrinsics"]
  void Init(const YAML::Node& node) {
    // std::cout << node.as<std::string>() << ": " << std::endl;
    // std::cout << "Node List: " << std::endl;
    // printYamlNode(node);
    // for (std::size_t i = 0; i < node.size(); ++i) {
    //     std::cout << node[i].as<std::string>() << std::endl;
    // }


    if (!(node["time_offset"]))
      LOG(WARNING) << "missing time_offset" << std::endl;
    else if(!(node["Trans"]))
      LOG(WARNING) << "missing Trans" << std::endl;
    else if(!(node["Rot"]))
      LOG(WARNING) << "missing Rot" << std::endl;

    if (!(node["time_offset"] && node["Trans"] && node["Rot"])) {
      LOG(WARNING)
          << "[ExtrinsicParam::Init] input yaml node has not parameters "
             "of Extrinsics struct. Return without Initialziation.";
      return;
    }

    double t_s = yaml::GetValue<double>(node, "time_offset", 0);
    std::cout << "set t_s = " << t_s << std::endl;
    t_offset_ns = t_s * 1e9;
    std::vector<double> params_vec;
    yaml::GetValues<double>(node, "Trans", 3, params_vec);
    p << params_vec[0], params_vec[1], params_vec[2];

    yaml::GetValues<double>(node, "Rot", 9, params_vec);
    Eigen::Matrix3d rot;
    rot << params_vec[0], params_vec[1], params_vec[2], params_vec[3],
        params_vec[4], params_vec[5], params_vec[6], params_vec[7],
        params_vec[8];

    q = Eigen::Quaterniond(rot);
    q.normalized();
    UpdateGroupParam();
  }

  // HAO TODO:
  // 将ExtrinsicParam转为齐次变换矩阵
  Eigen::Matrix4d Get_T() {
      // 从四元数获取旋转矩阵
      q = q.normalized();
      Eigen::Matrix3d R = q.toRotationMatrix();
      
      // 创建齐次变换矩阵
      Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
      T.block<3, 3>(0, 0) = R;    // 将旋转矩阵放入左上角
      T.block<3, 1>(0, 3) = p; // 将平移向量放入最后一列
      
      return T;
  }

  Eigen::Matrix3d Get_R() const
  {
    return q.toRotationMatrix();
  }

  // 获取传感器的时间延迟
  double Get_Timeoffset()
  {
    return t_offset_ns * 1e-9;
  }


  void UpdateGroupParam() {
    so3 = SO3d(q);
    se3 = SE3d(so3, p);
  }

  Eigen::Vector3d p;
  SO3d so3;            
  Eigen::Quaterniond q;
  SE3d se3;
  double t_offset_ns;
};


