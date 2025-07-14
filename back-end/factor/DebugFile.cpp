#ifndef DEBUG_FILE_CPP
#define DEBUG_FILE_CPP

#include "DebugFile.h"

// const std::string debug_ceres_path = "/home/hao/Desktop/ceres_solver.debug";

// Ceres_Debug_File debug_ceres(debug_ceres_path);


// 在源文件中定义全局变量
// "/home/hao/Desktop/ceres_solver.debug";
const std::string debug_ceres_path =  "/home/hao/Desktop/twist_ws/src/TwistEstimator/output/ceres_solver.debug";
Ceres_Debug_File debug_ceres(debug_ceres_path);  // 在唯一源文件中定义并初始化全局变量
bool clear_debug = false;

const std::string debug_ceres_jacobis_path =  "/home/hao/Desktop/twist_ws/src/TwistEstimator/output/ceres_solver_jacobi.debug";
Ceres_Debug_File debug_ceres_jacobis(debug_ceres_jacobis_path);

#endif