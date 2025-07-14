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

#include "marginalization_factor.h"
#include <iomanip>

#include <fstream>


void ResidualBlockInfo::Evaluate() {
  residuals.resize(cost_function->num_residuals());

  std::vector<int> block_sizes = cost_function->parameter_block_sizes();

  jacobians.resize(block_sizes.size());  
  std::vector<double *> raw_jacobians;
  raw_jacobians.reserve(block_sizes.size());
  for (int i = 0; i < static_cast<int>(block_sizes.size()); i++) {
    jacobians[i].resize(cost_function->num_residuals(), block_sizes[i]);
    raw_jacobians.push_back(jacobians[i].data());
  }
  cost_function->Evaluate(parameter_blocks.data(), residuals.data(),
                          raw_jacobians.data());

  if (loss_function) {
    double residual_scaling_, alpha_sq_norm_;

    double sq_norm, rho[3];
    sq_norm = residuals.squaredNorm();  
    loss_function->Evaluate(sq_norm, rho);

    double sqrt_rho1_ = sqrt(rho[1]);
    if ((sq_norm == 0.0) || (rho[2] <= 0.0)) {
      residual_scaling_ = sqrt_rho1_;
      alpha_sq_norm_ = 0.0;
    } else {
      const double D = 1.0 + 2.0 * sq_norm * rho[2] / rho[1];
      const double alpha = 1.0 - sqrt(D);
      residual_scaling_ = sqrt_rho1_ / (1 - alpha);
      alpha_sq_norm_ = alpha / sq_norm;
    }

    for (int i = 0; i < static_cast<int>(parameter_blocks.size()); i++) {
      jacobians[i] = sqrt_rho1_ * (jacobians[i] - alpha_sq_norm_ * residuals *
                                       (residuals.transpose() * jacobians[i]));
    }
    residuals *= residual_scaling_;
  }
}

MarginalizationInfo::~MarginalizationInfo() {
  for (auto it = parameter_block_data.begin(); it != parameter_block_data.end();
       ++it) {
    delete [] it->second;
  }

  for (int i = 0; i < (int)factors.size(); i++) {
    delete factors[i]->cost_function;
    delete factors[i];
  }
}

void MarginalizationInfo::addResidualBlockInfo(
    ResidualBlockInfo *residual_block_info) {
  factors.emplace_back(residual_block_info);

  std::vector<double *> &parameter_blocks = residual_block_info->parameter_blocks;
  std::vector<int> parameter_block_sizes = residual_block_info->cost_function->parameter_block_sizes();
                                    
  for (int i = 0; i < static_cast<int>(residual_block_info->parameter_blocks.size()); i++) {
    double *addr = parameter_blocks[i];  
    int size = parameter_block_sizes[i];
    parameter_block_size[reinterpret_cast<long>(addr)] = size;
  }
  for (int i = 0; i < static_cast<int>(residual_block_info->drop_set.size()); i++) {
    double *addr = parameter_blocks[residual_block_info->drop_set[i]];  
    parameter_block_idx[reinterpret_cast<long>(addr)] = 0;  
  }
}

void MarginalizationInfo::preMarginalize() {
  for (auto it : factors) {
    it->Evaluate();

    if (it->residual_type == RType_Image) {
      if (it->residuals.norm() > 1.5) {
        LOG(INFO) << "image: " << it->residuals.transpose();
      }
    }

    std::vector<int> block_sizes = it->cost_function->parameter_block_sizes();

    for (int i = 0; i < static_cast<int>(block_sizes.size()); i++) {
      long addr = reinterpret_cast<long>(it->parameter_blocks[i]);
      int size = block_sizes[i];
      if (parameter_block_data.find(addr) == parameter_block_data.end()) {
        double *data = new double[size];
        memcpy(data, it->parameter_blocks[i], sizeof(double) * size);
        parameter_block_data[addr] = data;
      }
    }
  }
}

int MarginalizationInfo::localSize(int size) const {
  return size == 4 ? 3 : size;
}

void *ThreadsConstructA(void *threadsstruct) {
  ThreadsStruct *p = ((ThreadsStruct *)threadsstruct);

  for (auto it : p->sub_factors) {
    for (int i = 0; i < static_cast<int>(it->parameter_blocks.size()); i++) {
      int idx_i = p->parameter_block_idx[reinterpret_cast<long>(it->parameter_blocks[i])];
      int size_i = p->parameter_block_size[reinterpret_cast<long>(it->parameter_blocks[i])];
      if (size_i == 4) size_i = 3;
      Eigen::MatrixXd jacobian_i = it->jacobians[i].leftCols(size_i);
      for (int j = i; j < static_cast<int>(it->parameter_blocks.size()); j++) {
        int idx_j = p->parameter_block_idx[reinterpret_cast<long>(it->parameter_blocks[j])];
        int size_j = p->parameter_block_size[reinterpret_cast<long>(it->parameter_blocks[j])];
        if (size_j == 4) size_j = 3;
        Eigen::MatrixXd jacobian_j = it->jacobians[j].leftCols(size_j);

        if (i == j) {
          p->A.block(idx_i, idx_j, size_i, size_j) += jacobian_i.transpose() * jacobian_j;
        } else {
          p->A.block(idx_i, idx_j, size_i, size_j) += jacobian_i.transpose() * jacobian_j;
          p->A.block(idx_j, idx_i, size_j, size_i) = p->A.block(idx_i, idx_j, size_i, size_j).transpose();
        }
      }
      p->b.segment(idx_i, size_i) += jacobian_i.transpose() * it->residuals; 
    }
  }
  return threadsstruct;
}

// Jacobi 迭代计算 SVD
void jacobiSVD(const Eigen::MatrixXd& A, Eigen::VectorXd& singular_values, Eigen::MatrixXd& eigenvectors) {
    int m = A.rows(), n = A.cols();
    Eigen::MatrixXd U = A; // 复制 A, 逐步对角化
    Eigen::MatrixXd V = Eigen::MatrixXd::Identity(n, n); // 初始化 V

    const int maxIter = 100; // 最大迭代次数
    const double tol = 1e-10; // 误差阈值

    for (int iter = 0; iter < maxIter; ++iter) {
        bool converged = true;

        // 遍历 U 的列对
        for (int p = 0; p < n - 1; ++p) {
            for (int q = p + 1; q < n; ++q) {
                double a_pp = U.col(p).squaredNorm();
                double a_qq = U.col(q).squaredNorm();
                double a_pq = U.col(p).dot(U.col(q));

                if (std::abs(a_pq) > tol * std::sqrt(a_pp * a_qq)) {
                    converged = false;

                    double tau = (a_qq - a_pp) / (2.0 * a_pq);
                    double t = (tau >= 0 ? 1.0 : -1.0) / (std::abs(tau) + std::sqrt(1.0 + tau * tau));
                    double c = 1.0 / std::sqrt(1.0 + t * t);
                    double s = t * c;

                    // 旋转 U
                    for (int i = 0; i < m; ++i) {
                        double u_ip = U(i, p);
                        double u_iq = U(i, q);
                        U(i, p) = c * u_ip - s * u_iq;
                        U(i, q) = s * u_ip + c * u_iq;
                    }

                    // 旋转 V
                    for (int i = 0; i < n; ++i) {
                        double v_ip = V(i, p);
                        double v_iq = V(i, q);
                        V(i, p) = c * v_ip - s * v_iq;
                        V(i, q) = s * v_ip + c * v_iq;
                    }
                }
            }
        }

        if (converged) break; // 迭代收敛
    }

    // 提取奇异值（U 的列范数）
    singular_values = U.colwise().norm();
    eigenvectors = V; // 右奇异向量
}


bool MarginalizationInfo::marginalize() {
  TicToc timer;
  double time[10];

  int pos = 0;
  for (auto &it : parameter_block_idx) {
    it.second = pos;
    pos += localSize(parameter_block_size[it.first]);
  }
  m = pos;  
  for (const auto &it : parameter_block_size) {
    if (parameter_block_idx.find(it.first) == parameter_block_idx.end()) {
      parameter_block_idx[it.first] = pos;
      pos += localSize(it.second);
    }
  }
  n = pos - m;

  if (n <= 0) {
    return false;
  }

  LOG(ERROR) << "marginize: total: " << pos 
              << ", marginize: " << m << ", leave:" << n << std::endl;

  TicToc t_summing;
  Eigen::MatrixXd A(pos, pos);
  Eigen::VectorXd b(pos);
  // Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor, Eigen::aligned_allocator<double>> A(pos, pos);
  // Eigen::Matrix<double, Eigen::Dynamic, 1, Eigen::ColMajor, Eigen::aligned_allocator<double>> b(pos);
  A.setZero();
  b.setZero();


  // LOG(ERROR) << " get A & b "<< std::endl;

  TicToc t_thread_summing;
  pthread_t tids[NUM_THREADS];
  ThreadsStruct threadsstruct[NUM_THREADS];
  int i = 0;
  for (auto it : factors) {
    threadsstruct[i].sub_factors.push_back(it);
    i++;
    i = i % NUM_THREADS;
  }
  for (int i = 0; i < NUM_THREADS; i++) {
    TicToc zero_matrix;
    threadsstruct[i].A = Eigen::MatrixXd::Zero(pos, pos);
    threadsstruct[i].b = Eigen::VectorXd::Zero(pos);
    threadsstruct[i].parameter_block_size = parameter_block_size;
    threadsstruct[i].parameter_block_idx = parameter_block_idx;
    int ret = pthread_create(&tids[i], NULL, ThreadsConstructA, (void *)&(threadsstruct[i])); 
    if (ret != 0) {
      LOG(WARNING) << "pthread_create error";
      LOG(ERROR) << "pthread_create error";
      break;
    }
  }
  for (int i = NUM_THREADS - 1; i >= 0; i--)  
  {
    pthread_join(tids[i], NULL);
    A += threadsstruct[i].A;
    b += threadsstruct[i].b;
  }

  // LOG(ERROR) << "construct thread " << std::endl;
  // LOG(ERROR) << "Marginize A.size = " << A << std::endl;
  // LOG(ERROR) << "Marginize b.size = " << b << std::endl;

  time[0] = timer.toc();

  Eigen::MatrixXd Amm = 0.5 * (A.block(0, 0, m, m) + A.block(0, 0, m, m).transpose());
  Eigen::CompleteOrthogonalDecomposition<Eigen::MatrixXd> cod(Amm);
  Eigen::MatrixXd Amm_inv = cod.pseudoInverse();

  // Eigen::MatrixXd Amm = 0.5 * (A.block(0, 0, m, m) + A.block(0, 0, m, m).transpose());
  // Eigen::EigenSolver<Eigen::MatrixXd> saes1(Amm);
  // Eigen::VectorXd Amm_S = saes1.eigenvalues().real();
  // Eigen::VectorXd Amm_S_inv_trunc = Eigen::VectorXd((Amm_S.array() > eps).select(Amm_S.array().inverse(), 0));
  // Eigen::MatrixXd Amm_inv = saes1.eigenvectors().real() * Amm_S_inv_trunc.asDiagonal() * saes1.eigenvectors().real().transpose();


  /*{
  Eigen::MatrixXd Amm = 0.5 * (A.block(0, 0, m, m) + A.block(0, 0, m, m).transpose());
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(Amm, Eigen::ComputeThinU | Eigen::ComputeThinV);
  Eigen::VectorXd singularValues = svd.singularValues();
  Eigen::MatrixXd leftSingularVectors = svd.matrixU();
  Eigen::MatrixXd rightSingularVectors = svd.matrixV();
  Eigen::VectorXd truncatedSingularValues = singularValues.array() > eps ? singularValues.array() : 0;
  Eigen::MatrixXd Amm_inv = svd.matrixV() * truncatedSingularValues.asDiagonal().inverse() * svd.matrixU().transpose();
  }*/

  LOG(ERROR) << "Amm = " << Amm << std::endl;

  /*{
    Eigen::MatrixXd Amm = 0.5 * (A.block(0, 0, m, m) + A.block(0, 0, m, m).transpose());
    Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> saes(Amm);
    Eigen::VectorXd Amm_S = saes.eigenvalues();
    Eigen::VectorXd Amm_S_inv_trunc = Eigen::VectorXd((Amm_S.array() > eps).select(Amm_S.array().inverse(), 0));
    Eigen::MatrixXd Amm_inv = saes.eigenvectors() * Amm_S_inv_trunc.asDiagonal() * saes.eigenvectors().transpose();
  }*/

  // LOG(ERROR) << "construct Amm " << std::endl;

  time[1] = timer.toc();
#if false
  int cnt[2] = {0,0};
  for (int i = 0; i < Amm_S.size(); ++i) {
    if (Amm_S(i) > eps) {
      cnt[0]++;
    } else {
      cnt[1]++;
    }
  }

  std::stringstream ss; 
  if (m > 1000) {
    ss << std::setprecision(2) << Amm_S.head<10>().transpose(); 
  } else {
    // ss << std::setprecision(15) << Amm_S.transpose();
    ss << std::setprecision(2) << Amm_S.transpose();
  }
  LOG(INFO) << "Amm_S cnt: " << cnt[0] << "/" << cnt[1] << "; n/m " << n << "/" << m << "; " << ss.str();
 #endif

  // LOG(ERROR) << "before A = " << A << std::endl;

  Eigen::VectorXd bmm = b.segment(0, m);
  Eigen::MatrixXd Amr = A.block(0, m, m, n);
  Eigen::MatrixXd Arm = A.block(m, 0, n, m);
  Eigen::MatrixXd Arr = A.block(m, m, n, n);
  Eigen::VectorXd brr = b.segment(m, n);

  std::fstream marginize_blocks_file("/home/hao/Desktop/twist_ws/src/TwistEstimator/output/marginize_block.txt", std::ios::out | std::ios::app);
  marginize_blocks_file << "A = \n" << A << std::endl;
  marginize_blocks_file << "b = \n" << b << std::endl;
  A = Arr - Arm * Amm_inv * Amr;
  b = brr - Arm * Amm_inv * bmm;
  marginize_blocks_file << "margin A = \n" << A << std::endl;
  marginize_blocks_file << "margin b = \n" << b << std::endl;
  marginize_blocks_file << "m = " << m << ", n = " << n << std::endl;
  marginize_blocks_file.close();

  time[2] = timer.toc();

  // LOG(ERROR) << "Arm = " << Arm << std::endl;
  // LOG(ERROR) << "Amm_inv = " << Amm_inv << std::endl;
  // LOG(ERROR) << "Amr = " << Amr << std::endl;

  LOG(ERROR) << "Shur Update " << std::endl;

  // LOG(ERROR) << "A " << A << std::endl;



  //  Hx=b or (Jx = b) ====> (J^T*J)x = J^T e
  //         H=V*S*(V^T) => 
  //      H^+ = V*(S^(-1))*(V^T)  
  //          = (S^(1/2)* V^T)^T * (S^(1/2)* V^T)
  //        J = S^(1/2)* V^T
  //      J^T = V * S^(-1/2)
  //        e = S^(1/2)* V^T b
  // LOG(ERROR) << "check A.size = " << A.rows() << ", " << A.cols() << std::endl;
  // LOG(ERROR) << "A = " << A << std::endl;

  Eigen::ArrayXXd A_array = A.array();
  A_array = A_array.binaryExpr(Eigen::ArrayXXd::Constant(A_array.rows(), A_array.cols(), 1e-3), 
                                [](double x, double threshold) { return x < threshold ? 0.0 : x; });
  A_array = A_array.binaryExpr(Eigen::ArrayXXd::Constant(A_array.rows(), A_array.cols(), 1e3), 
                                [](double x, double threshold) { return x > threshold ? 1e3 : x; });
  // 更新 A
  A = A_array.matrix();

  // LOG(ERROR) << "A_modi = " << A << std::endl;

  // Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd, Eigen::aligned_allocator<double>> saes2(A);
  // Eigen::MatrixXd::Identity A_regular(A.rows(), A.cols());
/*{
  Eigen::EigenSolver<Eigen::MatrixXd> saes2(A);
  Eigen::VectorXd eigenvalues = saes2.eigenvalues().real(); // 获取实部特征值
  LOG(ERROR) << "eigenvalues = " << eigenvalues << std::endl;
  Eigen::VectorXd S = Eigen::VectorXd((eigenvalues.array() > eps)
                          .select(eigenvalues.array(), 0));
  Eigen::VectorXd S_inv = Eigen::VectorXd((eigenvalues.array() > eps)
                          .select(eigenvalues.array().inverse(), 0));
  Eigen::VectorXd S_sqrt = S.cwiseSqrt();
  Eigen::VectorXd S_inv_sqrt = S_inv.cwiseSqrt();
}*/
  
  // LOG(ERROR) << "A.size = " << A.rows() << ", " << A.cols() << std::endl;
  // if (A.hasNaN()) {
  //     LOG(ERROR) << "Matrix A contains NaN values!" << std::endl;
  // }
  // assert(A.rows() > 0 && A.cols() > 0 && A.allFinite());

  Eigen::VectorXd singular_values;
  Eigen::MatrixXd eigenvectors;
  // Eigen::JacobiSVD<Eigen::MatrixXd> svd(A, Eigen::ComputeFullU | Eigen::ComputeFullV);
  // svd.compute(A, Eigen::ComputeFullU | Eigen::ComputeFullV);
  // // svd.compute(A); 
  // svd.compute(A, Eigen::ComputeThinU | Eigen::ComputeThinV);
  // computeSVD(A, singular_values, eigenvectors);
  jacobiSVD(A, singular_values, eigenvectors);

  // Eigen::VectorXd singular_values = svd.singularValues().eval();  // 奇异值

  // LOG(ERROR) << "singular_values = " << singular_values << std::endl;

  // Eigen::VectorXd S = singular_values.array().select(singular_values.array() < eps, 0).matrix();
  // Eigen::VectorXd S_inv = singular_values.array().select(singular_values.array().inverse() < eps, 0).matrix();

  Eigen::VectorXd S = Eigen::VectorXd::Zero(singular_values.size());
  Eigen::VectorXd S_inv = Eigen::VectorXd::Zero(singular_values.size());

  for (int i = 0; i < singular_values.size(); ++i) {
      if (singular_values[i] > eps) {
          S[i] = singular_values[i];
          S_inv[i] = 1.0 / singular_values[i];  // 避免除零
      }
  }

  LOG(ERROR) << "S = " << S.transpose() << std::endl;
  LOG(ERROR) << "S_inv = " << S_inv.transpose() << std::endl;

  // Eigen::VectorXd S = singular_values.array() > eps ? singular_values : 0;
  // Eigen::VectorXd S_inv = singular_values.array() > eps ? singular_values.array().inverse() : 0;
  Eigen::VectorXd S_sqrt = S.cwiseSqrt();
  Eigen::VectorXd S_inv_sqrt = S_inv.cwiseSqrt();
  LOG(ERROR) << "S_sqrt = " << S_sqrt.transpose() << std::endl;
  LOG(ERROR) << "S_inv_sqrt = " << S_inv_sqrt.transpose() << std::endl;



  /*{
    Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> saes2(A);
    Eigen::VectorXd S = Eigen::VectorXd((saes2.eigenvalues().array() > eps)
                            .select(saes2.eigenvalues().array(), 0));
    Eigen::VectorXd S_inv = Eigen::VectorXd((saes2.eigenvalues().array() > eps)
                            .select(saes2.eigenvalues().array().inverse(), 0));
    Eigen::VectorXd S_sqrt = S.cwiseSqrt();
    Eigen::VectorXd S_inv_sqrt = S_inv.cwiseSqrt();
  }*/

  /*{
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(A, Eigen::ComputeThinU | Eigen::ComputeThinV);
    Eigen::VectorXd sigma = svd.singularValues();
    // 计算 S_sqrt
    Eigen::VectorXd S_sqrt = sigma.unaryExpr([](double x) { return x > 0 ? std::sqrt(x) : 0.0; });
    // 计算 S_inv_sqrt
    Eigen::VectorXd S_inv_sqrt = sigma.unaryExpr([](double x) { return x > 0 ? 1.0 / std::sqrt(x) : 0.0; });
    LOG(ERROR) << "S_sqrt: \n" << S_sqrt << std::endl;
    LOG(ERROR) << "S_inv_sqrt: \n" << S_inv_sqrt << std::endl;
  }*/

  time[3] = timer.toc();

  LOG(ERROR) << "S eigen slover done " << std::endl;

  // LOG(ERROR) << "Debug S: " << std::endl;
  // Eigen::MatrixXd S_sqrt_dense_matrix = S_sqrt.asDiagonal().toDenseMatrix();
  // LOG(ERROR) << "S_sqrt.asDiagonal() = " << S_sqrt_dense_matrix << std::endl;
  // Eigen::MatrixXd saes2_eigenvectors = saes2.eigenvectors(); //.transpose();
  // LOG(ERROR) << "saes2.eigenvectors().transpose() = " << saes2_eigenvectors << std::endl;
  // Eigen::MatrixXd S_inv_sqrt_dense_matrix = S_inv_sqrt.asDiagonal().toDenseMatrix();
  // LOG(ERROR) << "S_inv_sqrt.asDiagonal() = " << S_inv_sqrt_dense_matrix << std::endl;
  // LOG(ERROR) << "b = " << b << std::endl;

  // LOG(ERROR) << "S_sqrt.size() = " << S_sqrt.size() << std::endl;
  // Eigen::MatrixXd S_sqrt_dense_matrix = S_sqrt.asDiagonal().toDenseMatrix();
  // LOG(ERROR) << "S_sqrt_dense_matrix.size() = " << S_sqrt_dense_matrix.rows() << ", " << S_sqrt_dense_matrix.cols() << std::endl;
  // LOG(ERROR) << "saes2.eigenvectors().rows() = " << saes2.eigenvectors().rows() << std::endl;
  // LOG(ERROR) << "saes2.eigenvectors().cols() = " << saes2.eigenvectors().cols() << std::endl;
  // Eigen::MatrixXd S_diag = S_sqrt.asDiagonal();

  // int eigenvectors_rows = saes2.eigenvectors().real().rows();
  // int eigenvectors_cols = saes2.eigenvectors().real().cols();
  // int eigenvectors_size = eigenvectors_rows * eigenvectors_cols;

  /*{
    int eigenvectors_rows = saes2.eigenvectors().rows();
    int eigenvectors_cols = saes2.eigenvectors().cols();
    int eigenvectors_size = eigenvectors_rows * eigenvectors_cols;
  }*/


  // Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor, 
  //             Eigen::aligned_allocator<double>> saes2_eigenvectors_aligned(eigenvectors_rows, eigenvectors_cols);
  // saes2_eigenvectors_aligned = saes2.eigenvectors().transpose().eval();
  // LOG(ERROR) << "aligned is done" << std::endl;

  // Eigen::MatrixXd linearized_jacobians_temp = S_sqrt.asDiagonal() * saes2.eigenvectors().transpose().eval();
  // LOG(ERROR) << "linearized_jacobians_temp = " << linearized_jacobians_temp << std::endl;  

  // Eigen::MatrixXd eigenvalues = saes2.eigenvectors().real();
  // LOG(ERROR) << "eigenvalues calculate done" << std::endl;  

  // 显式对齐
  // Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor, 
                // Eigen::aligned_allocator<double>> eigenvalues_aligned = eigenvalues.transpose();
  
  // Eigen::MatrixXd eigenvalues_aligned = eigenvalues.transpose();

  // LOG(ERROR) << "eigenvalues_aligned calculate done" << std::endl;  

  // Eigen::MatrixXd S_Diagonal = S_sqrt.asDiagonal();

  // LOG(ERROR) << "S_Diagonal size = [" << S_Diagonal.rows() << ", " << S_Diagonal.cols() << "]" << std::endl;  
  // LOG(ERROR) << "eigenvalues_aligned size = [" << eigenvalues_aligned.rows() << ", " << eigenvalues_aligned.cols() << "]" << std::endl;

  // LOG(ERROR) << "linearized_jacobians calculate " << std::endl;  
  // linearized_jacobians = S_sqrt.asDiagonal() * saes2.eigenvectors().transpose().eval(); //saes2.eigenvectors().transpose();

  // Eigen::MatrixXd eigenvectors = svd.matrixU().eval();  // 特征向量
  LOG(ERROR) << "eigenvectors = " << eigenvectors << std::endl;
  linearized_jacobians = S_sqrt.asDiagonal() * eigenvectors;
  linearized_residuals = S_inv_sqrt.asDiagonal() * eigenvectors.transpose().eval() * b;

  LOG(ERROR) << "linearized_jacobians = " << linearized_jacobians << std::endl;
  LOG(ERROR) << "linearized_residuals = " << linearized_residuals << std::endl;

  /*
  Eigen::MatrixXd eigenvectors = saes2.eigenvectors().real();
  linearized_jacobians = S_sqrt.asDiagonal() *  eigenvectors;  // saes2.eigenvectors().transpose();
  linearized_residuals = S_inv_sqrt.asDiagonal() * eigenvectors.transpose().eval() * b;
  */

  
  LOG(ERROR) << "Calculate linearized " << std::endl;

  time[4] = timer.toc();
  LOG(ERROR) << "marginalize costs " 
            << time[0] << "/" 
            << time[1] - time[0] << "/" 
            << time[2] - time[1] << "/" 
            << time[3] - time[2] << "/"
            << time[4] - time[3] << " = " 
            << time[4] << " ms.";

  return true;
}

std::vector<double *> MarginalizationInfo::getParameterBlocks(
    std::unordered_map<long, double *> &addr_shift) {
  std::vector<double *> keep_block_addr;
  keep_block_size.clear();
  keep_block_idx.clear();
  keep_block_data.clear();

  for (const auto &it : parameter_block_idx) {
    if (it.second >= m) {
      keep_block_size.push_back(parameter_block_size[it.first]);
      keep_block_idx.push_back(parameter_block_idx[it.first]);
      keep_block_data.push_back(parameter_block_data[it.first]);
      keep_block_addr.push_back(addr_shift[it.first]);
    }
  }

  // sum_block_size = std::accumulate(std::begin(keep_block_size),
  // std::end(keep_block_size), 0);

  return keep_block_addr;
}


std::vector<double *> MarginalizationInfo::getParameterBlocks() {
  std::vector<double *> keep_block_addr;
  keep_block_size.clear();
  keep_block_idx.clear();
  keep_block_data.clear();

  for (const auto &it : parameter_block_idx) {
    if (it.second >= m) {
      keep_block_size.push_back(parameter_block_size[it.first]);
      keep_block_idx.push_back(parameter_block_idx[it.first]);
      keep_block_data.push_back(parameter_block_data[it.first]);
      keep_block_addr.push_back(reinterpret_cast<double *>(it.first));
    }
  }

  return keep_block_addr;
}


MarginalizationFactor::MarginalizationFactor(
    MarginalizationInfo::Ptr _marginalization_info)
    : marginalization_info(_marginalization_info) {
  int cnt = 0;
  for (auto it : marginalization_info->keep_block_size) {
    mutable_parameter_block_sizes()->push_back(it);
    cnt += it;
  }
  set_num_residuals(marginalization_info->n);  
};

bool MarginalizationFactor::Evaluate(double const *const *parameters,
                                     double *residuals,
                                     double **jacobians) const {
  int n = marginalization_info->n; 
  int m = marginalization_info->m; 

  // for (size_t i = 0; i < parameter_block_data.size(); ++i) {
  //   if (parameter_block_data[i] == nullptr) {
  //     LOG(ERROR) << "parameter_block_data[" << i << "] is nullptr!";
  //   }
  // }

  Eigen::VectorXd dx(n);  // delta_x
  for (int i = 0; i < static_cast<int>(marginalization_info->keep_block_size.size()); i++) {
    int size = marginalization_info->keep_block_size[i];
    int idx = marginalization_info->keep_block_idx[i] - m;  
    Eigen::VectorXd x = Eigen::Map<const Eigen::VectorXd>(parameters[i], size);  
    Eigen::VectorXd x0 = Eigen::Map<const Eigen::VectorXd>(marginalization_info->keep_block_data[i], size); 
    LOG(ERROR) << "size = " << size << std::endl;
    if (size != 4)
      dx.segment(idx, size) = x - x0;
    else {
      // dx.segment<3>(idx + 0) = x.head<3>() - x0.head<3>();
      Eigen::Quaterniond q0_inv = Eigen::Quaterniond(x0(3), x0(0), x0(1), x0(2)).inverse();
      Eigen::Quaterniond qx_inv = Eigen::Quaterniond(x(3), x(0), x(1), x(2));
      dx.segment<3>(idx + 0) = 2.0 * positify(q0_inv * qx_inv).vec();
      if (!((q0_inv * qx_inv).w() >= 0)) {
        dx.segment<3>(idx + 0) = 2.0 * -positify(q0_inv * qx_inv).vec();
      }
    }
  }

  Eigen::Map<Eigen::VectorXd>(residuals, n) = marginalization_info->linearized_residuals + 
                                              marginalization_info->linearized_jacobians * dx;
  if (jacobians) {
    for (int i = 0; i < static_cast<int>(marginalization_info->keep_block_size.size()); i++) {
      if (jacobians[i]) {
        int size = marginalization_info->keep_block_size[i];
        int local_size = marginalization_info->localSize(size);
        int idx = marginalization_info->keep_block_idx[i] - m;
        Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> 
                jacobian(jacobians[i], n, size);  
        jacobian.setZero();
        jacobian.leftCols(local_size) = marginalization_info->linearized_jacobians.middleCols(idx, local_size);
      }
    }
  }
  return true;
}


