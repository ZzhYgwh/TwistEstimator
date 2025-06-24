/**
BSD 3-Clause License

This file is part of the Basalt project.
https://gitlab.com/VladyslavUsenko/basalt-headers.git

Copyright (c) 2019, Vladyslav Usenko and Nikolaus Demmel.
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

* Neither the name of the copyright holder nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

@file
@brief Uniform B-spline for SE(3)
*/

#pragma once

#include "assert.h"
// #include "r3_spline.h"
#include "rd_spline.h"
// #include "linear_spline.h"
#include "spline_segment.h"

#include <array>



/// @brief Uniform B-spline for SE(3) of order N. Internally uses an SO(3) (\ref
/// So3Spline) spline for rotation and 3D Euclidean spline (\ref RdSpline) for
/// translation (split representaion).
///
/// See [[arXiv:1911.08860]](https://arxiv.org/abs/1911.08860) for more details.
template <int _N, typename _Scalar = double>
class Rd6Spline {

 public:
 EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  static constexpr int N = _N;        ///< Order of the spline.
  static constexpr int DEG = _N - 1;  ///< Degree of the spline.

  using MatN = Eigen::Matrix<_Scalar, _N, _N>;
  using VecN = Eigen::Matrix<_Scalar, _N, 1>;
  using VecNp1 = Eigen::Matrix<_Scalar, _N + 1, 1>;

  using Vec3 = Eigen::Matrix<_Scalar, 3, 1>;
  using Vec6 = Eigen::Matrix<_Scalar, 6, 1>;
  using Vec9 = Eigen::Matrix<_Scalar, 9, 1>;
  using Vec12 = Eigen::Matrix<_Scalar, 12, 1>;

  using Mat3 = Eigen::Matrix<_Scalar, 3, 3>;
  using Mat6 = Eigen::Matrix<_Scalar, 6, 6>;

  using Mat36 = Eigen::Matrix<_Scalar, 3, 6>;
  using Mat39 = Eigen::Matrix<_Scalar, 3, 9>;
  using Mat312 = Eigen::Matrix<_Scalar, 3, 12>;

  using Matrix3Array = std::array<Mat3, N>;
  using Matrix36Array = std::array<Mat36, N>;
  using Matrix6Array = std::array<Mat6, N>;

  using PosJacobianStruct = typename RdSpline<3, N, _Scalar>::JacobianStruct;
  using SO3JacobianStruct = typename So3Spline<N, _Scalar>::JacobianStruct;

  /// @brief Struct to store the accelerometer residual Jacobian with
  /// respect to knots
  struct AccelPosSO3JacobianStruct {
    size_t start_idx;
    std::array<Mat36, N> d_val_d_knot;
  };

  /// @brief Struct to store the pose Jacobian with respect to knots
  struct PosePosSO3JacobianStruct {
    size_t start_idx;
    std::array<Mat6, N> d_val_d_knot;
  };

  /// @brief Constructor with knot interval and start time
  ///
  /// @param[in] time_interval_ns knot time interval in seconds
  /// @param[in] start_time_ns start time of the spline in seconds
  Rd6Spline(int64_t time_interval_ns, int64_t start_time_ns = 0)
      : linear_spline(time_interval_ns, start_time_ns),
        angular_spline(time_interval_ns, start_time_ns),
        dt_ns_(time_interval_ns) {}

  /// @brief Gererate random trajectory
  ///
  /// @param[in] n number of knots to generate
  /// @param[in] static_init if true the first N knots will be the same
  /// resulting in static initial condition
  void genRandomTrajectory(int n, bool static_init = false) {
    linear_spline.genRandomTrajectory(n, static_init);
    angular_spline.genRandomTrajectory(n, static_init);
  }

  /// @brief Set the knot to particular SE(3) pose
  ///
  /// @param[in] pose SE(3) pose
  /// @param[in] i index of the knot
  void setKnot(const Vec6 &twist, int i) {
    linear_spline.getKnot(i) = twist.block<3,1>(0,0);
    angular_spline.getKnot(i) = twist.block<3,1>(3,0);
  }

  /// @brief Set the knot to particular Vec3 pose
  ///
  /// @param[in] pos Vec3 pose
  /// @param[in] i index of the knot
  void setKnotAngular(const Vec3 angular, int i) { angular_spline.getKnot(i) = angular; }

  /// @brief Set the knot to particular Vec3 pose
  ///
  /// @param[in] ori Vec3 pose
  /// @param[in] i index of the knot
  void setKnotLinear(const Vec3 linear, int i) { linear_spline.getKnot(i) = linear; }

  /// @brief Reset spline to have num_knots initialized at pose
  ///
  /// @param[in] pose SE(3) pose
  /// @param[in] num_knots number of knots to initialize
  void setKnots(const Vec6 &twist, int num_knots) {
    linear_spline.resize(num_knots);
    angular_spline.resize(num_knots);

    for (int i = 0; i < num_knots; i++) {
      linear_spline.getKnot(i) = twist.block<3,1>(0,0);
      angular_spline.getKnot(i) = twist.block<3,1>(3,0);
    }
  }

  /// @brief Reset spline to the knots from other spline
  ///
  /// @param[in] other spline to copy knots from
  void setKnots(const Rd6Spline<N, _Scalar> &other) {
    BASALT_ASSERT(other.dt_ns_ == dt_ns_);
    BASALT_ASSERT(other.angular_spline.getKnots().size() ==
                  other.angular_spline.getKnots().size());

    size_t num_knots = other.angular_spline.getKnots().size();

    linear_spline.resize(num_knots);
    angular_spline.resize(num_knots);

    for (size_t i = 0; i < num_knots; i++) {
      linear_spline.getKnot(i) = other.linear_spline.getKnot(i);
      angular_spline.getKnot(i) = other.angular_spline.getKnot(i);
    }
  }

  /// @brief extend trajectory to time t
  ///
  /// @param[in] time_ns time
  /// @param[in] initial_so3 initial knot of linear_spline
  /// @param[in] initial_pos initial knot of angular_spline
  void extendKnotsTo(int64_t time_ns, const Vec3& initial_vel,
                     const Vec3& initial_ang) {
    while ((numKnots() < N) || (maxTimeNs() < time_ns)) {
      // linear_spline.knots_push_back(initial_vel);
      // angular_spline.knots_push_back(initial_angs);
    
      knots_push_back(initial_vel, initial_ang);
    }
  }

  /// @brief Add knot to the end of the spline
  ///
  /// @param[in] knot knot to add
  inline void knots_push_back(const Vec3& initial_vel, const Vec3& initial_ang) {
    linear_spline.knots_push_back(initial_vel);
    angular_spline.knots_push_back(initial_ang);
  }


  /// @brief extend trajectory to time t
  ///
  /// @param[in] time_ns timestamp
  /// @param[in] initial_knot initial knot
  void extendKnotsTo(int64_t time_ns, const Vec6 &initial_knot) {
    while ((numKnots() < N) || (maxTimeNs() < time_ns)) {
      knots_push_back(initial_knot);
    }
  }

  /// @brief Add knot to the end of the spline
  ///
  /// @param[in] knot knot to add
  inline void knots_push_back(const Vec6 &knot) {
    linear_spline.knots_push_back(knot.block<3,1>(0,0));
    angular_spline.knots_push_back(knot.block<3,1>(3,0));
  }

  /// @brief Remove knot from the back of the spline
  inline void knots_pop_back() {
    linear_spline.knots_pop_back();
    angular_spline.knots_pop_back();
  }

  /// @brief Return the first knot of the spline
  ///
  /// @return first knot of the spline
  inline Vec6 knots_front() const {
    Vec6 res(linear_spline.knots_front(), angular_spline.knots_front());

    return res;
  }

  /// @brief Remove first knot of the spline and increase the start time
  inline void knots_pop_front() {
    linear_spline.knots_pop_front();
    angular_spline.knots_pop_front();

    BASALT_ASSERT(linear_spline.minTimeNs() == angular_spline.minTimeNs());
    BASALT_ASSERT(linear_spline.getKnots().size() == angular_spline.getKnots().size());
  }

  /// @brief Return the last knot of the spline
  ///
  /// @return last knot of the spline
  Vec6 getLastKnot() {
    BASALT_ASSERT(linear_spline.getKnots().size() == angular_spline.getKnots().size());

    Vec6 res;
    res << linear_spline.getKnots().back(), angular_spline.getKnots().back();
    return res;
  }

  // std::pair<Vec3, Vec3> getLastKnots() {
  //   BASALT_ASSERT(linear_spline.getKnots().size() == angular_spline.getKnots().size());

  //   Vec6 res;
  //   res << linear_spline.getKnots().back(), angular_spline.getKnots().back();
  //   return res;
  // }

  std::pair<Vec3, Vec3> getLastKnots() {
    const auto& linear_knots = linear_spline.getKnots();
    const auto& angular_knots = angular_spline.getKnots();

    BASALT_ASSERT(linear_knots.size() == angular_knots.size());
    BASALT_ASSERT(!linear_knots.empty());

    return {linear_knots.back(), angular_knots.back()};
  }



  /// @brief Return knot with index i
  ///
  /// @param i index of the knot
  /// @return knot
  Vec6 getKnot(size_t i) const {
    Vec6 res(getKnotLinear(i), getKnotAngular(i));
    return res;
  }

  /// @brief Return knot with index i
  ///
  /// @param i index of the knot
  /// @return knot
  size_t getKnotSize() const {
    BASALT_ASSERT(linear_spline.getKnots().size() == angular_spline.getKnots().size());

    return linear_spline.getKnots().size();
  }

  /// @brief Return reference to the SO(3) knot with index i
  ///
  /// @param i index of the knot
  /// @return reference to the SO(3) knot
  inline Vec3 &getKnotLinear(size_t i) { return linear_spline.getKnot(i); }

  /// @brief Return const reference to the SO(3) knot with index i
  ///
  /// @param i index of the knot
  /// @return const reference to the SO(3) knot
  inline const Vec3 &getKnotLinear(size_t i) const { return linear_spline.getKnot(i); }

  /// @brief Return reference to the position knot with index i
  ///
  /// @param i index of the knot
  /// @return reference to the position knot
  inline Vec3 &getKnotAngular(size_t i) { return angular_spline.getKnot(i); }

  /// @brief Return const reference to the position knot with index i
  ///
  /// @param i index of the knot
  /// @return const reference to the position knot
  inline const Vec3 &getKnotAngular(size_t i) const {
    return angular_spline.getKnot(i);
  }

  /// @brief Set start time for spline
  ///
  /// @param[in] start_time start time of the spline in seconds
  inline void setStartTime(double timestamp) {
    linear_spline.setStartTime(timestamp);
    angular_spline.setStartTime(timestamp);
  }

  /// @brief Apply increment to the knot
  ///
  /// The incremernt vector consists of translational and rotational parts \f$
  /// [\upsilon, \omega]^T \f$. Given the current pose of the knot \f$ R \in
  /// SO(3), p \in \mathbb{R}^3\f$ the updated pose is: \f{align}{ R' &=
  /// \exp(\omega) R
  /// \\ p' &= p + \upsilon
  /// \f}
  ///  The increment is consistent with \ref
  /// PoseState::applyInc.
  ///
  /// @param[in] i index of the knot
  /// @param[in] inc 6x1 increment vector
  template <typename Derived>
  void applyInc(int i, const Eigen::MatrixBase<Derived> &inc) {
    EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Derived, 6);

    angular_spline.getKnot(i) += inc.template head<3>();
    linear_spline.getKnot(i) += inc.template head<3>();
        // Vec3::exp(inc.template tail<3>()) * linear_spline.getKnot(i);
  }

  /// @brief Minimum time represented by spline
  ///
  /// @return minimum time represented by spline in seconds
  int64_t minTimeNs() const {
    BASALT_ASSERT_STREAM(linear_spline.minTimeNs() == angular_spline.minTimeNs(),
                         "linear_spline.minTimeNs() " << linear_spline.minTimeNs()
                                                   << " angular_spline.minTimeNs() "
                                                   << angular_spline.minTimeNs());
    return angular_spline.minTimeNs();
  }

  int64_t maxTimeNs() const {
    BASALT_ASSERT_STREAM(linear_spline.maxTimeNs() == angular_spline.maxTimeNs(),
                         "linear_spline.maxTimeNs() " << linear_spline.maxTimeNs()
                                                   << " angular_spline.maxTimeNs() "
                                                   << angular_spline.maxTimeNs());
    return angular_spline.maxTimeNs();
  }

  /// @brief Number of knots in the spline
  size_t numKnots() const { return angular_spline.getKnots().size(); }

  int Get_N() const {  return N; }

 protected:
  /// @brief Linear acceleration in the world frame.
  ///
  /// @param[in] time_ns time to evaluate linear acceleration in seconds
  inline Vec3 LinearJerkWorld(int64_t time_ns) const {
    return linear_spline.acceleration(time_ns);
  }

  /// @brief Linear velocity in the world frame.
  ///
  /// @param[in] time_ns time to evaluate linear velocity in seconds
  inline Vec3 LinearAccWorld(int64_t time_ns) const {
    return linear_spline.velocity(time_ns);
  }

  /// @brief Position in the world frame.
  ///
  /// @param[in] time_ns time to evaluate position in seconds
  inline Vec3 LinearVelWorld(int64_t time_ns) const {
    return linear_spline.evaluate(time_ns);
  }

  /// @brief Linear acceleration in the world frame.
  ///
  /// @param[in] time_ns time to evaluate linear acceleration in seconds
  inline Vec3 AngularJerkWorld(int64_t time_ns) const {
    return angular_spline.acceleration(time_ns);
  }

  /// @brief Linear velocity in the world frame.
  ///
  /// @param[in] time_ns time to evaluate linear velocity in seconds
  inline Vec3 AngularAccWorld(int64_t time_ns) const {
    return angular_spline.velocity(time_ns);
  }

  /// @brief Position in the world frame.
  ///
  /// @param[in] time_ns time to evaluate position in seconds
  inline Vec3 AngularVelWorld(int64_t time_ns) const {
    return angular_spline.evaluate(time_ns);
  }

  /// @brief Evaluate pose.
  ///
  /// @param[in] time_ns time to evaluate pose in seconds
  /// @return SE(3) pose at time
  Vec6 poseNs(int64_t time_ns) const {
    Vec6 res;
    res.setZero(); // 加这一行，初始化为零
    // res.block<3,1>(0,0) = linear_spline.evaluate(time_ns);
    // res.block<3,1>(3,0) = angular_spline.evaluate(time_ns);

    res.template block<3,1>(0,0) = linear_spline.evaluate(time_ns);
    res.template block<3,1>(3,0) = angular_spline.evaluate(time_ns);

    return res;
  }


  /// @brief Evaluate pose and compute Jacobian.
  ///
  /// @param[in] time_ns time to evaluate pose inseconds
  /// @param[out] J Jacobian of the pose with respect to knots
  /// @return SE(3) pose at time
  Vec6 poseNs(int64_t time_ns, PosePosSO3JacobianStruct *J) const {
    Vec6 res;

    typename RdSpline<3, N, _Scalar>::JacobianStruct J_v;
    typename RdSpline<3, N, _Scalar>::JacobianStruct J_a; // a means angular

    res.block<3,1>(0,0) = linear_spline.evaluate(time_ns, &J_v);
    res.block<3,1>(3,0) = angular_spline.evaluate(time_ns, &J_a);

    // HAO TODO:
    /*if (J) {
      Eigen::Matrix3d RT = res.so3().inverse().matrix();

      J->start_idx = Jr.start_idx;
      for (int i = 0; i < N; i++) {
        J->d_val_d_knot[i].setZero();
        J->d_val_d_knot[i].template topLeftCorner<3, 3>() =
            RT * Jp.d_val_d_knot[i];
        J->d_val_d_knot[i].template bottomRightCorner<3, 3>() =
            RT * Jr.d_val_d_knot[i];
      }
    }*/

    return res;
  }

 public:
  /// @brief Print knots for debugging.
  inline void print_knots() const {
    for (size_t i = 0; i < angular_spline.getKnots().size(); i++) {
      std::cout << i << ": linear:" << angular_spline.getKnot(i).transpose() << " angular: "
                << linear_spline.getKnot(i).transpose()
                << std::endl;
    }
  }

  /// @brief Print position knots for debugging.
  inline void print_linear_knots() const {
    std::cout << "linear Knots : " << std::endl;
    for (size_t i = 0; i < linear_spline.getKnots().size(); i++) {
      std::cout << i << " : " << linear_spline.getKnot(i).transpose() << std::endl;
    }
  }

  /// @brief Print position knots for debugging.
  inline void print_angular_knots() const {
    std::cout << "angular Knots : " << std::endl;
    for (size_t i = 0; i < angular_spline.getKnots().size(); i++) {
      std::cout << i << " : " << angular_spline.getKnot(i).transpose() << std::endl;
    }
  }

  /// @brief Knot time interval in nanoseconds.
  inline int64_t getDtNs() const { return dt_ns_; }

  inline double getDt() const { return dt_ns_ * NS_TO_S; }

  std::pair<double, size_t> computeTIndexNs(int64_t time_ns) const {
    return angular_spline.computeTIndexNs(time_ns);
  }

  size_t GetCtrlIndex(int64_t time_ns) const {
    return angular_spline.computeTIndexNs(time_ns).second;
  }

  void CaculateSplineMeta(time_init_t times,
                          SplineMeta<_N> &spline_meta) const {
    int64_t master_dt_ns = getDtNs();
    int64_t master_t0_ns = minTimeNs();
    size_t current_segment_start = 0;
    size_t current_segment_end = 0;  // Negative signals no segment created yet

    // Times are guaranteed to be sorted correctly and t2 >= t1
    for (auto tt : times) {
      std::pair<double, size_t> ui_1, ui_2;
      ui_1 = linear_spline.computeTIndexNs(tt.first);
      ui_2 = linear_spline.computeTIndexNs(tt.second);

      size_t i1 = ui_1.second;
      size_t i2 = ui_2.second;

      // Create new segment, or extend the current one
      if (spline_meta.segments.empty() || i1 > current_segment_end) {
        int64_t segment_t0_ns = master_t0_ns + master_dt_ns * i1;
        spline_meta.segments.push_back(
            SplineSegmentMeta<_N>(segment_t0_ns, master_dt_ns));
        current_segment_start = i1;
      } else {
        i1 = current_segment_end + 1;
      }

      auto &current_segment_meta = spline_meta.segments.back();

      for (size_t i = i1; i < (i2 + N); ++i) {
        current_segment_meta.n += 1;
      }

      current_segment_end = current_segment_start + current_segment_meta.n - 1;
    }  // for times
  }

  void CaculateSplineMeta(time_init_t times,
                          SplineMeta<_N> &spline_meta, const RdSpline<3, _N, _Scalar> spline) const {
    int64_t master_dt_ns = getDtNs();
    int64_t master_t0_ns = minTimeNs();
    size_t current_segment_start = 0;
    size_t current_segment_end = 0;  // Negative signals no segment created yet

    // Times are guaranteed to be sorted correctly and t2 >= t1
    for (auto tt : times) {
      std::pair<double, size_t> ui_1, ui_2;
      ui_1 = spline.computeTIndexNs(tt.first);
      ui_2 = spline.computeTIndexNs(tt.second);

      size_t i1 = ui_1.second;
      size_t i2 = ui_2.second;

      // Create new segment, or extend the current one
      if (spline_meta.segments.empty() || i1 > current_segment_end) {
        int64_t segment_t0_ns = master_t0_ns + master_dt_ns * i1;
        spline_meta.segments.push_back(
            SplineSegmentMeta<_N>(segment_t0_ns, master_dt_ns));
        current_segment_start = i1;
      } else {
        i1 = current_segment_end + 1;
      }

      auto &current_segment_meta = spline_meta.segments.back();

      for (size_t i = i1; i < (i2 + N); ++i) {
        current_segment_meta.n += 1;
      }

      current_segment_end = current_segment_start + current_segment_meta.n - 1;
    }  // for times
  }

  void CaculateLinearSplineMeta(time_init_t times,
                          SplineMeta<_N> &spline_meta) const { 
    CaculateSplineMeta(times, spline_meta, linear_spline);
  }

  void CaculateAngularSplineMeta(time_init_t times,
                          SplineMeta<_N> &spline_meta) const { 
    CaculateSplineMeta(times, spline_meta, angular_spline);
  }

  void CaculateSplineMeta(time_init_t times,
                          SplineMeta<_N> &spline_linear_meta,
                          SplineMeta<_N> &spline_angular_meta) const { 
      CaculateLinearSplineMeta(times, spline_linear_meta);
      CaculateAngularSplineMeta(times, spline_angular_meta);
  }

  void GetCtrlIdxs(const SplineMeta<_N> &spline_meta,
                   std::vector<int> &ctrl_ids) {
    size_t last_ctrl_id = 0;
    for (auto const &seg : spline_meta.segments) {
      size_t start_idx = GetCtrlIndex(seg.t0_ns);
      for (size_t i = 0; i < seg.NumParameters(); i++) {
        if (i + start_idx > last_ctrl_id || (i + start_idx == 0)) {
          ctrl_ids.push_back(start_idx + i);
        }
      }
      last_ctrl_id = ctrl_ids.back();
    }
  }

  void GetCtrlIdxs(const std::vector<double> &timestamps,
                   std::vector<int> &ctrl_ids) {
    int last_ctrl_id = -1;
    for (const double &t : timestamps) {
      int start_idx = GetCtrlIndex(t);
      for (int i = 0; i < SplineOrder; i++) {
        if (i + start_idx > last_ctrl_id) {
          ctrl_ids.push_back(start_idx + i);
        }
      }
      last_ctrl_id = ctrl_ids.back();
    }
  }

  RdSpline<3, _N, _Scalar> GetLinearSpline() {
    return linear_spline;
  }
  RdSpline<3, _N, _Scalar> GetAngularSpline() {
    return angular_spline;
  }

  

 private:
 
  RdSpline<3, _N, _Scalar> angular_spline;  ///< Position spline
  RdSpline<3, _N, _Scalar> linear_spline;    ///< Orientation spline

  std::vector<Rd6Spline<N>, Eigen::aligned_allocator<Rd6Spline<N>>> splines;

  int64_t dt_ns_;  ///< Knot interval in nanoseconds
};


