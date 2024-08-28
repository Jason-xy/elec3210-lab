/**
** Created by Zhijian QIAO.
** UAV Group, Hong Kong University of Science and Technology
** email: zqiaoac@connect.ust.hk
**/

#ifndef SRC_UTILS_H
#define SRC_UTILS_H

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <chrono>

class TicToc {
 public:
  TicToc() { tic(); }

  void tic() { start = std::chrono::system_clock::now(); }

  double toc() {
    end = std::chrono::system_clock::now();
    std::chrono::duration<double> elapsed_seconds = end - start;
    duration = elapsed_seconds.count();
    return duration * 1000;
  }

  double duration_ms() { return duration * 1000; }

 private:
  std::chrono::time_point<std::chrono::system_clock> start, end;
  double duration;
};

template <typename Derived>
Eigen::Matrix<typename Derived::Scalar, 3, 1> R2ypr(
    const Eigen::MatrixBase<Derived> &R) {
  typedef typename Derived::Scalar Scalar_t;
  Eigen::Matrix<Scalar_t, 3, 1> n = R.col(0);
  Eigen::Matrix<Scalar_t, 3, 1> o = R.col(1);
  Eigen::Matrix<Scalar_t, 3, 1> a = R.col(2);

  Eigen::Matrix<Scalar_t, 3, 1> ypr(3);
  Scalar_t y = atan2(n(1), n(0));
  Scalar_t p = atan2(-n(2), n(0) * cos(y) + n(1) * sin(y));
  Scalar_t r =
      atan2(a(0) * sin(y) - a(1) * cos(y), -o(0) * sin(y) + o(1) * cos(y));
  ypr(0) = y;
  ypr(1) = p;
  ypr(2) = r;

  return ypr / M_PI * 180.0;
}

inline Eigen::Vector3d Pose6DTo3D(const Eigen::Matrix4d &pose6d) {
  Eigen::Vector3d pose3d;
  pose3d(0) = pose6d(0, 3);
  pose3d(1) = pose6d(1, 3);
  pose3d(2) = atan2(pose6d(1, 0), pose6d(0, 0));
  return pose3d;
}

inline Eigen::Matrix4d Pose3DTo6D(const Eigen::Vector3d &pose3d) {
  Eigen::Matrix4d pose6d;
  pose6d << cos(pose3d(2)), -sin(pose3d(2)), 0, pose3d(0), sin(pose3d(2)),
      cos(pose3d(2)), 0, pose3d(1), 0, 0, 1, 0, 0, 0, 0, 1;
  return pose6d;
}

inline Eigen::Matrix3d ExpSE2(const Eigen::Vector3d &se2) {
  Eigen::Matrix3d R;
  R << cos(se2(2)), -sin(se2(2)), se2(0), sin(se2(2)), cos(se2(2)), se2(1), 0,
      0, 1;
  return R;
}

inline Eigen::Vector3d LogSE2(const Eigen::Matrix3d &SE2) {
  Eigen::Vector3d se2;
  se2(0) = SE2(0, 2);
  se2(1) = SE2(1, 2);
  se2(2) = atan2(SE2(1, 0), SE2(0, 0));
  return se2;
}

#endif  // SRC_UTILS_H
