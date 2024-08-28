#ifndef VOXEL_H
#define VOXEL_H

#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <stdio.h>

#include <Eigen/Core>
#include <Eigen/Eigenvalues>
#include <algorithm>
#include <cstdlib>
#include <unordered_map>
#include <vector>

using VoxelKey = std::tuple<int, int, int>;

template <typename PointT>
VoxelKey point_to_voxel_key(const PointT &point,
                            const Eigen::Vector3f &voxel_size) {
  int x = static_cast<int>(std::floor(point.x / voxel_size.x()));
  int y = static_cast<int>(std::floor(point.y / voxel_size.y()));
  int z = static_cast<int>(std::floor(point.z / voxel_size.z()));
  return std::make_tuple(x, y, z);
}

template <typename T>
VoxelKey point_to_voxel_key(const Eigen::Matrix<T, 3, 1> &point,
                            const Eigen::Vector3f &voxel_size) {
  int x = static_cast<int>(std::floor(point.x() / voxel_size.x()));
  int y = static_cast<int>(std::floor(point.y() / voxel_size.y()));
  int z = static_cast<int>(std::floor(point.z() / voxel_size.z()));
  return std::make_tuple(x, y, z);
}

template <typename PointT, typename T>
void solveCovMat(const pcl::PointCloud<PointT> &cloud,
                 Eigen::Matrix<T, 3, 1> &mu, Eigen::Matrix<T, 3, 3> &cov) {
  mu.setZero();
  cov.setZero();
  Eigen::Matrix<T, 3, 1> point;
  auto N = cloud.size();
  for (int i = 0; i < N; ++i) {
    point = cloud.points[i].getVector3fMap().template cast<T>();
    mu += point;
    cov += point * point.transpose();
  }
  mu /= N;
  cov.noalias() = cov / N - mu * mu.transpose();
}

template <typename PointT, typename T>
void solveCenter(const pcl::PointCloud<PointT> &cloud,
                 Eigen::Matrix<T, 3, 1> &mu) {
  mu.setZero();
  Eigen::Matrix<T, 3, 1> point;
  auto N = cloud.size();
  for (int i = 0; i < N; ++i) {
    point = cloud.points[i].getVector3fMap().template cast<T>();
    mu += point;
  }
  mu /= N;
}

class Voxel {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  typedef std::shared_ptr<Voxel> Ptr;

  Voxel(const VoxelKey key) { key_ = key; }

  const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud() const {
    return cloud_.makeShared();
  }

  static void getNeighbors(const VoxelKey &loc,
                           std::vector<VoxelKey> &neighbors) {
    neighbors.clear();
    neighbors.reserve(27);  // 3^3
    int64_t x = std::get<0>(loc), y = std::get<1>(loc), z = std::get<2>(loc);
    for (int64_t i = x - 1; i <= x + 1; ++i) {
      for (int64_t j = y - 1; j <= y + 1; ++j) {
        for (int64_t k = z - 1; k <= z + 1; ++k) {
          neighbors.emplace_back(i, j, k);
        }
      }
    }
  }

  void parse() {
    if (cloud_.size() < 3) {
      solveCenter(cloud_, center_);
    }

    solveCovMat(cloud_, center_, sigma_);

    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> saes(sigma_);
    lambda_ = saes.eigenvalues();
    normal_ = saes.eigenvectors().col(0);
    if (lambda_(1) / lambda_(0) > 10) {
      plane = true;
    }
  }

  void getNeighbors(std::vector<VoxelKey> &neighbors) {
    getNeighbors(key_, neighbors);
  }

  void insertPoint(const pcl::PointXYZ &point) { cloud_.push_back(point); }

  VoxelKey loc() const { return key_; }

 public:
  int instance_id = -1;
  bool plane = false;

 private:
  VoxelKey key_;
  Eigen::Vector3d center_, normal_, lambda_, direction_;
  Eigen::Matrix3d sigma_, eigen_vectors_;  // eigen_vectors_ is in the ascending
                                           // order of eigenvalues
  pcl::PointCloud<pcl::PointXYZ> cloud_;
};

struct Vec3dHash {
  std::size_t operator()(const std::tuple<int, int, int> &vec3) const {
    return size_t(((std::get<0>(vec3)) * 73856093) ^
                  ((std::get<1>(vec3)) * 471943) ^
                  ((std::get<2>(vec3)) * 83492791)) %
           10000000;
  }
};
using VoxelMap = std::unordered_map<VoxelKey, Voxel::Ptr, Vec3dHash>;

template <typename PointT>
void cutCloud(const pcl::PointCloud<PointT> &cloud,
              const Eigen::Vector3f voxel_size, VoxelMap &voxel_map) {
  for (size_t i = 0; i < cloud.size(); i++) {
    VoxelKey position = point_to_voxel_key(cloud.points[i], voxel_size);
    VoxelMap::iterator voxel_iter = voxel_map.find(position);
    if (voxel_iter != voxel_map.end()) {
      voxel_iter->second->insertPoint(cloud.points[i]);
    } else {
      Voxel::Ptr voxel = Voxel::Ptr(new Voxel(position));
      voxel->insertPoint(cloud.points[i]);
      voxel_map.insert(std::make_pair(position, voxel));
    }
  }
}

#endif