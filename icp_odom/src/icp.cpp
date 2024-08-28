/**
** Created by Zhijian QIAO.
** UAV Group, Hong Kong University of Science and Technology
** email: zqiaoac@connect.ust.hk
**/
#include "icp.h"

#include <pcl/registration/icp.h>

#include "parameters.h"

Eigen::Matrix4d icp_registration(pcl::PointCloud<pcl::PointXYZ>::Ptr src_cloud,
                                 pcl::PointCloud<pcl::PointXYZ>::Ptr tar_cloud,
                                 Eigen::Matrix4d init_guess) {
  // This is an example of using pcl::IterativeClosestPoint to align two point
  // clouds In your project, you should implement your own ICP algorithm!!!

  pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
  icp.setInputSource(src_cloud);
  icp.setInputTarget(tar_cloud);
  icp.setMaximumIterations(params::max_iterations);  // set maximum iteration
  icp.setTransformationEpsilon(1e-6);  // set transformation epsilon
  icp.setMaxCorrespondenceDistance(
      params::max_distance);  // set maximum correspondence distance
  pcl::PointCloud<pcl::PointXYZ> aligned_cloud;
  icp.align(aligned_cloud, init_guess.cast<float>());

  Eigen::Matrix4d transformation = icp.getFinalTransformation().cast<double>();
  return transformation;
}

// TODO: Implement your own ICP algorithm here
// Eigen::Matrix4d icp_registration(pcl::PointCloud<pcl::PointXYZ>::Ptr
// src_cloud,
//                                  pcl::PointCloud<pcl::PointXYZ>::Ptr
//                                  tar_cloud, Eigen::Matrix4d init_guess) {

// }