/**
** Created by Zhijian QIAO.
** UAV Group, Hong Kong University of Science and Technology
** email: zqiaoac@connect.ust.hk
**/

#include "parameters.h"

namespace params {
// icp parameters
double max_distance = 1.0;
int max_iterations = 100;

void readParameters(const ros::NodeHandle& nh) {
  std::cout << "Reading parameters..." << std::endl;
  nh.getParam("/icp/max_iterations", max_iterations);
  std::cout << "max_iterations: " << max_iterations << std::endl;
  nh.getParam("/icp/max_distance", max_distance);
  std::cout << "max_distance: " << max_distance << std::endl;
}
}  // namespace params
