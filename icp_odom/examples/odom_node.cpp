#include <thread>

#include "odometry.h"
#include "parameters.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, "icp_odom");
  ros::NodeHandle n("~");
  params::readParameters(n);

  OdomICP odom(n);
  std::thread odomThread(&OdomICP::run, &odom);
  ros::spin();

  return 0;
}