#ifndef SRC_ICP_ODOM_H
#define SRC_ICP_ODOM_H

#include <nav_msgs/Path.h>
#include <pcl/filters/voxel_grid.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_broadcaster.h>

#include <Eigen/Eigen>
#include <fstream>
#include <mutex>
#include <numeric>
#include <queue>
#include <vector>

#include "icp.h"
#include "nav_msgs/Odometry.h"
#include "ros/console.h"
#include "ros/publisher.h"
#include "ros/ros.h"
#include "ros/subscriber.h"
#include "utils.h"

class OdomICP {
 private:
  ros::NodeHandle& nh_;
  std_msgs::Header cloudHeader;
  pcl::PointCloud<pcl::PointXYZ>::Ptr laserCloudIn;
  pcl::PointCloud<pcl::PointXYZ>::Ptr refCloud;
  std::queue<std::pair<std_msgs::Header, sensor_msgs::PointCloud2ConstPtr>>
      cloudQueue;

  Eigen::Matrix4d Twb;       // transformation from body to world
  Eigen::Matrix4d Twk;       // transformation from keyframe to world
  Eigen::Matrix4d Twb_gt;    // transformation from body to world (ground truth)
  Eigen::Matrix4d Twb_prev;  // transformation from body to world (previous)
  Eigen::Matrix4d
      deltaT_pred;  // predicted transformation from previous to current

  pcl::VoxelGrid<pcl::PointXYZ> dsFilterScan;
  pcl::VoxelGrid<pcl::PointXYZ> dsFilterMap;

  std::mutex cloudQueueMutex;
  bool firstFrame = true;

  ros::Subscriber lidar_sub;
  ros::Publisher odom_pub;
  ros::Publisher path_pub;
  ros::Publisher map_pub;
  ros::Publisher scan_pub;
  nav_msgs::Path path;

  TicToc timer;

  std::ofstream traj_file;

 public:
  OdomICP(ros::NodeHandle& n);

  ~OdomICP();

  void run();

  void publishResult();

  void cloudHandler(const sensor_msgs::PointCloud2ConstPtr& laserCloudMsg);

  void filterLocalMap();

  pcl::PointCloud<pcl::PointXYZ>::Ptr parseCloud(
      const sensor_msgs::PointCloud2ConstPtr& laserCloudMsg);
};
#endif