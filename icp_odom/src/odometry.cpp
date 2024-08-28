#include "odometry.h"

#include <pcl/common/transforms.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/passthrough.h>
#include <pcl_ros/point_cloud.h>
#include <sensor_msgs/PointCloud2.h>

#include "global_definition.h"

using namespace std;
using namespace Eigen;

OdomICP::~OdomICP() {}

OdomICP::OdomICP(ros::NodeHandle& nh) : nh_(nh) {
  //    initialize variables here
  Twb = Eigen::Matrix4d::Identity();  // initial pose
  laserCloudIn.reset(new pcl::PointCloud<pcl::PointXYZ>);
  refCloud.reset(new pcl::PointCloud<pcl::PointXYZ>);

  //    initialze downsample filter here
  const double scan_leaf_size = 0.5, map_leaf_size = 0.1;
  dsFilterScan.setLeafSize(scan_leaf_size, scan_leaf_size, scan_leaf_size);
  dsFilterMap.setLeafSize(map_leaf_size, map_leaf_size, map_leaf_size);

  //    initialize ros publisher
  lidar_sub =
      nh_.subscribe("/velodyne_points", 1, &OdomICP::cloudHandler, this);
  odom_pub = nh_.advertise<nav_msgs::Odometry>("icp_odom", 1);
  path_pub = nh_.advertise<nav_msgs::Path>("icp_path", 1);
  scan_pub = nh_.advertise<sensor_msgs::PointCloud2>("current_scan", 1);
  map_pub = nh_.advertise<sensor_msgs::PointCloud2>("cloud_map", 1);

  // traj_file.open(WORK_SPACE_PATH + "/../dataset/true_trajectory.txt");
  std::cout << "Odometry ICP initialized" << std::endl;
}

void OdomICP::run() {
  ros::Rate rate(1000);
  while (ros::ok()) {
    if (cloudQueue.empty()) {
      rate.sleep();
      continue;
    }

    cloudQueueMutex.lock();
    cloudHeader = cloudQueue.front().first;
    laserCloudIn = parseCloud(cloudQueue.front().second);
    cloudQueue.pop();
    cloudQueueMutex.unlock();

    dsFilterScan.setInputCloud(laserCloudIn);
    dsFilterScan.filter(*laserCloudIn);

    if (firstFrame) {
      firstFrame = false;
      pcl::PointCloud<pcl::PointXYZ>::Ptr laserTransformed(
          new pcl::PointCloud<pcl::PointXYZ>);
      pcl::transformPointCloud(*laserCloudIn, *laserTransformed,
                               Twb.cast<float>());
      *refCloud = *laserTransformed;
      Twk = Twb;
      Twb_prev = Twb;
      deltaT_pred = Eigen::Matrix4d::Identity();
      continue;
    }

    timer.tic();
    // ICP process
    Eigen::Matrix4d guess = Twb_prev * deltaT_pred;
    Twb = icp_registration(laserCloudIn, refCloud, Twb);
    deltaT_pred = Twb_prev.inverse() * Twb;
    Twb_prev = Twb;
    timer.toc();

    // update map
    Eigen::Matrix4d Tbk = Twb.inverse() * Twk;
    double delat_t = Tbk.block<3, 1>(0, 3).norm();
    double delta_r = acos((Tbk.block<3, 3>(0, 0).trace() - 1) / 2) * 180 / M_PI;
    if (delat_t > 2.0 || delta_r > 5.0) {
      pcl::PointCloud<pcl::PointXYZ>::Ptr laserTransformed(
          new pcl::PointCloud<pcl::PointXYZ>);
      pcl::transformPointCloud(*laserCloudIn, *laserTransformed,
                               Twb.cast<float>());
      *refCloud += *laserTransformed;
      Twk = Twb;
      filterLocalMap();
    }

    publishResult();
    rate.sleep();
  }
}

void OdomICP::cloudHandler(
    const sensor_msgs::PointCloud2ConstPtr& laserCloudMsg) {
  cloudQueueMutex.lock();
  std_msgs::Header cloudHeader = laserCloudMsg->header;
  cloudHeader.stamp = ros::Time::now();
  cloudQueue.push(std::make_pair(cloudHeader, laserCloudMsg));
  cloudQueueMutex.unlock();
}

pcl::PointCloud<pcl::PointXYZ>::Ptr OdomICP::parseCloud(
    const sensor_msgs::PointCloud2ConstPtr& laserCloudMsg) {
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloudTmp(
      new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(*laserCloudMsg, *cloudTmp);
  // Remove Nan points
  std::vector<int> indices;
  pcl::removeNaNFromPointCloud(*cloudTmp, *cloudTmp, indices);
  return cloudTmp;
}

void OdomICP::filterLocalMap() {
  dsFilterMap.setInputCloud(refCloud);
  dsFilterMap.filter(*refCloud);
  // local map update. [-50, 50]
  pcl::PointCloud<pcl::PointXYZ>::Ptr localMap(
      new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PassThrough<pcl::PointXYZ> pass;
  pass.setInputCloud(refCloud);
  pass.setFilterFieldName("x");
  pass.setFilterLimits(Twb(0, 3) - 50, Twb(0, 3) + 50);
  pass.filter(*localMap);
  pass.setInputCloud(localMap);
  pass.setFilterFieldName("y");
  pass.setFilterLimits(Twb(1, 3) - 50, Twb(1, 3) + 50);
  pass.filter(*localMap);
  pcl::copyPointCloud(*localMap, *refCloud);
}

void OdomICP::publishResult() {
  //    publish odom
  nav_msgs::Odometry odom;
  odom.header.frame_id = "map";
  odom.child_frame_id = "base_link";
  odom.header.stamp = cloudHeader.stamp;
  odom.pose.pose.position.x = Twb(0, 3);
  odom.pose.pose.position.y = Twb(1, 3);
  odom.pose.pose.position.z = Twb(2, 3);
  Eigen::Quaterniond q(Twb.block<3, 3>(0, 0));
  q.normalize();
  odom.pose.pose.orientation.x = q.x();
  odom.pose.pose.orientation.y = q.y();
  odom.pose.pose.orientation.z = q.z();
  odom.pose.pose.orientation.w = q.w();
  odom_pub.publish(odom);

  //    publish path
  path.header.frame_id = "map";
  path.header.stamp = cloudHeader.stamp;
  geometry_msgs::PoseStamped pose;
  pose.header = odom.header;
  pose.pose = odom.pose.pose;
  path.poses.push_back(pose);
  path_pub.publish(path);

  //    publish map
  sensor_msgs::PointCloud2 mapMsg;
  pcl::toROSMsg(*refCloud, mapMsg);
  mapMsg.header.frame_id = "map";
  mapMsg.header.stamp = cloudHeader.stamp;
  map_pub.publish(mapMsg);

  //    publish laser
  sensor_msgs::PointCloud2 laserMsg;
  pcl::PointCloud<pcl::PointXYZ>::Ptr laserTransformed(
      new pcl::PointCloud<pcl::PointXYZ>);
  pcl::transformPointCloud(*laserCloudIn, *laserTransformed, Twb.cast<float>());
  pcl::toROSMsg(*laserTransformed, laserMsg);
  laserMsg.header.frame_id = "map";
  laserMsg.header.stamp = cloudHeader.stamp;
  scan_pub.publish(laserMsg);

  // Eigen::Vector3d rpy = q.toRotationMatrix().eulerAngles(0, 1, 2) * 180 /
  // M_PI; std::cout << "x: " << Twb(0, 3) << " y: " << Twb(1, 3) << " z: " <<
  // Twb(2, 3)
  //           << " roll: " << rpy(0) << " pitch: " << rpy(1) << " yaw: " <<
  //           rpy(2)
  //           << " time: " << timer.duration_ms() << " ms" << std::endl;
  // traj_file << std::fixed << cloudHeader.stamp.toSec() << " " << Twb(0, 0)
  //           << " " << Twb(0, 1) << " " << Twb(0, 2) << " " << Twb(0, 3) << "
  //           "
  //           << Twb(1, 0) << " " << Twb(1, 1) << " " << Twb(1, 2) << " "
  //           << Twb(1, 3) << " " << Twb(2, 0) << " " << Twb(2, 1) << " "
  //           << Twb(2, 2) << " " << Twb(2, 3) << std::endl;
}
