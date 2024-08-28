/**
** Created by Zhijian QIAO.
** UAV Group, Hong Kong University of Science and Technology
** email: zqiaoac@connect.ust.hk
**/

#ifndef SRC_EXTRACT_CYLINDER_H
#define SRC_EXTRACT_CYLINDER_H

#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl_ros/point_cloud.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include "voxel.h"

class ExtractCylinder {
 public:
  ExtractCylinder(ros::NodeHandle& n) {
    nh_ = n;
    cylinder_marker_pub =
        nh_.advertise<visualization_msgs::MarkerArray>("cylinder_marker", 1);
    cylinder_cloud_pub =
        nh_.advertise<sensor_msgs::PointCloud2>("cylinder_cloud", 1);
    non_plane_cloud_pub =
        nh_.advertise<sensor_msgs::PointCloud2>("non_plane_cloud", 1);
    colored_cloud_pub =
        nh_.advertise<sensor_msgs::PointCloud2>("colored_cloud", 1);
    reset();
  }

  Eigen::MatrixX2d extract(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloudIn,
                           std_msgs::Header header) {
    reset();
    cloudHeader = header;

    // detect columns of the point cloud
    // 1. remove ground points
    for (int i = 0; i < cloudIn->points.size(); ++i) {
      if (cloudIn->points[i].z > -0.0) {
        non_ground_cloud->points.push_back(cloudIn->points[i]);
      }
    }

    // 2. euclidean clustering
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clusters =
        clustering(non_ground_cloud);

    // 3. fit cylinder
    std::vector<Eigen::Vector2d> centers;
    for (int i = 0; i < clusters.size(); ++i) {
      pcl::PointCloud<pcl::PointXYZ>::Ptr cluster = clusters[i];
      if (cluster->points.size() < 20) continue;
      Eigen::Vector2d center = fitCircle(cluster);
      // evaluate the fitness of the circle
      double fitness = 0;
      std::vector<double> positive_angles, negative_angles;
      for (int j = 0; j < cluster->points.size(); ++j) {
        double dist = std::sqrt(std::pow(cluster->points[j].x - center[0], 2) +
                                std::pow(cluster->points[j].y - center[1], 2));
        fitness += std::abs(dist - radius);
        double angle = std::atan2(cluster->points[j].y - center[1],
                                  cluster->points[j].x - center[0]) *
                       180 / M_PI;
        if (angle > 0)
          positive_angles.push_back(angle);
        else
          negative_angles.push_back(angle);
      }
      fitness /= cluster->points.size();
      double angle_diff = 0;
      if (positive_angles.size() > 0 && negative_angles.size() > 0) {
        double max_positive_angle =
            *std::max_element(positive_angles.begin(), positive_angles.end());
        double min_negative_angle =
            *std::min_element(negative_angles.begin(), negative_angles.end());
        double angle_diff1 = 360 - (max_positive_angle - min_negative_angle);
        double min_positive_angle =
            *std::min_element(positive_angles.begin(), positive_angles.end());
        double max_negative_angle =
            *std::max_element(negative_angles.begin(), negative_angles.end());
        double angle_diff2 = 360 - (min_positive_angle - max_negative_angle);
        angle_diff = std::max(angle_diff1, angle_diff2);
      } else if (positive_angles.size() > 0) {
        angle_diff =
            *std::max_element(positive_angles.begin(), positive_angles.end()) -
            *std::min_element(positive_angles.begin(), positive_angles.end());
      } else if (negative_angles.size() > 0) {
        angle_diff =
            *std::max_element(negative_angles.begin(), negative_angles.end()) -
            *std::min_element(negative_angles.begin(), negative_angles.end());
      }
      if (fitness < 0.01 && angle_diff > 120) {
        cylinder_cloud->push_back(pcl::PointXYZ(center[0], center[1], 0));
        int r = rand() % 255;
        int g = rand() % 255;
        int b = rand() % 255;
        for (int j = 0; j < cluster->points.size(); ++j) {
          pcl::PointXYZRGB point;
          point.x = cluster->points[j].x;
          point.y = cluster->points[j].y;
          point.z = cluster->points[j].z;
          point.r = r;
          point.g = g;
          point.b = b;
          colored_cloud->points.push_back(point);
        }
      }
    }

    //        publishResult();

    Eigen::MatrixX2d cylinder_points(cylinder_cloud->points.size(), 2);
    for (int i = 0; i < cylinder_cloud->points.size(); ++i) {
      cylinder_points(i, 0) = cylinder_cloud->points[i].x;
      cylinder_points(i, 1) = cylinder_cloud->points[i].y;
    }
    return cylinder_points;
  }

  std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clustering(
      const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloudIn) {
    Eigen::Vector3f voxel_size(0.3, 0.3, 1.0);
    cutCloud(*cloudIn, voxel_size, voxelMap);
    for (auto it = voxelMap.begin(); it != voxelMap.end(); ++it) {
      it->second->instance_id = -1;
    }

    std::map<int, pcl::PointCloud<pcl::PointXYZ>::Ptr> clusters;
    std::map<int, std::vector<VoxelKey>> voxel_indices;
    int global_index = 0;
    for (auto it = voxelMap.begin(); it != voxelMap.end(); ++it) {
      pcl::PointCloud<pcl::PointXYZ>::Ptr cluster(
          new pcl::PointCloud<pcl::PointXYZ>);
      auto& cur_voxel = it->second;
      if (cur_voxel->plane) continue;
      if (cur_voxel->instance_id < 0) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(
            new pcl::PointCloud<pcl::PointXYZ>);
        *cloud = *cur_voxel->cloud();
        clusters[global_index] = cloud;
        voxel_indices[global_index] = std::vector<VoxelKey>();
        voxel_indices[global_index].push_back(it->first);
        cur_voxel->instance_id = global_index;
        global_index++;
      }

      std::vector<VoxelKey> neighbors;
      cur_voxel->getNeighbors(neighbors);
      for (VoxelKey& neighbor : neighbors) {
        auto neighbor_iter = voxelMap.find(neighbor);
        if (neighbor_iter == voxelMap.end()) continue;
        Voxel& neighbor_voxel = *neighbor_iter->second;
        if (neighbor_voxel.instance_id < 0) {
          neighbor_voxel.instance_id = cur_voxel->instance_id;
          voxel_indices[cur_voxel->instance_id].push_back(neighbor_iter->first);
          *clusters[cur_voxel->instance_id] += *neighbor_voxel.cloud();
        } else {
          // if neighbor has been assigned to a surface, try to merge
          if (neighbor_voxel.instance_id == cur_voxel->instance_id) continue;
          // merge neighbor_voxel to cur_voxel
          int origin_instance_id = neighbor_voxel.instance_id;
          for (VoxelKey& voxelKey : voxel_indices[neighbor_voxel.instance_id]) {
            voxelMap[voxelKey]->instance_id = cur_voxel->instance_id;
            *clusters[cur_voxel->instance_id] += *voxelMap[voxelKey]->cloud();
          }
          voxel_indices[cur_voxel->instance_id].insert(
              voxel_indices[cur_voxel->instance_id].end(),
              voxel_indices[origin_instance_id].begin(),
              voxel_indices[origin_instance_id].end());
          voxel_indices.erase(origin_instance_id);
          clusters.erase(origin_instance_id);
        }
      }
    }
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cluster_vec;
    for (auto it = clusters.begin(); it != clusters.end(); ++it) {
      auto& cluster = it->second;
      // compute W H D
      float min_x = 1000, max_x = -1000, min_y = 1000, max_y = -1000;
      for (int i = 0; i < cluster->points.size(); ++i) {
        if (cluster->points[i].x < min_x) min_x = cluster->points[i].x;
        if (cluster->points[i].x > max_x) max_x = cluster->points[i].x;
        if (cluster->points[i].y < min_y) min_y = cluster->points[i].y;
        if (cluster->points[i].y > max_y) max_y = cluster->points[i].y;
      }
      float w = max_x - min_x;
      float h = max_y - min_y;
      if (w > 1.5 || h > 1.5) continue;
      cluster_vec.push_back(it->second);
    }
    return cluster_vec;
  }

  Eigen::Vector2d fitCircle(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
    // variables to be optimized
    Eigen::Vector2d center = Eigen::Vector2d::Zero();

    // project points to 2D plane
    Eigen::Matrix2Xd points = Eigen::Matrix2Xd::Zero(2, cloud->points.size());
    for (int i = 0; i < cloud->points.size(); ++i) {
      points(0, i) = cloud->points[i].x;
      points(1, i) = cloud->points[i].y;
    }

    // optimize by Gauss-Newton method
    center = points.rowwise().mean();  // initialize center
    for (int i = 0; i < 10; ++i) {
      Eigen::Matrix<double, 2, 2> H = Eigen::Matrix<double, 2, 2>::Zero();
      Eigen::Matrix<double, 2, 1> b = Eigen::Matrix<double, 2, 1>::Zero();
      for (int i = 0; i < points.cols(); ++i) {
        double dist2 = (points.col(i) - center).squaredNorm();
        Eigen::Vector2d jacobi = 2 * (center - points.col(i));
        double residual = dist2 - radius * radius;
        Eigen::Matrix<double, 1, 1> residual_mat;
        residual_mat << residual;

        H += jacobi * jacobi.transpose();
        b += -jacobi * residual_mat;
      }
      Eigen::Matrix<double, 2, 1> delta = H.ldlt().solve(b);
      center += delta;
      if (delta.norm() < 1e-3) break;
    }
    return center;
  }

  void publishResult() {
    sensor_msgs::PointCloud2 laserCloudInNoPlaneMsg;
    pcl::toROSMsg(*cylinder_cloud, laserCloudInNoPlaneMsg);
    laserCloudInNoPlaneMsg.header = cloudHeader;
    laserCloudInNoPlaneMsg.header.frame_id = "map";
    cylinder_cloud_pub.publish(laserCloudInNoPlaneMsg);

    sensor_msgs::PointCloud2 colored_cloud_msg;
    pcl::toROSMsg(*colored_cloud, colored_cloud_msg);
    colored_cloud_msg.header = cloudHeader;
    colored_cloud_msg.header.frame_id = "map";
    colored_cloud_pub.publish(colored_cloud_msg);
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr removePlanes(
      const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloudIn) {
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inlier_indices(new pcl::PointIndices);
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(100);
    seg.setDistanceThreshold(0.01);  // set the threshold of plane detection

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOut(
        new pcl::PointCloud<pcl::PointXYZ>);
    *cloudOut = *cloudIn;
    while (ros::ok()) {
      seg.setInputCloud(cloudOut);
      seg.segment(*inlier_indices, *coefficients);
      if (inlier_indices->indices.size() <= 200) {
        break;
      }

      pcl::ExtractIndices<pcl::PointXYZ> extract;
      extract.setInputCloud(cloudOut);
      extract.setIndices(inlier_indices);
      extract.setNegative(true);  // remove the plane points
      extract.filter(*cloudOut);
    }
    return cloudOut;
  }

  void reset() {
    markerArray.markers.clear();
    cylinder_cloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
    non_plane_cloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
    non_ground_cloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
    colored_cloud.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
    voxelMap.clear();
  }

 private:
  ros::NodeHandle nh_;
  std_msgs::Header cloudHeader;
  const double radius = 0.5;

  VoxelMap voxelMap;

  pcl::PointCloud<pcl::PointXYZ>::Ptr non_ground_cloud;
  pcl::PointCloud<pcl::PointXYZ>::Ptr non_plane_cloud;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cylinder_cloud;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cloud;

  ros::Publisher cylinder_marker_pub;
  ros::Publisher cylinder_cloud_pub;
  ros::Publisher non_plane_cloud_pub;
  ros::Publisher colored_cloud_pub;
  visualization_msgs::MarkerArray markerArray;
};

#endif  // SRC_EXTRACT_CYLINDER_H
