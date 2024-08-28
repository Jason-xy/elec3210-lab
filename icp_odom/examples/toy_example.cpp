/**
** Created by Zhijian QIAO.
** UAV Group, Hong Kong University of Science and Technology
** email: zqiaoac@connect.ust.hk
**/
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <fstream>

#include "global_definition.h"
#include "icp.h"
#include "parameters.h"

int main(int argc, char **argv) {
  std::string source_pcd_file = WORK_SPACE_PATH + "/../dataset/src.pcd";
  std::string target_pcd_file = WORK_SPACE_PATH + "/../dataset/tgt.pcd";

  pcl::PointCloud<pcl::PointXYZ>::Ptr src_cloud(
      new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr tar_cloud(
      new pcl::PointCloud<pcl::PointXYZ>);
  pcl::io::loadPCDFile(source_pcd_file, *src_cloud);
  pcl::io::loadPCDFile(target_pcd_file, *tar_cloud);

  Eigen::Matrix4d transform = Eigen::Matrix4d::Identity();
  transform.block<3, 3>(0, 0) =
      Eigen::AngleAxisd(M_PI / 16, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  transform(0, 3) = 1.0;
  pcl::transformPointCloud(*src_cloud, *src_cloud, transform);

  // icp
  params::max_distance = 2.0;
  params::max_iterations = 100;
  Eigen::Matrix4d transformation =
      icp_registration(src_cloud, tar_cloud, transform);

  // visualization
  pcl::PointCloud<pcl::PointXYZ>::Ptr src_cloud_transformed(
      new pcl::PointCloud<pcl::PointXYZ>);
  pcl::transformPointCloud(*src_cloud, *src_cloud_transformed, transformation);
  pcl::visualization::PCLVisualizer viewer("ICP demo");
  int v1(0);
  int v2(1);
  viewer.createViewPort(0.0, 0.0, 0.5, 1.0, v1);
  viewer.createViewPort(0.5, 0.0, 1.0, 1.0, v2);
  viewer.setBackgroundColor(0, 0, 0, v1);
  viewer.setBackgroundColor(0, 0, 0, v2);
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>
      src_cloud_color(src_cloud, 255, 0, 0);
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>
      tar_cloud_color(tar_cloud, 0, 255, 0);
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>
      src_cloud_transformed_color(src_cloud_transformed, 0, 0, 255);
  viewer.addPointCloud<pcl::PointXYZ>(src_cloud, src_cloud_color, "src_cloud",
                                      v1);
  viewer.addPointCloud<pcl::PointXYZ>(tar_cloud, tar_cloud_color, "tar_cloud",
                                      v1);
  viewer.addPointCloud<pcl::PointXYZ>(src_cloud_transformed,
                                      src_cloud_transformed_color,
                                      "src_cloud_transformed", v2);
  viewer.addPointCloud<pcl::PointXYZ>(tar_cloud, tar_cloud_color, "tar_cloud2",
                                      v2);
  viewer.spin();

  //    Eigen::Matrix4d T_gt = transformation;
  //    // write to file
  //    std::ofstream fout;
  //    fout.open(WORK_SPACE_PATH + "/../dataset/true_tf.txt");
  //    fout << T_gt(0, 0) << " " << T_gt(0, 1) << " " << T_gt(0, 2) << " " <<
  //    T_gt(0, 3)
  //    << " " << T_gt(1, 0) << " " << T_gt(1, 1) << " " << T_gt(1, 2) << " " <<
  //    T_gt(1, 3)
  //    << " " << T_gt(2, 0) << " " << T_gt(2, 1) << " " << T_gt(2, 2) << " " <<
  //    T_gt(2, 3)
  //    << " " << T_gt(3, 0) << " " << T_gt(3, 1) << " " << T_gt(3, 2) << " " <<
  //    T_gt(3, 3) << endl; fout.close(); std::cout << "T_gt: " << std::endl <<
  //    T_gt << std::endl;

  return 0;
}