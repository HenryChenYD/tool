/*
 * pcd_viewer.cc
 *
 *  Created on: Jan 24, 2019
 *      Author: Henry Chen
 */

#include <ros/ros.h>
#include <iostream>
#include <tf/tf.h>

#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/io/pcd_io.h>

#include <eigen_conversions/eigen_msg.h>

#include "test_encrypt/encrypt_ct.h"

int main (int argc, char** argv) {
  TESTENCRYPT::CheckAuth ();

  ros::init (argc, argv, "PCD_Viewer");
  ros::NodeHandle nh ("~");

  ros::Publisher pub_pointcloud2 = nh.advertise<sensor_msgs::PointCloud2>("/pcd_viewer", 10, true);
  ros::Publisher pub_pcd_source = nh.advertise<sensor_msgs::PointCloud2>("/pcd_source", 10, true);

  /* Create Point Cloud */
  pcl::PointCloud<pcl::PointXYZ> pcl_cloud_in, pcl_cloud_tmp;
  sensor_msgs::PointCloud2 msg_cloud_tmp, msg_cloud_out;

  /* Read PCD File */
  /* Read Wrong */
  if (-1
      == pcl::io::loadPCDFile<pcl::PointXYZ> ("/home/henry001/laser_bag/al/2019-01-18-11-10-19.pcd",
                                              pcl_cloud_in)) {
    return 0;
  }

  Eigen::Quaterniond rot;
//  rot.setFromTwoVectors(
//      Eigen::Vector3d(0, 1, 0),
//      Eigen::Vector3d(-0.05124, 0.997991, 0.0372494));
  rot.setFromTwoVectors(
      Eigen::Vector3d(0, 1, 0),
      Eigen::Vector3d(-0.0604646, 0.997991, 0.0188966));

  Eigen::Affine3d e_tf = Eigen::Affine3d::Identity();
  e_tf.linear() = rot.toRotationMatrix();

  geometry_msgs::Transform g_tf;

  tf::transformEigenToMsg(e_tf, g_tf);

  tf::Transform tmp;
//  tf::Quaternion qua_tf;
  tf::transformMsgToTF(g_tf, tmp);

//  tmp.setIdentity();
//  tmp.setRotation(qua_tf);

  pcl_ros::transformPointCloud(pcl_cloud_in, pcl_cloud_tmp, tmp.inverse());

  pcl::toROSMsg (pcl_cloud_in, msg_cloud_tmp);
  msg_cloud_tmp.header.stamp = ros::Time::now ();
  msg_cloud_tmp.header.frame_id = "lidar_map";

  pcl::toROSMsg(pcl_cloud_tmp, msg_cloud_out);
  msg_cloud_out.header.stamp = ros::Time::now();
  msg_cloud_out.header.frame_id = "lidar_map";

  /* Show the point cloud */
  pub_pcd_source.publish(msg_cloud_tmp);
  pub_pointcloud2.publish(msg_cloud_out);

  ros::spin();

  return 0;
}
