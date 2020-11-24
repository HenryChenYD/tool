/*
 * test.cc
 *
 *  Created on: Jan 27, 2019
 *      Author: Henry Chen
 */



#include <ros/ros.h>
#include <iostream>
#include <tf/tf.h>

#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/io/pcd_io.h>
#include <pcl/features/normal_3d.h>

#include <eigen_conversions/eigen_msg.h>

#include "test_encrypt/encrypt_ct.h"

ros::Subscriber sub_;
ros::Publisher pub_pointcloud_Processed_;

tf::Transform transform_for_Lidar_, transform_for_map_;

void subPointcloudAndPublishAfterTransform(const sensor_msgs::PointCloud2::ConstPtr & msgs) {
  sensor_msgs::PointCloud2 msg_out;

  pcl_ros::transformPointCloud ("velodyne_processed",
                                transform_for_Lidar_.inverse(),
                                *msgs,
                                msg_out);

  pub_pointcloud_Processed_.publish(msg_out);
}

void subPointcloud(const sensor_msgs::PointCloud2::ConstPtr & msgs) {
  pcl::PointCloud<pcl::PointXYZ> pcl_in;
  pcl::fromROSMsg(*msgs, pcl_in);

  std::vector<int> indices;
  double max = 10.0, min = -10.0;
  for (size_t i = 0; i < pcl_in.size(); i++) {
    if (pcl_in.points[i].z < 0.0 &&
        pcl_in.points[i].x < max &&
        pcl_in.points[i].x > min &&
        pcl_in.points[i].y < max &&
        pcl_in.points[i].y > min &&
        (pcl_in.points[i].x < -2.0 ||
            pcl_in.points[i].x > 0.0 ||
            pcl_in.points[i].y < -1.0 ||
            pcl_in.points[i].y > 1.0)) {
      indices.push_back(i);
    }
  }
  pcl::PointCloud<pcl::PointXYZ> pcl_out(pcl_in, indices);

  sensor_msgs::PointCloud2 msg_out;
  pcl::toROSMsg(pcl_out, msg_out);
  msg_out.header = msgs->header;
  pub_pointcloud_Processed_.publish(msg_out);

  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
  ne.setInputCloud(pcl_out.makeShared());
  ne.setRadiusSearch (20.0);
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
  std::cerr << "Begin to compute normals!\n";
  ne.compute (*cloud_normals);
  std::cerr << "Finish!\n";

  for (int ix = 0; ix < cloud_normals->points.size (); ix++) {
    if (std::isnan (cloud_normals->points[ix].normal_x)
        || std::isnan (cloud_normals->points[ix].normal_y)
        || std::isnan (cloud_normals->points[ix].normal_z)) {
      std::cerr << "ERROR normal!";
    } else {
      std::cout << "x: " << cloud_normals->points[ix].normal_x
          << " y: " << cloud_normals->points[ix].normal_y
          << " z: " << cloud_normals->points[ix].normal_z
          << std::endl;
    }
  }

  sub_.shutdown();
}

void setTransformForLidar() {
  Eigen::Quaterniond rot;

  rot.setFromTwoVectors (Eigen::Vector3d (0, 0, 1),
                         Eigen::Vector3d (0.03967, -0.01902, 0.99903));
  Eigen::Affine3d e_tf = Eigen::Affine3d::Identity();
  e_tf.linear() = rot.toRotationMatrix();

  geometry_msgs::Transform g_tf;
  tf::transformEigenToMsg(e_tf, g_tf);
  tf::transformMsgToTF(g_tf, transform_for_Lidar_);

  return;
}

void setTransformForMap() {
  Eigen::Quaterniond rot;

  // sunnai new
  rot.setFromTwoVectors(Eigen::Vector3d(0, 1, 0),
                        Eigen::Vector3d(-0.0358502, 0.998966, 0.0284049));
  // sunnai old
//  rot.setFromTwoVectors(Eigen::Vector3d(0, 1, 0),
//                        Eigen::Vector3d(-0.0604646, 0.997991, 0.0188966));

//  rot.setFromTwoVectors(Eigen::Vector3d (0, 1, 0),
//                        Eigen::Vector3d (-0.01902, 0.99903, 0.03967));

  Eigen::Affine3d e_tf = Eigen::Affine3d::Identity();
  e_tf.linear() = rot.toRotationMatrix();

  geometry_msgs::Transform g_tf;
  tf::transformEigenToMsg(e_tf, g_tf);
  tf::transformMsgToTF(g_tf, transform_for_map_);

  std::cout << "x: " << transform_for_map_.inverse().getOrigin().x() << std::endl
      << "y: " << transform_for_map_.inverse().getOrigin().y() << std::endl
      << "z: " << transform_for_map_.inverse().getOrigin().z() << std::endl
      << "x: " << transform_for_map_.inverse().getRotation().x() << std::endl
      << "y: " << transform_for_map_.inverse().getRotation().y() << std::endl
      << "z: " << transform_for_map_.inverse().getRotation().z() << std::endl
      << "w: " << transform_for_map_.inverse().getRotation().w() << std::endl;

  return;
}

int main (int argc, char** argv) {
  TESTENCRYPT::CheckAuth ();

  ros::init (argc, argv, "TEST");
  ros::NodeHandle nh ("~");

  setTransformForLidar();
  setTransformForMap();

  ros::Publisher pub_pointcloud2 = nh.advertise<sensor_msgs::PointCloud2>("/pcd_viewer", 10, true);
  ros::Publisher pub_pcd_source = nh.advertise<sensor_msgs::PointCloud2>("/pcd_source", 10, true);

  sub_ = nh.subscribe<sensor_msgs::PointCloud2>("/velodyne_points", 2000, subPointcloudAndPublishAfterTransform);
  pub_pointcloud_Processed_ = nh.advertise<sensor_msgs::PointCloud2>("/pointcloud_Processed", 1000, true);

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

  pcl_ros::transformPointCloud(pcl_cloud_in, pcl_cloud_tmp, transform_for_map_.inverse());

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
