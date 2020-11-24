#include <ros/ros.h>
#include <iostream>
#include <fstream>
#include <vector>
#include <set>
#include <iterator>
#include <string>
#include <algorithm>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include "test_encrypt/encrypt_ct.h"

/*
 * A class to create and write data in a csv file.
 */
class CSVWriter
{
  std::string fileName;
  std::string delimeter;
  int linesCount;

public:
  CSVWriter(std::string filename, std::string delm = " , ") :
      fileName(filename), delimeter(delm), linesCount(0)
  {}
  /*
   * Member function to store a range as comma seperated value
   */
  template<typename T>
  void addDatainRow(T first, T last);

  void subb(const sensor_msgs::PointCloud2::ConstPtr &msgs);
};

/*
 * This Function accepts a range and appends all the elements in the range
 * to the last row, seperated by delimeter (Default is comma)
 */
template<typename T>
void CSVWriter::addDatainRow(T first, T last)
{
  std::fstream file;
  // Open the file in truncate mode if first line else in Append Mode
  file.open(fileName, std::ios::out | (linesCount ? std::ios::app : std::ios::trunc));

  // Iterate over the range and add each lement to file seperated by delimeter.
  for (; first != last; )
  {
    file << *first;
    if (++first != last)
      file << delimeter;
  }
  file << "\n";
  linesCount++;

  // Close the file
  file.close();
}

void CSVWriter::subb(const sensor_msgs::PointCloud2::ConstPtr &msgs) {
  pcl::PointCloud<pcl::PointXYZ> pcl_in;
  pcl::fromROSMsg(*msgs, pcl_in);

  std::string str = "xyz";
  addDatainRow(str.begin(), str.end());

  double arr [3];
  for (pcl::PointCloud<pcl::PointXYZ>::iterator it_pcl = pcl_in.points.begin();
      it_pcl != pcl_in.points.end();
      it_pcl++){
    arr [0] = it_pcl->x;
    arr [1] = it_pcl->y;
    arr [2] = it_pcl->z;

    // Wrote an elements in array to csv file.
    addDatainRow(arr , arr + sizeof(arr) / sizeof(double));
  }
  std::cout << "Success! Convert sensor_msgs/PointCloud2 to CSV file.\n"
      << "Topic: /laser_cloud_surround\n"
      << "File name: " << fileName.c_str() << "\n";
  ros::shutdown();
}

int main(int argc, char** argv)
{
  TESTENCRYPT::CheckAuth();
  ros::init(argc, argv, "PC2_to_CSV");
  ros::NodeHandle nh;

  // Creating an object of CSVWriter
  CSVWriter writer("example0.csv");

  ros::Subscriber sub =
      nh.subscribe<sensor_msgs::PointCloud2>(
          "/laser_cloud_surround", 1, boost::bind(&CSVWriter::subb, writer, _1));

/*
  std::vector<std::string> dataList_1 = { "20", "hi", "99" };

  // Adding vector to CSV File
  writer.addDatainRow(dataList_1.begin(), dataList_1.end());

  // Create a set of integers
  std::set<int> dataList_2 = { 3, 4, 5};

  // Adding Set to CSV File
  writer.addDatainRow(dataList_2.begin(), dataList_2.end());

  std::string str = "abc";

  // Adding characters in a string in csv file.
  writer.addDatainRow(str.begin(), str.end());
*/

  ros::spin();
  return 0;
}
