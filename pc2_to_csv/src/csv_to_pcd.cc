/*
 * csv_to_pcd.cc
 *
 *  Created on: Jan 10, 2019
 *      Author: Henry Chen
 */
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
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include "test_encrypt/encrypt_ct.h"

using namespace std;

void validateArgs(int argc, char *argv[]);
/*
 * A class to create and write data in a pcd file.
 */
class PCDWriter
{
  std::string in_fileName_;
  std::string out_fileName_;

public:
  PCDWriter (std::string in_filename, std::string out_filename) :
    in_fileName_(in_filename), out_fileName_(out_filename)
  {
    write();

    return;
  }

  void write ();
};

void PCDWriter::write()
{
  ifstream csvFile;
  csvFile.open(in_fileName_.c_str());

  if (!csvFile.is_open())
  {
    cout << "Path Wrong!!!!" << endl;
    exit(EXIT_FAILURE);
  }

  pcl::PointCloud<pcl::PointXYZ> pointCloud_in;

  string line;
  vector <string> vec;
  getline(csvFile, line); // skip the 1st line

  while (getline (csvFile, line)) {
    if (line.empty ()) // skip empty lines:
    {
      continue;
    }

    istringstream iss (line);
    string lineStream;
    string::size_type sz;

    vector<float> row;

    while (getline (iss, lineStream, ',')) {
      row.push_back (stold (lineStream, &sz)); // convert to float
    }

    pcl::PointXYZ tmp;
    tmp.x = row[0];
    tmp.y = row[1];
    tmp.z = row[2];
    tmp.data[3] = 1.0f;

    pointCloud_in.push_back(tmp);
  }

  csvFile.close();

  pcl::io::savePCDFileBinaryCompressed (out_fileName_.c_str(), pointCloud_in);

  std::cout << "Finish!\n"
      << "The PCD file path: " << out_fileName_ << std::endl;

  return;
}

int main(int argc, char** argv)
{
  TESTENCRYPT::CheckAuth();

  ros::init(argc, argv, "CSV_to_PCD");
  ros::NodeHandle nh("~");

  std::string input_file, output_file;

  input_file = nh.param("input_csv_file", input_file);
  output_file = nh.param("output_pcd_file", output_file);

  //validateArgs(argc, argv);

  if (!input_file.empty() && !output_file.empty()){
    // Creating an object of PCDWriter
    PCDWriter writer(input_file, output_file);
  } else {
    std::cout << "Error: input_csv_file or output_pcd_file not set!\n";
  }

  ros::spinOnce();

  return 0;
}

void validateArgs(int argc, char *argv[])
{
  if (argc != 3)
  {
    std::cerr << "Wrong number of arguments, usage " << argv[0] << " reference.csv reading.csv" << std::endl;
    std::cerr << "Will create 3 vtk files for inspection: ./test_ref.vtk, ./test_data_in.vtk and ./test_data_out.vtk" << std::endl;
    std::cerr << std::endl << "2D Example:" << std::endl;
    std::cerr << "  " << argv[0] << " ../../examples/data/2D_twoBoxes.csv ../../examples/data/2D_oneBox.csv" << std::endl;
    std::cerr << std::endl << "3D Example:" << std::endl;
    std::cerr << "  " << argv[0] << " ../../examples/data/car_cloud400.csv ../../examples/data/car_cloud401.csv" << std::endl;
    exit(1);
  }
}
