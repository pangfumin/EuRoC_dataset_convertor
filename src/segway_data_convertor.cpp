/*********************************************************************************
 *  OKVIS - Open Keyframe-based Visual-Inertial SLAM
 *  Copyright (c) 2015, Autonomous Systems Lab / ETH Zurich
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *
 *   * Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright notice,
 *     this list of conditions and the following disclaimer in the documentation
 *     and/or other materials provided with the distribution.
 *   * Neither the name of Autonomous Systems Lab / ETH Zurich nor the names of
 *     its contributors may be used to endorse or promote products derived from
 *     this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 *  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 *  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 *  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 *  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 *  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 *  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 *  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 *  Created on: Jan 13, 2016
 *      Author: Stefan Leutenegger (s.leutenegger@imperial.ac.uk)
 *********************************************************************************/

/**
 * @file dataset_convertor.cpp
 * @brief Source file for the VioParametersReader class.
 * @author Stefan Leutenegger
 * @author Andrea Nicastro
 */

#include <vector>
#include <map>
#include <memory>
#include <stdio.h>
#include <iostream>
#include <fstream>
#include <algorithm>

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wnon-virtual-dtor"
#pragma GCC diagnostic ignored "-Woverloaded-virtual"
#include <ros/ros.h>
#include <ros/console.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <rosbag/chunked_file.h>
#include <cv_bridge/cv_bridge.h>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#pragma GCC diagnostic pop


#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <boost/foreach.hpp>
#include <boost/filesystem.hpp>
#include <boost/filesystem/path.hpp>
#include "okvis/file-system-tools.h"
#include <pcl/io/pcd_io.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>

using namespace boost::filesystem;
using std::vector;
using std::string;
using std::map;
using std::cout;
using std::endl;
using std::ofstream;
using std::shared_ptr;

const string RESET = "\033[0m";
const string BLACK = "0m";
const string RED = "1m";
const string GREEN = "2m";
const string BOLD = "\033[1;3";
const string REGULAR = "\033[0;3";
const string UNDERLINE = "\033[4;3";
const string BACKGROUND = "\033[4";

const int DOUBLE_PRECISION = 17;



string colouredString(string str, string colour, string option)
{
  return option + colour + str + RESET;
}


int writePointCloudToBinFile(pcl::PointCloud<pcl::PointXYZI>& pointcloud, const std::string bin_filename) {
    // load point cloud
    std::ofstream output(bin_filename, std::ios::out | std::ios::binary);
    if(!output.is_open()){
        std::cerr << "Could not read file: " << bin_filename << std::endl;
        exit(EXIT_FAILURE);
    }

    for (auto i : pointcloud) {
        output.write((char *) &i.x, 3*sizeof(float));
        output.write((char *) &i.intensity, sizeof(float));
    }

    output.close();
    return pointcloud.size();
}


int main(int argc, char **argv)
{

  cout << colouredString("Initializing ROS node:", RED, BOLD) << endl;

  ros::init(argc, argv, "dataset_converter");

  ros::NodeHandle nh;

  cout << colouredString("DONE!", GREEN, BOLD) << endl;

  cout << colouredString("Initializing sensor information:", RED, BOLD) << endl;

  string path(argv[1]);
  size_t pos = path.find_last_of("/");
  size_t pos_dot = path.find_last_of(".");

  string bagname;
  string bagpath;
  if (pos == string::npos) {
    cout
        << colouredString(
            "Relative path are not supported. Use an absolute path instead."
            "For example: roslaunch okvis_ros convert_datasert.launch bag:= /absolute/path/here",
            RED, BOLD) << endl;
    exit (EXIT_FAILURE);
  } else {
    bagname = path.substr(pos + 1, pos_dot - pos - 1);
    bagpath = path.substr(0, pos + 1);
  }

  cout << colouredString("Reading bag:", RED, BOLD) << endl;

  std::string save_folder = bagpath + bagname;
  auto velodyne_folder = save_folder+ "/velodyne";
  auto image_left_folder = save_folder+ "/image_L";
  auto image_right_folder = save_folder+ "/image_R";
  auto image_0_folder = save_folder+ "/image_0";
  auto image_1_folder = save_folder+ "/image_1";
  auto image_2_folder = save_folder+ "/image_2";
  auto image_3_folder = save_folder+ "/image_3";



  if (!common::pathExists(save_folder)) {
      common::createPath(save_folder);
  }

    if (!common::pathExists(velodyne_folder)) {
        common::createPath(velodyne_folder);
    }
    if (!common::pathExists(image_left_folder)) {
        common::createPath(image_left_folder);
    }
    if (!common::pathExists(image_right_folder)) {
        common::createPath(image_right_folder);
    }

    if (!common::pathExists(image_0_folder)) {
        common::createPath(image_0_folder);
    }
    if (!common::pathExists(image_1_folder)) {
        common::createPath(image_1_folder);
    }

    if (!common::pathExists(image_2_folder)) {
        common::createPath(image_2_folder);
    }
    if (!common::pathExists(image_3_folder)) {
        common::createPath(image_3_folder);
    }

    std::string velodyne_list_file = save_folder+ "/velodyne.txt";
    std::string left_image_list_file = save_folder+ "/image_L.txt";
    std::string right_image_list_file = save_folder+ "/image_R.txt";
    std::string image0_list_file = save_folder+ "/image0.txt";
    std::string image1_list_file = save_folder+ "/image1.txt";
    std::string image2_list_file = save_folder+ "/image2.txt";
    std::string image3_list_file = save_folder+ "/image3.txt";

    std::ofstream velodyne_ofs(velodyne_list_file);
    std::ofstream left_image_ofs(left_image_list_file);
    std::ofstream right_image_ofs(right_image_list_file);
    std::ofstream image0_ofs(image0_list_file);
    std::ofstream image1_ofs(image1_list_file);
    std::ofstream image2_ofs(image2_list_file);
    std::ofstream image3_ofs(image3_list_file);




    rosbag::Bag bag;
  cout << colouredString("\tOpening bag...", RED, REGULAR);
  bag.open(argv[1], rosbag::bagmode::Read);
  cout << colouredString("\t[DONE!]", GREEN, REGULAR) << endl;

  cout << colouredString("\tQuering topics bag...", RED, REGULAR);

  vector < string > topic_list;

  rosbag::View view(bag);

  vector<const rosbag::ConnectionInfo *> bag_info = view.getConnections();
  std::set < string > bag_topics;

  for (const rosbag::ConnectionInfo *info : bag_info) {
    string topic_name;
    topic_name = info->topic;

    std::cout << "topic: " << topic_name << std::endl;

    topic_list.push_back(topic_name);
  }

  view.addQuery(bag, rosbag::TopicQuery(topic_list));

  cout << colouredString("\t[DONE!]", GREEN, REGULAR) << endl;

  cout << colouredString("\tOpening file streams...", RED, REGULAR);

  double view_size = view.size();
  std::cout << "view_size: " << view_size << std::endl;
  double counter = 0;
  int velodyne_points_cnt = 0;
  int camera_left = 0;
  int camera_right = 0;

  int camera0 = 0;
  int camera1 = 0;
  int camera2 = 0;
  int camera3 = 0;

  uint64_t last_lidar_ts = 0;
  uint64_t last_camera0_ts = 0;
  uint64_t last_camera1_ts = 0;
  uint64_t last_camera2_ts = 0;
  uint64_t last_camera3_ts = 0;

  std::vector<uint64_t> lidar_ts_vec;
  std::vector<uint64_t> camera0_ts_vec;
  std::vector<uint64_t> camera1_ts_vec;
  std::vector<uint64_t> camera2_ts_vec;
  std::vector<uint64_t> camera3_ts_vec;



  for (auto bagIt : view) {
    string topic = bagIt.getTopic();

    if (topic == "/velodyne_points" ) {
      // todo: save
        sensor_msgs::PointCloud2::ConstPtr pointcloud =
                bagIt.instantiate<sensor_msgs::PointCloud2>();

        if (last_lidar_ts >= bagIt.getTime().toNSec()) continue;
        last_lidar_ts = bagIt.getTime().toNSec();
        lidar_ts_vec.push_back(bagIt.getTime().toNSec());

        pcl::PointCloud<pcl::PointXYZI>::Ptr lidar(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::PCLPointCloud2 pcl_pc2;
        pcl_conversions::toPCL(*pointcloud,pcl_pc2);
        pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZI>::Ptr out_cloud(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::fromPCLPointCloud2(pcl_pc2,*temp_cloud);
        for (auto pt: *temp_cloud) {
            pcl::PointXYZI p;
            p.x = pt.x;
            p.y = pt.y;
            p.z = pt.z;
            out_cloud->push_back(p);
        }

        std::string lidar_name = std::string(6 - std::to_string(velodyne_points_cnt).length(), '0') + std::to_string(velodyne_points_cnt) + ".bin";
        std::stringstream ss;
        ss <<velodyne_folder << "/" << lidar_name;

        velodyne_ofs << std::to_string(bagIt.getTime().toNSec()) << " " << "velodyne/"<< lidar_name << std::endl;

        writePointCloudToBinFile(*out_cloud, ss.str());

        velodyne_points_cnt ++;
    }

    if (topic == "/stereo/cameraL" ) {
      sensor_msgs::Image::ConstPtr image =
              bagIt.instantiate<sensor_msgs::Image>();




      cv_bridge::CvImagePtr cv_ptr;
      cv_ptr = cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::BGR8);
      cv::Mat image_cv = cv_ptr->image;
//      cv::imshow("image left", image_cv);
//      cv::waitKey(20);

        std::string image_name = std::string(6 - std::to_string(camera_left).length(), '0') + std::to_string(camera_left) + ".png";
        std::stringstream ss;
        ss <<image_left_folder << "/" << image_name;
        cv::imwrite(ss.str(), image_cv);

      left_image_ofs << std::to_string(bagIt.getTime().toNSec()) << " " << "image0/"<< image_name << std::endl;
      camera_left ++;
    }

    if (topic == "/stereo/cameraR" ) {

      sensor_msgs::Image::ConstPtr image =
              bagIt.instantiate<sensor_msgs::Image>();



      cv_bridge::CvImagePtr cv_ptr;
      cv_ptr = cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::BGR8);
      cv::Mat image_cv = cv_ptr->image;
//      cv::imshow("image right", image_cv);
//      cv::waitKey(20);


        std::string image_name = std::string(6 - std::to_string(camera_right).length(), '0') + std::to_string(camera_right) + ".png";
        std::stringstream ss;
        ss <<image_right_folder << "/" << image_name;
        cv::imwrite(ss.str(), image_cv);

        right_image_ofs << std::to_string(bagIt.getTime().toNSec()) << " " << "image1/"<< image_name << std::endl;

      camera_right ++;
    }


      if (topic == "/camera/post1/compressed" ) {
          sensor_msgs::CompressedImage::ConstPtr image =
                  bagIt.instantiate<sensor_msgs::CompressedImage>();


          if (last_camera0_ts >= bagIt.getTime().toNSec()) continue;
          last_camera0_ts = bagIt.getTime().toNSec();
          camera0_ts_vec.push_back(bagIt.getTime().toNSec());

          cv_bridge::CvImagePtr cv_ptr;
          cv_ptr = cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::BGR8);
          cv::Mat image_cv = cv_ptr->image;
//      cv::imshow("image left", image_cv);
//      cv::waitKey(20);

          std::string image_name =
                  std::string(6 - std::to_string(camera0).length(), '0') + std::to_string(camera0) + ".png";
          std::stringstream ss;
          ss <<image_0_folder << "/" << image_name;
          cv::imwrite(ss.str(), image_cv);

          image0_ofs << std::to_string(bagIt.getTime().toNSec()) << " " << "image0/"<< image_name << std::endl;
          camera0 ++;
      }

      if (topic == "/camera/post2/compressed" ) {
          sensor_msgs::CompressedImage::ConstPtr image =
                  bagIt.instantiate<sensor_msgs::CompressedImage>();

          if (last_camera1_ts >= bagIt.getTime().toNSec()) continue;
          last_camera1_ts = bagIt.getTime().toNSec();
          camera1_ts_vec.push_back(bagIt.getTime().toNSec());

          cv_bridge::CvImagePtr cv_ptr;
          cv_ptr = cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::BGR8);
          cv::Mat image_cv = cv_ptr->image;
//      cv::imshow("image left", image_cv);
//      cv::waitKey(20);

          std::string image_name =
                  std::string(6 - std::to_string(camera1).length(), '0') + std::to_string(camera1) + ".png";
          std::stringstream ss;
          ss <<image_1_folder << "/" << image_name;
          cv::imwrite(ss.str(), image_cv);

          image1_ofs << std::to_string(bagIt.getTime().toNSec()) << " " << "image1/"<< image_name << std::endl;
          camera1 ++;
      }

      if (topic == "/camera/post3/compressed" ) {
          sensor_msgs::CompressedImage::ConstPtr image =
                  bagIt.instantiate<sensor_msgs::CompressedImage>();

          if (last_camera2_ts >= bagIt.getTime().toNSec()) continue;
          last_camera2_ts = bagIt.getTime().toNSec();
          camera2_ts_vec.push_back(bagIt.getTime().toNSec());

          cv_bridge::CvImagePtr cv_ptr;
          cv_ptr = cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::BGR8);
          cv::Mat image_cv = cv_ptr->image;
//      cv::imshow("image left", image_cv);
//      cv::waitKey(20);

          std::string image_name = std::string(6 - std::to_string(camera2).length(), '0') + std::to_string(camera2) + ".png";
          std::stringstream ss;
          ss <<image_2_folder << "/" << image_name;
          cv::imwrite(ss.str(), image_cv);

          image2_ofs << std::to_string(bagIt.getTime().toNSec()) << " " << "image2/"<< image_name << std::endl;
          camera2 ++;
      }

      if (topic == "/camera/post4/compressed" ) {
          sensor_msgs::CompressedImage::ConstPtr image =
                  bagIt.instantiate<sensor_msgs::CompressedImage>();

          if (last_camera3_ts >= bagIt.getTime().toNSec()) continue;
          last_camera3_ts = bagIt.getTime().toNSec();
          camera3_ts_vec.push_back(bagIt.getTime().toNSec());

          cv_bridge::CvImagePtr cv_ptr;
          cv_ptr = cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::BGR8);
          cv::Mat image_cv = cv_ptr->image;
//      cv::imshow("image left", image_cv);
//      cv::waitKey(20);

          std::string image_name = std::string(6 - std::to_string(camera3).length(), '0') + std::to_string(camera3) + ".png";
          std::stringstream ss;
          ss <<image_3_folder << "/" << image_name;
          cv::imwrite(ss.str(), image_cv);

          image3_ofs << std::to_string(bagIt.getTime().toNSec()) << " " << "image3/"<< image_name << std::endl;
          camera3 ++;
      }

      int cnt  = camera_right + camera_left + velodyne_points_cnt +
              camera0 + camera1 + camera2 + camera3;
      if (cnt % 100 ==0)
          std::cout<< "saving: " << cnt  << "/"  << view_size << std::endl;

  }



  std::cout << "lidar: " << lidar_ts_vec.front() << " "
            << lidar_ts_vec.back() << " "
            << (lidar_ts_vec.back() - lidar_ts_vec.front())/ 1e9 << "m "
            << lidar_ts_vec.size() << std::endl;

    std::cout << "camera0: " << camera0_ts_vec.front() << " "
              << camera0_ts_vec.back() << " "
              << (camera0_ts_vec.back() - camera0_ts_vec.front())/ 1e9 << "m "
              << camera0_ts_vec.size() << std::endl;

    std::cout << "camera1: " << camera1_ts_vec.front() << " "
              << camera1_ts_vec.back() << " "
              << (camera1_ts_vec.back() - camera1_ts_vec.front())/ 1e9 << "m "
              << camera1_ts_vec.size() << std::endl;

    std::cout << "camera2: " << camera2_ts_vec.front() << " "
              << camera2_ts_vec.back() << " "
              << (camera2_ts_vec.back() - camera2_ts_vec.front())/ 1e9 << "m "
              << camera2_ts_vec.size() << std::endl;

    std::cout << "camera3: " << camera3_ts_vec.front() << " "
              << camera3_ts_vec.back() << " "
              << (camera3_ts_vec.back() - camera3_ts_vec.front())/ 1e9 << "m "
              << camera3_ts_vec.size() << std::endl;




  velodyne_ofs.close();
  left_image_ofs.close();
  right_image_ofs.close();




  ros::shutdown();

}
