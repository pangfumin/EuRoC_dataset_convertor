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
  auto image_left_folder = save_folder+ "/left";
  auto image_front_folder = save_folder+ "/front";



  if (!common::pathExists(save_folder)) {
      common::createPath(save_folder);
  }


    if (!common::pathExists(image_left_folder)) {
        common::createPath(image_left_folder);
    }
    if (!common::pathExists(image_front_folder)) {
        common::createPath(image_front_folder);
    }




    std::string left_image_list_file = save_folder+ "/image_left.txt";
    std::string front_image_list_file = save_folder+ "/image_front.txt";

    std::ofstream left_image_ofs(left_image_list_file);
    std::ofstream front_image_ofs(front_image_list_file);





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
  int camera_left = 0;
  int camera_front = 0;



  std::vector<uint64_t> lidar_ts_vec;
  std::vector<uint64_t> camera0_ts_vec;
  std::vector<uint64_t> camera1_ts_vec;
  std::vector<uint64_t> camera2_ts_vec;
  std::vector<uint64_t> camera3_ts_vec;



  for (auto bagIt : view) {
      string topic = bagIt.getTopic();


      if (topic == "/gmsl_cam/image_left/compressed") {
          sensor_msgs::CompressedImage::ConstPtr image =
                  bagIt.instantiate<sensor_msgs::CompressedImage>();

          cv_bridge::CvImagePtr cv_ptr;
          cv_ptr = cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::BGR8);
          cv::Mat image_cv = cv_ptr->image;
          cv::imshow("image left", image_cv);
          cv::waitKey(2);

          std::string image_name =
                  std::string(6 - std::to_string(camera_left).length(), '0') + std::to_string(camera_left) + ".png";
          std::stringstream ss;
          ss << image_left_folder << "/" << image_name;
          cv::imwrite(ss.str(), image_cv);

          left_image_ofs << std::to_string(bagIt.getTime().toNSec()) << " " << "image0/" << image_name << std::endl;
          camera_left++;
      }

      if (topic == "/gmsl_cam/image_front/compressed") {

          sensor_msgs::CompressedImage::ConstPtr image =
                  bagIt.instantiate<sensor_msgs::CompressedImage>();


          cv_bridge::CvImagePtr cv_ptr;
          cv_ptr = cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::BGR8);
          cv::Mat image_cv = cv_ptr->image;
          cv::imshow("image right", image_cv);
          cv::waitKey(2);


          std::string image_name =
                  std::string(6 - std::to_string(camera_front).length(), '0') + std::to_string(camera_front) + ".png";
          std::stringstream ss;
          ss << image_front_folder << "/" << image_name;
          cv::imwrite(ss.str(), image_cv);

          front_image_ofs << std::to_string(bagIt.getTime().toNSec()) << " " << "image1/" << image_name << std::endl;

          camera_front++;
      }
  }


  left_image_ofs.close();
  front_image_ofs.close();




  ros::shutdown();

}
