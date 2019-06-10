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





int main(int argc, char **argv)
{


  ros::init(argc, argv, "dataset_converter");

  ros::NodeHandle nh;

  std::string dataset_path = "/home/pang/cui_stereo_calib";
    std::string generated_rosbag_file = dataset_path + "/kalibr.bag";

    auto data_bag_ptr =  std::make_shared<rosbag::Bag>(generated_rosbag_file,
                                                   rosbag::bagmode::Read|rosbag::bagmode::Write);

//     /cam1/image_raw
    std::string camera0_topic = "/cam0/image_raw";
    std::string camera1_topic = "/cam1/image_raw";
    std::string left_camera_folder = dataset_path+ "/newCamPics/Camera_2L_recorder";
    std::string right_camera_folder = dataset_path+ "/newCamPics/Camera_1R_recorder";

    /// image0
    std::vector<std::string> left_image_filenames, right_image_filenames;
    common::getAllFilesInFolder(left_camera_folder, &left_image_filenames);
    std::cout<<"image0_list: " << left_image_filenames.size() << std::endl;

    std::sort(left_image_filenames.begin(),left_image_filenames.end(), [](std::string a, std::string b) {
        return !common::compareNumericPartsOfStrings(a,b);
    });

    /// image1
    common::getAllFilesInFolder(right_camera_folder, &right_image_filenames);
    std::cout<<"image1_list: " << right_image_filenames.size() << std::endl;
    std::sort(right_image_filenames.begin(),right_image_filenames.end(), [](std::string a, std::string b) {
        return !common::compareNumericPartsOfStrings(a,b);
    });

    ros::Time t(ros::Time::now());
    for (int i =0; i < left_image_filenames.size(); i++) {
        std::cout << left_image_filenames.at(i) << std::endl;
        t =  ros::Time(t.toSec() + 1);


        cv::Mat image0 = cv::imread(left_image_filenames.at(i), CV_LOAD_IMAGE_GRAYSCALE);
        cv::Mat image1 = cv::imread(right_image_filenames.at(i), CV_LOAD_IMAGE_GRAYSCALE);
        cv::imshow("left", image0);
        cv::imshow("right", image1);
        cv::waitKey(1000);

        cv_bridge::CvImage out_msg;
        std_msgs::Header header;
        header.stamp = t;
        out_msg.header   = header;
        out_msg.encoding = sensor_msgs::image_encodings::TYPE_8UC1;

        out_msg.image    = image0; // Your cv::Mat
        data_bag_ptr->write(camera0_topic, t, out_msg);

        out_msg.image    = image1; // Your cv::Mat
        data_bag_ptr->write(camera1_topic, t, out_msg);
    }

    data_bag_ptr->close();

    ros::shutdown();

}
