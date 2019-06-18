#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <fstream>
#include <opencv2/highgui/highgui.hpp>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/io/pcd_io.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>

#include "okvis/file-system-tools.h"
using namespace sensor_msgs;
using namespace message_filters;

std::string save_folder = "/home/pang/data/dataset/segway_outdoor/2019-06-05-16-45-28_forMap";
auto image0_syn_folder = save_folder+ "/image0_syn";
auto image1_syn_folder = save_folder+ "/image1_syn";
auto lidar_syn_folder = save_folder+ "/velodyne_syn";
auto sensor_syn_file = save_folder+ "/sensor_syn.txt";

std::ofstream sensor_syn_ofs;
int sensor_cnt = 0;

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

void callback(const ImageConstPtr& image0, const ImageConstPtr& image1, const PointCloud2ConstPtr& lidar)
{
    // Solve all of perception here...
    int64_t  dt0 = (int64_t)(image0->header.stamp.toNSec() - image1->header.stamp.toNSec());
    int64_t  dt1 = (int64_t)(image0->header.stamp.toNSec() - lidar->header.stamp.toNSec());

//    std::cout << dt0 << " " << dt1 << std::endl;

    // in 5 ms, save it
    if (abs(dt0) < 5 * 1e6 && abs(dt1) < 30 * 1e6) {

        cv_bridge::CvImagePtr left_cv_ptr;
        left_cv_ptr = cv_bridge::toCvCopy(image0, sensor_msgs::image_encodings::BGR8);
        cv::Mat left_image_cv = left_cv_ptr->image;

        cv_bridge::CvImagePtr right_cv_ptr;
        right_cv_ptr = cv_bridge::toCvCopy(image1, sensor_msgs::image_encodings::BGR8);
        cv::Mat right_image_cv = right_cv_ptr->image;


        std::string image_name = std::string(6 - std::to_string(sensor_cnt).length(), '0') + std::to_string(sensor_cnt) + ".png";
        std::stringstream ss0, ss1;
        ss0 <<image0_syn_folder << "/" << image_name;
        cv::imwrite(ss0.str(), left_image_cv);

        ss1 <<image1_syn_folder << "/" << image_name;
        cv::imwrite(ss1.str(), right_image_cv);

//        pcl::PointCloud<pcl::PointXYZI>::Ptr lidar(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::PCLPointCloud2 pcl_pc2;
        pcl_conversions::toPCL(*lidar.get(),pcl_pc2);
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

        std::string lidar_name = std::string(6 - std::to_string(sensor_cnt).length(), '0') + std::to_string(sensor_cnt) + ".bin";
        std::stringstream ss_lidar;
        ss_lidar <<lidar_syn_folder << "/" << lidar_name;

        writePointCloudToBinFile(*out_cloud, ss_lidar.str());


        sensor_syn_ofs  << "image0_syn/"<< image_name << ","
                        << std::to_string(image0->header.stamp.toNSec()) << ","
                << "image1_syn/"<< image_name << ","
                << std::to_string(image1->header.stamp.toNSec()) << ","
                << "velodyne_syn/"<< lidar_name << ","
                << std::to_string(lidar->header.stamp.toNSec()) << ","
                << dt0 << "," << dt1
                <<  std::endl;

        std::cout << image_name << " "
                << image0->header.stamp.toNSec() << " "
                << image1->header.stamp.toNSec() << " "
                << lidar->header.stamp.toNSec() << std::endl;

        sensor_cnt ++;
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "vision_node");

    ros::NodeHandle nh;
    message_filters::Subscriber<Image> image0_sub(nh, "/stereo/cameraL", 10);
    message_filters::Subscriber<Image> image1_sub(nh, "/stereo/cameraR", 10);
    message_filters::Subscriber<PointCloud2> lidar_sub(nh, "/velodyne_points", 10);

    if (!common::pathExists(image0_syn_folder)) {
        common::createPath(image0_syn_folder);
    }
    if (!common::pathExists(image1_syn_folder)) {
        common::createPath(image1_syn_folder);
    }

    if (!common::pathExists(lidar_syn_folder)) {
        common::createPath(lidar_syn_folder);
    }

    sensor_syn_ofs.open(sensor_syn_file);

    typedef sync_policies::ApproximateTime<Image, Image, PointCloud2> MySyncPolicy;
    // ApproximateTime takes a queue size as its constructor argument, hence MySyncPolicy(10)
    Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), image0_sub, image1_sub, lidar_sub);
    sync.registerCallback(boost::bind(&callback, _1, _2, _3));

    ros::spin();

    return 0;
}