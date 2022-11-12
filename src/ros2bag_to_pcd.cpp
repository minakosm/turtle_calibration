#include "utils.hpp"
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/common/io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

int main(/*int argc, char* argv[]*/) {
    // get pcl2
    sensor_msgs::msg::PointCloud2 pclMsg = get_pcl_from_rosbag();

    // save PCD
    pcl::io::savePCDFile("lidar_right.pcd", pclMsg); 
    return 0;
}