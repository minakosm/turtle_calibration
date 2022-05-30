#include "utils.hpp"
#include <iostream>


int main(/*int argc, char* argv[]*/) {
    // get pcl2
    sensor_msgs::msg::PointCloud2 pclMsg = get_pcl_from_rosbag();
    // std::cout << "pcl point step: "<< pclMsg.point_step << std::endl;
    sensor_msgs::msg::PointCloud2::SharedPtr p(new sensor_msgs::msg::PointCloud2(pclMsg));
    std::cout << "p point step: " << p->point_step << std::endl;

    // plane fitting

    // plane inliers

    // inlier center

    // visualization

    // normal vector from plane + center

    // get and transform normal vector from camera

    // formulate matrix equation

    // minimize matrix

    return 0;
}