#include "projectPointcloud.h"
#include <memory>

#include "ament_index_cpp/get_package_share_directory.hpp" 

#include "utils.hpp"

#ifndef PI
#define PI 3.14159
#endif

int main(int argc, char* argv[]) {
    sensor_msgs::msg::PointCloud2 pclMsg = get_pcl_from_rosbag();
    std::cout <<"pcl point step: "<<pclMsg.point_step<<std::endl;
    // sensor_msgs::msg::PointCloud2::SharedPtr p(new sensor_msgs::msg::PointCloud2(pclMsg));
    //std::cout <<"p point step: "<<p->point_step<<std::endl;
    transform_pointcloud(pclMsg);
    return 0;
}
