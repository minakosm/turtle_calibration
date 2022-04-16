
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "message_filters/subscriber.h"
#include "message_filters/time_synchronizer.h"
#include "message_filters/sync_policies/approximate_time.h"
#include "yaml-cpp/yaml.h"
#include "eigen3/Eigen/Eigen"


std::pair<Eigen::Matrix<float, 3, 3>, Eigen::Matrix<float, 1, 5>> readIntrinsicParams(int cameraIndex);
void transform_pointcloud(const sensor_msgs::msg::PointCloud2 pclMsg);

