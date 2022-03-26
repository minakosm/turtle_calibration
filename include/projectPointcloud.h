
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "message_filters/subscriber.h"
#include "message_filters/time_synchronizer.h"
#include "message_filters/sync_policies/approximate_time.h"
#include "yaml-cpp/yaml.h"
#include "eigen3/Eigen/Eigen"


/**
 * Code to implement in order to project lidar 3D pointclouds
 * on 2D images. 
 * 
 * 1) subscribers
 * 2) Callback
 *      a) load intrinsic Params --> readIntrinsicParams()
 *      b) calculate R,t
 *      c) project points
 * 4) ???
 * 5) profit!
 * 
 **/

class projectPointcloud : public rclcpp::Node
{
private:
    message_filters::Subscriber<sensor_msgs::msg::PointCloud2> *pclSubscriber; // Pointcloud Subscriber
    message_filters::Subscriber<sensor_msgs::msg::Image> *imgSubscriber; // Image Subscriber

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr projectionPublisher;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr offlineSub;

    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::PointCloud2, sensor_msgs::msg::Image> approximatePolicy;
    message_filters::Synchronizer<approximatePolicy> *sync;

    sensor_msgs::msg::PointCloud2 pointcloudMsg;

    YAML::Node yaml_root;
    Eigen::MatrixXd intrinsic_K;    // 3x3 Camera Matrix    
    Eigen::VectorXd intrinsic_D;    // 1x5 Distortion Matrix

    int cameraIndex;

    Eigen::Matrix<double, 3, Eigen::Dynamic> px;    // Homogenous pixel points
    Eigen::Matrix<double, 3, 4> R_t;    // 3x4 Transformation Matrix [R|t]
    Eigen::Matrix<double, 3, 4> proj_mat;   // 3x4 Projection Matrix
    Eigen::Matrix<double, 4, Eigen::Dynamic> lp;    // Homogenous lidar 3D-points

    void syncCallback(sensor_msgs::msg::PointCloud2::SharedPtr pclMsg, sensor_msgs::msg::Image::SharedPtr imgMsg); 
    void offlineCallback(sensor_msgs::msg::PointCloud2::SharedPtr pclMsg);

public:

    projectPointcloud(int offline);
    virtual ~projectPointcloud(){}

    void initSubs();
    void initPubs();
    void readIntrinsicParams(int cameraIndex);
    Eigen::MatrixXd calculateProjMat();

    void initOfflineProjection();
};


