#include "projectPointcloud.h"

#include <string>

#include "rclcpp/rclcpp.hpp"
#include "yaml-cpp/yaml.h"
#include "eigen3/Eigen/Eigen"
#include "message_filters/subscriber.h"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "opencv2/opencv.hpp"
#include "opencv2/core/eigen.hpp"

#define CALIBRATION_FILENAME "extrinsic_calibration.yaml"
#define OFFLINE 1   // Boolean Offline Mode
#define TSIGGAN 1   // Boolean Tsigganies

#ifndef CAMERA_N
#define CAMERA_N 2  // Number of Cameras
#endif

// constructor
projectPointcloud::projectPointcloud(int offline) : rclcpp::Node("LidarImage"){
    rclcpp::QoS qos(10);
    qos.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
    offlineSub = this->create_subscription<sensor_msgs::msg::PointCloud2>("ouster/rawPointcloud", qos, std::bind(&projectPointcloud::offlineCallback, this, std::placeholders::_1));
    std::cout<<"InsideConstructor"<<std::endl;
    if(offline){
        std::cout<<"InsideConstructor offline1"<<std::endl;
        initOfflineProjection();
        RCLCPP_INFO(this->get_logger(), "Offline Subscriber initiated");
    }

    initPubs();
    initSubs();

    RCLCPP_INFO(this->get_logger(), "Subscribers and Publishers initiated");

};

void projectPointcloud::initSubs() {

    rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data; 

    this->pclSubscriber = new message_filters::Subscriber<sensor_msgs::msg::PointCloud2>(this, "/lidar/groundFiltered", qos_profile);
    this->imgSubscriber = new message_filters::Subscriber<sensor_msgs::msg::Image>(this, "/camera/imageRaw", qos_profile);

    this->sync = new message_filters::Synchronizer<approximatePolicy>(approximatePolicy(10), *pclSubscriber, *imgSubscriber);
    this->sync->registerCallback(&projectPointcloud::syncCallback, this);
}

void projectPointcloud::initPubs() {

    rclcpp::SensorDataQoS qos;

    this->projectionPublisher = this->create_publisher<sensor_msgs::msg::Image>("/calibration/projectedImage", qos);

}

void projectPointcloud::readIntrinsicParams(int cameraIndex) {
    std::string intrinsic_filepath = "/settings/camera_settings" + std::to_string(cameraIndex);
    yaml_root = YAML::LoadFile(intrinsic_filepath);
    
    for(YAML::const_iterator it=yaml_root.begin(); it!=yaml_root.end(); it++) {
        const std::string &key = it->first.as<std::string>();

        YAML::Node attributes = it->second;
        int rows = attributes["rows"].as<int>();
        int cols = attributes["cols"].as<int>();

        const std::vector<double> data = attributes["data"].as<std::vector<double>>();

        for(int i=0; i<rows; i++){
            for(int j=0; j<cols; j++){
                if(key == "K"){
                    intrinsic_K(i,j) = data[i*cols + j];
                } else if(key == "D"){
                    intrinsic_D(j) = data[j];
                } else {
                    std::cout<<"Invalid yaml attribute"<<std::endl;
                    break;
                }
            }
        }
        
    }
}

Eigen::MatrixXd projectPointcloud::calculateProjMat() {
    yaml_root = YAML::LoadFile(CALIBRATION_FILENAME);

    std::vector<double> tempVector = yaml_root["Transformation_matrix"].as<std::vector<double>>();
    R_t = Eigen::Map<Eigen::Matrix<double, 3, 4>>(tempVector.data());

    // Eigen::Map<Eigen::MatrixXf> R_t(tempMat.data());
    // R_t = yaml_root["Transformation_matrix"].as<Eigen::MatrixXd>();
    
    Eigen::MatrixXd proj_mat = intrinsic_K * R_t;

    return proj_mat;
}

void projectPointcloud::syncCallback(sensor_msgs::msg::PointCloud2::SharedPtr pclMsg, sensor_msgs::msg::Image::SharedPtr imgMsg) {
/**
 * TODO : 1) Take data from subscribers 
 *        2) Take CameraMatrix and [R|t]
 *        3) Transform points
 *        4) projectPoints - Publish image
 */
    auto a1 = std::chrono::steady_clock::now();
    RCLCPP_INFO(this->get_logger(), "\nPointCloud2 size = %u", pclMsg->width * pclMsg->height);

    for(int i=0; i<CAMERA_N; i++){
        readIntrinsicParams(i);    
        proj_mat = calculateProjMat();
    }

    auto a2 = std::chrono::steady_clock::now();
    RCLCPP_INFO(this->get_logger(), "Total syncCallback time in us: %lu", std::chrono::duration_cast<std::chrono::microseconds>(a2 - a1).count());
}

void projectPointcloud::initOfflineProjection() {

    pointcloudMsg.header.frame_id = "os1";
    pointcloudMsg.is_bigendian = false;
    pointcloudMsg.is_dense = true;
    pointcloudMsg.point_step = 12;
    pointcloudMsg.fields.resize(3);

    pointcloudMsg.fields[0].name = "x";
    pointcloudMsg.fields[0].offset = 0;
    pointcloudMsg.fields[0].datatype = 7;
    pointcloudMsg.fields[0].count = 1;

    pointcloudMsg.fields[1].name = "y";
    pointcloudMsg.fields[1].offset = 4;
    pointcloudMsg.fields[1].datatype = 7;
    pointcloudMsg.fields[1].count = 1;

    pointcloudMsg.fields[2].name = "z";
    pointcloudMsg.fields[2].offset = 8;
    pointcloudMsg.fields[2].datatype = 7;
    pointcloudMsg.fields[2].count = 1;

    pointcloudMsg.fields[3].name = "intensity";
    pointcloudMsg.fields[3].offset = 12;
    pointcloudMsg.fields[3].datatype = 4;
    pointcloudMsg.fields[3].count = 1;

    cameraIndex = 0;
}

void projectPointcloud::offlineCallback(const sensor_msgs::msg::PointCloud2::SharedPtr pclMsg){
    std::cout<<"InsideCallback"<<std::endl;
    uint8_t* ptr = pclMsg->data.data();
    Eigen::Array<bool, Eigen::Dynamic, 1> logic_expression;

    for(int i=0; i<CAMERA_N; i++){
        std::string filename = "../images/image" + std::to_string(i) + ".jpg";

        cv::Mat img = cv::imread(filename);
        if(img.empty()) {std::cout << "Error loading image" << i << std::endl; return;}
        cv::Mat img_proj = cv::Mat::zeros(img.size(), img.type());

        readIntrinsicParams(i);
        cameraIndex++;
        // proj_mat = calculateProjMat();
        
        if(TSIGGAN) {
        R_t << 1, 0 ,0, 0,
               0, 1, 0, 0,
               0, 0, 1, 0;

        proj_mat = intrinsic_K * R_t;
        }

        for(int i=0; i=pclMsg->data.size(); i++){
            lp(0,i) = *((float*)(ptr + i*pclMsg->point_step));
            lp(1,i) = *((float*)(ptr + i*pclMsg->point_step + 4));
            lp(2,i) = *((float*)(ptr + i*pclMsg->point_step + 8));  
            lp(3,i) = 1;

            px = proj_mat * lp;

            logic_expression(i) = px(0,i) < img.cols &&
                                    px(0,i) > 0 &&
                                    px(1,i) < img.rows &&
                                    px(1,i) > 0;
        }

        for(unsigned int i=0; i<logic_expression.size(); i++){
            if(logic_expression(i)){
                for(int c=0; c<img.channels(); c++){
                    img_proj.at<cv::Vec3b>(px(1,i),px(0,i))[c] = 255/(int)lp(2,i);
                }
            }
        }

        cv::imshow("camera_image", img);
        cv::imshow("lidar_image", img_proj);
        cv::waitKey(0);

    }

}