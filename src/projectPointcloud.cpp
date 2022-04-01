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


#define PI 3.14159
#define CALIBRATION_FILENAME "extrinsic_calibration.yaml"
#define OFFLINE 1   // Boolean Offline Mode
#define TSIGGAN 1   // Boolean Tsigganies


#define CAMERA_N 2  // Number of Cameras


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

    std::pair<Eigen::Matrix<float, 3, 3>, Eigen::Matrix<float, 1, 5>> readIntrinsicParams(int cameraIndex) {
    Eigen::Matrix<float, 3, 3> intrinsic_K;  // 3x3 Camera Matrix
    Eigen::Matrix<float, 1, 5> intrinsic_D;  // 1x5 Distortion Matrix

    std::string intrinsic_filepath = "/home/minakosm/turtle_ws/src/turtle_calibration/settings/camera_settings" + std::to_string(cameraIndex) + ".yaml";
    auto yaml_root = YAML::LoadFile(intrinsic_filepath);
    
    for(YAML::const_iterator it=yaml_root.begin(); it!=yaml_root.end(); it++) {
        const std::string &key = it->first.as<std::string>();

        YAML::Node attributes = it->second;
        int rows = attributes["rows"].as<int>();
        int cols = attributes["cols"].as<int>();

        std::cout << "rows" << rows << std::endl;
        std::cout << "cols" << cols << std::endl;

        const std::vector<float> data = attributes["data"].as<std::vector<float>>();

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
    std::cout<<"Returning to transform_pointcloud"<<std::endl;
    return std::make_pair(intrinsic_K, intrinsic_D);
}

Eigen::MatrixXf projectPointcloud::calculateProjMat() {
    yaml_root = YAML::LoadFile(CALIBRATION_FILENAME);

    std::vector<float> tempVector = yaml_root["Transformation_matrix"].as<std::vector<float>>();
    R_t = Eigen::Map<Eigen::Matrix<float, 3, 4>>(tempVector.data());

    // Eigen::Map<Eigen::MatrixXf> R_t(tempMat.data());
    // R_t = yaml_root["Transformation_matrix"].as<Eigen::MatrixXf>();
    
    Eigen::MatrixXf proj_mat = intrinsic_K * R_t;

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
        auto params = readIntrinsicParams(i);    
        intrinsic_K = params.first;
        intrinsic_D = params.second;
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
}

void transform_pointcloud(sensor_msgs::msg::PointCloud2 pclMsg){
    std::cout<<"InsideCallback"<<std::endl;
    uint8_t* ptr = pclMsg.data.data();


    for(int i=0; i<CAMERA_N; i++){
        std::string filename = "/home/minakosm/turtle_ws/src/turtle_calibration/images/image" + std::to_string(i) + ".jpg";

        cv::Mat img = cv::imread(filename);
        if(img.empty()) {
            std::cout << "Error loading image" << i << std::endl; 
            return;
        }
        cv::imshow("camera_image", img);
        cv::waitKey(0);
        cv::Mat img_proj = cv::Mat::zeros(img.size(), img.type());

        img_proj = img;

        auto params = readIntrinsicParams(i);
        Eigen::Matrix<float, 3, 3> intrinsic_K = params.first;
        Eigen::Matrix<float, 1, 5> intrinsic_D = params.second;

        // cameraIndex++;
        // proj_mat = calculateProjMat();

        Eigen::Matrix3Xf px;  // Homogenous pixel points
        Eigen::Matrix<float, 3, 4> R_t;              // 3x4 Transformation Matrix [R|t]
        Eigen::Matrix<float, 3, 4> proj_mat;         // 3x4 Projection Matrix
        Eigen::Matrix4Xf lp;  // Homogenous lidar 3D-points
        Eigen::Matrix<bool, Eigen::Dynamic, 1> logic_expression ;

        if(TSIGGAN) {
        R_t << 0, -1 ,0, 10,
               0, 0, -1, 10,
               -1, 0, 1, 10;

        proj_mat = intrinsic_K * R_t;
        }

        std::cout <<"R_t =  "<< R_t <<std::endl;
        std::cout <<"proj_mat = " << proj_mat << std::endl;

        lp.resize(4, pclMsg.data.size()/pclMsg.point_step);

        for(int j=0; j < pclMsg.data.size()/pclMsg.point_step; j++){
            

            lp(2,j) = /*((float*)(ptr + j*pclMsg.point_step + 8))*/ 1 ;
            lp(0,j) = *((float*)(ptr + j*pclMsg.point_step)) / lp(2,j);
            lp(1,j) = *((float*)(ptr + j*pclMsg.point_step + 4)) / lp(2,j) ;  
            lp(3,j) = 1/ lp(2,j);

            std::cout<<" x"<<j<<" = "<<lp(0,j);
            std::cout<<" y"<<j<<" = "<<lp(1,j);
            std::cout<<" z"<<j<<" = "<<lp(2,j)<<std::endl;

            // px = proj_mat * tmp;
        }

        px.resize(proj_mat.rows(), lp.cols());
        px = proj_mat * lp;

        std::cout<< "px.cols() = "<< px.cols() << std::endl;
        logic_expression.resize(px.cols());
        for(int l=0; l<px.cols(); l++){
            
            px(0,l) = px(0,l)/100;
            px(1,l) = px(1,l)/100;
            logic_expression(l) = px(0,l) < img.cols &&
                                  px(0,l) > 0 &&
                                  px(1,l) < img.rows &&
                                  px(1,l) > 0;
        }

        for(int k=0; k<logic_expression.size(); k++){
            if(logic_expression(k)){
                std::cout <<"px(1,"<<k<<") = " <<px(1,k) << " px(0,"<<k<<") = " << px(0,k) << std::endl;
                img_proj.at<cv::Vec3b>(int(px(1,k)),int(px(0,k))) = cv::viz::Color::red();
                std::cout<<img_proj.at<cv::Vec3b>(px(1,k),px(0,k))<<std::endl;
            }
        }
        
        std::cout<< "image_rows = "<<img.rows<<std::endl;
        std::cout<< "image_cols = "<<img.cols<<std::endl;

        cv::imshow("lidar_image", img_proj);
        cv::waitKey(0);

    }

}