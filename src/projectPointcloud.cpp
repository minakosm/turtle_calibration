#include "projectPointcloud.h"

#include <string>

#include "rclcpp/rclcpp.hpp"
#include "yaml-cpp/yaml.h"
#include "eigen3/Eigen/Eigen"
#include "eigen3/Eigen/Core"
#include "message_filters/subscriber.h"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "opencv2/opencv.hpp"
#include "opencv2/core/eigen.hpp"

#ifndef PI
#define PI 3.14159
#endif


#define CAMERA_N 3  // Number of Cameras


float roll, pitch, yaw;

Eigen::Matrix<float, 3, 3> intrinsic_K;         // Intrinsic Camera Matrix 
Eigen::Matrix<float, 1, 5> intrinsic_D;         // Intrinsic Distortion Matrix

Eigen::Matrix4Xf lp;                            // 4x4 Homogenous Lidar Points
Eigen::Matrix3Xf rotMat;                        // 3x3 Rotation Matrix 
Eigen::MatrixXf intensities;

Eigen::Matrix<bool, Eigen::Dynamic, 1> logic_expression ;
// Eigen::VectorXf intensities;
Eigen::Matrix<float, 4, 4> camFrame;


std::pair<Eigen::Matrix<float, 3, 3>, Eigen::Matrix<float, 1, 5>> readIntrinsicParams(int cameraIndex) {
    Eigen::Matrix<float, 3, 3> intrinsic_K;  // 3x3 Camera Matrix
    Eigen::Matrix<float, 1, 5> intrinsic_D;  // 1x5 Distortion Matrix

    // std::string intrinsic_filepath = "/home/minakosm/turtle_ws/src/turtle_calibration/settings/camera_settings" + std::to_string(cameraIndex) + ".yaml";
    std::string intrinsic_filepath = "/home/minakosm/turtle_ws/src/turtle_calibration/settings/intrinsic_params.yaml";
    auto yaml_root = YAML::LoadFile(intrinsic_filepath);
    
    for(YAML::const_iterator it=yaml_root.begin(); it!=yaml_root.end(); it++) {
        const std::string &key = it->first.as<std::string>();

        YAML::Node attributes = it->second;
        int rows = attributes["rows"].as<int>();
        int cols = attributes["cols"].as<int>();
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
    return std::make_pair(intrinsic_K, intrinsic_D);
}

Eigen::Matrix3f calculateTransformMat(float yaw, float pitch, float roll, int cameraIndex){
    Eigen::Matrix3f R;

    switch (cameraIndex)
    {
    case 0:
        R = Eigen::AngleAxisf(-0.04244655,
                     Eigen::Vector3f::UnitX())
            *Eigen::AngleAxisf(0.30226558,
                   Eigen::Vector3f::UnitY())
            *Eigen::AngleAxisf(-0.04521109,
                    Eigen::Vector3f::UnitZ());
        break;
    
    case 1:
        R = Eigen::AngleAxisf(0,
                     Eigen::Vector3f::UnitZ())
            *Eigen::AngleAxisf(pitch,
                   Eigen::Vector3f::UnitY())
            *Eigen::AngleAxisf(roll,
                    Eigen::Vector3f::UnitX());
        break;
    case 2:
    R = Eigen::AngleAxisf(-yaw,
                     Eigen::Vector3f::UnitZ())
            *Eigen::AngleAxisf(pitch,
                   Eigen::Vector3f::UnitY())
            *Eigen::AngleAxisf(roll,
                    Eigen::Vector3f::UnitX());
        break;
    }

    return R;
}

void transform_pointcloud(sensor_msgs::msg::PointCloud2 pclMsg){
    uint8_t* ptr = pclMsg.data.data();
    yaw = 17*PI/180.0;
    pitch = 10*PI/180.0;
    roll = 0.2*PI/180.0;


    for(int i=0; i<CAMERA_N; i++){
        // std::string filename = "/home/minakosm/turtle_ws/src/turtle_calibration/images/image" + std::to_string(i) + ".jpg";
        std::string filename = "/home/minakosm/lidar_snapshot_test/" + std::to_string(i) + ".jpg";

        cv::Mat img = cv::imread(filename);
        if(img.empty()) {
            std::cout << "Error loading image" << i << std::endl; 
            return;
        }
        cv::imshow("camera_image", img);
        cv::waitKey(0);
    
        auto params = readIntrinsicParams(i);
        intrinsic_K = params.first;
        intrinsic_D = params.second;

        rotMat = calculateTransformMat(yaw, pitch, roll, i);


        lp.resize(4, pclMsg.data.size()/pclMsg.point_step);
        intensities.resize(pclMsg.data.size()/pclMsg.point_step,1);
        std::vector<cv::Point3f> obj_points;

        for(int j=0; j < pclMsg.data.size()/pclMsg.point_step; j++){

            lp(0,j) = *((float*)(ptr + j*pclMsg.point_step));       //X
            lp(1,j) = *((float*)(ptr + j*pclMsg.point_step + 4));   //Y
            lp(2,j) = *((float*)(ptr + j*pclMsg.point_step + 8));   //Z
            //lp(3,j) = intensity(j)
            lp(3,j) = 1;

            intensities(j,0) = *((ptr + j*pclMsg.point_step + 16)); 

            obj_points.push_back(cv::Point3f(lp(0,j), lp(1,j), lp(2,j))); 

        }

        Eigen::Matrix<float, 3, 3> rot;
        rot<<0, 0, 1,
             -1, 0, 0,
             0, -1, 0;

        Eigen::Matrix<float, 3, 4> intrinsic;
        intrinsic << intrinsic_K, Eigen::Vector3f::Zero();

        Eigen::MatrixXf rotationEigen;
        cv::Mat rotationCV;
        cv::Vec3f rvec;
        cv::Vec3f tvec(0, 0, 0);
        cv::Mat camMat;
        cv::Mat distMat;
        std::vector<cv::Point2f> imgPoints(obj_points.size());
        
        rotationEigen.resize(3,3);
        rotationEigen =  rot * rotMat;

        cv::eigen2cv(rotationEigen ,rotationCV);
        
        cv::Rodrigues(rotationCV, rvec);

        std::cout<<"rvec = "<<rvec<<std::endl;
        std::cout<<"tvec = " <<tvec<<std::endl;
        cv::eigen2cv(intrinsic_K, camMat);
        cv::eigen2cv(intrinsic_D, distMat);

        cv::projectPoints(obj_points, rvec, tvec, camMat, distMat, imgPoints);
        cv::Rect2f boundaries(0,0,img.cols,img.rows);


        //Color 

        cv::Vec3f min_color = cv::Vec3f(0,0,255);
        cv::Vec3f max_color = cv::Vec3f(255, 0, 0);
        cv::Vec3f final_c;

        float norm_intensity;

        float min_intensity = intensities.minCoeff();
        float max_intensity = intensities.maxCoeff();

        std::cout<<"min_intensity = "<<min_intensity<<std::endl;
        std::cout<<"max_intensity = "<<max_intensity<<std::endl;
        std::cout <<"imgPoints.size() = "<<imgPoints.size()<<std::endl;
        int counter = 0;        

        for(int k=0; k<imgPoints.size(); k++){
                if(imgPoints[k].inside(boundaries)){
                    counter++;
                    norm_intensity = (intensities(k,0) - min_intensity) / (max_intensity - min_intensity);
                    final_c = (norm_intensity * min_color) + (1-norm_intensity)*max_color;

                    img.at<cv::Vec3b>(imgPoints[k]) = final_c;

                    for(int m=1; m<4; m++){
                        for(int n=1; n<4; n++){
                        if(imgPoints[k].x+m+n<img.cols && imgPoints[k].y+m+n<img.rows){
                            img.at<cv::Vec3b>(imgPoints[k].y, imgPoints[k].x+n) = final_c;
                            img.at<cv::Vec3b>(imgPoints[k].y+n, imgPoints[k].x) = final_c;
                            img.at<cv::Vec3b>(imgPoints[k].y+n, imgPoints[k].x+m) = final_c;
                            img.at<cv::Vec3b>(imgPoints[k].y+m, imgPoints[k].x+n) = final_c;
                        }
                        }
                    }

                    
                }
        }

        std::cout << "counter = "<<counter<<std::endl;
        obj_points.clear();
        imgPoints.clear();
        
        std::string imgSave = "test" + std::to_string(i) + ".jpg";
        cv::imshow("projected", img);
        cv::imwrite(imgSave, img);

        cv::waitKey(0);

    }

}