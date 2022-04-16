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

#ifndef PI
#define PI 3.14159
#endif

#define CALIBRATION_FILENAME "extrinsic_calibration.yaml"
#define OFFLINE 1   // Boolean Offline Mode
#define TSIGGAN 0   // Boolean Tsigganies


#define CAMERA_N 2  // Number of Cameras


float roll, pitch, yaw;

Eigen::Matrix<float, 3, 3> intrinsic_K;         // Intrinsic Camera Matrix 
Eigen::Matrix<float, 1, 5> intrinsic_D;         // Intrinsic Distortion Matrix

Eigen::Matrix4Xf lp;                            // 4xN Homogenous lidar 3D-points
Eigen::Matrix4Xf cp;                            // Homogenous camera points
Eigen::Matrix4f T_mat;                          // 4x4 Transformation Matrix 
Eigen::Matrix<float, 3, 4> proj_mat;            // 3x4 Projection Matrix
Eigen::Matrix<float, 3, Eigen::Dynamic> px;     // 3xN Pixel Points

Eigen::Matrix<bool, Eigen::Dynamic, 1> logic_expression ;
// Eigen::VectorXf intensities;
Eigen::Matrix<float, 4, 4> camFrame;


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

Eigen::Matrix4f calculateTransformMat(float yaw, float pitch, float roll, int cameraIndex){
    Eigen::Vector3f t;
    Eigen::Matrix3f R;
    Eigen::Matrix4f R_t;
    Eigen::Matrix3f rot;

        // rot<<0, 0, -1,
        //      1, 0, 0,
        //      0, 1, 0;


    R = Eigen::AngleAxisf(std::pow(-1, cameraIndex)*yaw,
                    Eigen::Vector3f::UnitZ())
        *Eigen::AngleAxisf(pitch,
                   Eigen::Vector3f::UnitY())
        *Eigen::AngleAxisf(roll,
                    Eigen::Vector3f::UnitX());
    
    // R = R*rot;
    std::cout<<"Rotation Matrix R = "<<R<<std::endl;

    t << -0.031, 0, -0.25;

    R_t <<R,t,Eigen::MatrixXf::Zero(1,R_t.cols()-1),1;
    std::cout<<"TRANSFORMATION MATRIX \n =" << R_t << std::endl;

    return R_t;
    
}

void transform_pointcloud(sensor_msgs::msg::PointCloud2 pclMsg){
    uint8_t* ptr = pclMsg.data.data();
    yaw = 16*PI/180.0;
    pitch = 14*PI/180.0;
    roll = 0.0;


    for(int i=0; i<CAMERA_N; i++){
        std::string filename = "/home/minakosm/turtle_ws/src/turtle_calibration/images/image" + std::to_string(i) + ".jpg";

        cv::Mat img = cv::imread(filename);
        if(img.empty()) {
            std::cout << "Error loading image" << i << std::endl; 
            return;
        }
        cv::imshow("camera_image", img);
        cv::waitKey(0);
        cv::Mat img_cpy;
        cv::Mat img_proj_resized;
        int img_width_resized = 2560;
        int img_height_resized = 2048;

        img_cpy = img;
        cv::resize(img, img_proj_resized, cv::Size(img_width_resized, img_height_resized), cv::INTER_LINEAR);

        auto params = readIntrinsicParams(i);
        intrinsic_K = params.first;
        intrinsic_D = params.second;

        std::cout<<"Camera Matrix = "<<intrinsic_K<<std::endl;
        T_mat = calculateTransformMat(yaw, pitch, roll, i);

        std::cout <<"T_mat =  "<< T_mat <<std::endl;
        // std::cout <<"proj_mat = " << proj_mat << std::endl;

        lp.resize(4, pclMsg.data.size()/pclMsg.point_step);
        // intensities.resize(pclMsg.data.size()/pclMsg.point_step);

        for(int j=0; j < pclMsg.data.size()/pclMsg.point_step; j++){

            lp(0,j) = *((float*)(ptr + j*pclMsg.point_step));
            lp(1,j) = *((float*)(ptr + j*pclMsg.point_step + 4));
            lp(2,j) = *((float*)(ptr + j*pclMsg.point_step + 8)); 
            lp(3,j) = 1;

            // intensities(j) = *((float*)(ptr + j*pclMsg.point_step + 12)); 

            // std::cout<<" x"<<j<<" = "<<lp(0,j);
            // std::cout<<" y"<<j<<" = "<<lp(1,j);
            // std::cout<<" z"<<j<<" = "<<lp(2,j)<<std::endl;

            // px = proj_mat * tmp;
        }

        cp.resize(T_mat.rows(), lp.cols());

        // cp = T_mat * lp;
        std::cout<< "DEBUG PX"<<std::endl;

        // for(int i=0; i< cp.cols(); i++){
            // std::cout<<"cp(0, "<<i<<") = "<<cp(0,i)<<std::endl;
            // std::cout<<"cp(1, "<<i<<") = "<<cp(1,i)<<std::endl;
            // std::cout<<"cp(2, "<<i<<") = "<<cp(2,i)<<std::endl;
            // std::cout<<"cp(3, "<<i<<") = "<<cp(3,i)<<std::endl;

        // }
        
        px.resize(3, cp.cols());
        std::cout<<"PROJ MAT DEBUG 1" <<std::endl;
        Eigen::Matrix<float, 4, 4> rot;
        rot<<0, -1, 0, 0,
             0, 0, -1, 0,
             1, 0, 0, 0,
             0, 0, 0, 1;

        Eigen::Matrix<float, 3, 4> intrinsic;
        intrinsic << intrinsic_K, Eigen::Vector3f::Zero();
        cp = rot*T_mat.inverse()*lp ;
        std::cout<<"PROJ MAT DEBUG 2" <<std::endl;

        px = intrinsic * cp ;
        std::cout <<"Intrinsic = "<< intrinsic <<std::endl;
        for(int i=0; i<px.cols(); i++){
            px(0,i) = px(0,i)/px(2,i) ;
            px(1,i) = px(1,i)/px(2,i) ;

            // std::cout<<"px(0," <<i<<") =" <<px(0,i)<<std::endl;
            // std::cout<<"px(1," <<i<<") =" <<px(1,i)<<std::endl;
            
        }
        std::cout<<"PROJ MAT DEBUG 3 "<<px.cols() <<std::endl;
        
        logic_expression.resize(px.cols());
        for(int l=0; l<px.cols(); l++){
            
            logic_expression(l) = px(0,l) < img_proj_resized.cols &&
                                  px(0,l) > 0 &&
                                  px(1,l) < img_proj_resized.rows &&
                                  px(1,l) > 0;
        }
        std::cout<<logic_expression.count() <<std::endl;

        std::vector<cv::Point3f> obj_points;
        // std::cout <<"logic_expression = "<<logic_expression<<std::endl;
        for(int k=0; k<logic_expression.size(); k++){
            if(logic_expression(k)){
                obj_points.push_back(cv::Point3f(lp(0,k), lp(1,k), lp(2,k)));
                // std::cout <<"px(1,"<<k<<") = " <<proj_p(1,k) << " proj_p(0,"<<k<<") = " << proj_p(0,k) << std::endl;
                img_proj_resized.at<cv::Vec3b>(int(px(1,k)),int(px(0,k))) = cv::viz::Color::red();
                img_proj_resized.at<cv::Vec3b>(int(px(1,k)+1),int(px(0,k))) = cv::viz::Color::red();
                img_proj_resized.at<cv::Vec3b>(int(px(1,k)),int(px(0,k)+1)) = cv::viz::Color::red();
                img_proj_resized.at<cv::Vec3b>(int(px(1,k)+1),int(px(0,k)+1)) = cv::viz::Color::red();

                
            }
        }
        
        Eigen::MatrixXf test;
        cv::Mat testCv;
        cv::Vec3f rvec;
        cv::Vec3f tvec = (-0.031, 0, -0.25);
        cv::Mat camMat;
        cv::Mat distMat;
        cv::Mat imgPoints;
        test = (rot * T_mat.inverse());
        test.conservativeResize(3,3);

        cv::eigen2cv(test ,testCv);
        
        cv::Rodrigues(testCv, rvec);

        cv::eigen2cv(intrinsic_K, camMat);
        cv::eigen2cv(intrinsic_D, distMat);


        cv::projectPoints(obj_points, rvec, tvec, camMat, distMat, imgPoints);
        
        // for(int k=0; k<imgPoints.size(); k++){
        //         img_proj_resized.at<cv::Point2f>(imgPoints(k)) = cv::viz::Color::red();
        //         img_proj_resized.at<cv::Point2f>(int(px(1,k)+1),int(px(0,k))) = cv::viz::Color::red();
        //         img_proj_resized.at<cv::Point2f>(int(px(1,k)),int(px(0,k)+1)) = cv::viz::Color::red();
        //         img_proj_resized.at<cv::Point2f>(int(px(1,k)+1),int(px(0,k)+1)) = cv::viz::Color::red();
        // }

        std::cout<< "image_rows / img_proj_resized_rows = "<<img_cpy.rows/img_proj_resized.rows<<std::endl;
        std::cout<< "image_cols / img_proj_resized_cols = "<<img_cpy.cols/img_proj_resized.cols<<std::endl;

        cv::Mat img_downscaled;
        cv::resize(img_proj_resized, img_downscaled, cv::Size(img.cols, img.rows), cv::INTER_LINEAR);
        cv::imshow("downscaled",img_downscaled);
        cv::imwrite("test.jpg", img_proj_resized);

        cv::waitKey(0);

    }

}