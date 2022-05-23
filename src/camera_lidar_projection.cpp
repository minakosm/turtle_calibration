#include <memory>
#include <stdio.h>

// Yaml library for yaml file parsing
#include "yaml-cpp/yaml.h"

// Libraries for .ini file parsing
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/ini_parser.hpp>

// Get Package Share Directory library
#include "ament_index_cpp/get_package_share_directory.hpp" 

// Eigen Libraries
#include "eigen3/Eigen/Eigen"
#include "eigen3/Eigen/Core"
#include "eigen3/Eigen/Geometry"

// Libraries for pointcloud2 messages explotiation
#include "message_filters/subscriber.h"
#include "sensor_msgs/msg/point_cloud2.hpp"

// OpenCV Libraries
#include "opencv2/opencv.hpp"
#include "opencv2/core/eigen.hpp"

// Rosbag2_CPP Libraries
#include "rosbag2_cpp/reader.hpp"
#include "rosbag2_cpp/readers/sequential_reader.hpp"
#include "rosbag2_cpp/writer.hpp"
#include "rosbag2_cpp/writers/sequential_writer.hpp"
#include "rosbag2_storage/serialized_bag_message.hpp"

// Message Serialization Libraries
#include "rclcpp/serialization.hpp"
#include "rclcpp/serialized_message.hpp"

#ifndef PI
#define PI 3.14159
#endif

#define CAMERA_N 3  // Number of Cameras

Eigen::MatrixXf lidar_xyz;                  // LiDAR 3D points
Eigen::VectorXf intensities;                // LiDAR intensities 

Eigen::MatrixXf camera_3d_points;           // [x_c y_c z_c] 3d points described in camera world frame

Eigen::MatrixXf pixel_points;               // [u v w] homeogenous pixel points

Eigen::Matrix3f intrinsic_K;                // Camera Matrix 
Eigen::Array<float, 1, 5> intrinsic_D;      // Distortion Coefficients

int camera_index;

/**
 * @brief Get the pointcloud2 msg from rosbag object
 * 
 * @param filename The path for the rosbag file
 * @return sensor_msgs::msg::PointCloud2 
 */
sensor_msgs::msg::PointCloud2 get_pcl_from_rosbag(std::string filename){
    rosbag2_storage::StorageOptions storage_options;

    std::string rosbag_filename = ament_index_cpp::get_package_share_directory("turtle_calibration");
    rosbag_filename = rosbag_filename +"/rosbags/" + filename;
    storage_options.uri = rosbag_filename;

    // defaults
    storage_options.storage_id = "sqlite3";
    storage_options.max_bagfile_size = 0;  
    storage_options.max_cache_size = 0;    

    rosbag2_cpp::ConverterOptions converter_options;

    // defaults
    converter_options.input_serialization_format = "cdr";
    converter_options.output_serialization_format = "cdr";

    rosbag2_cpp::Reader reader(std::make_unique<rosbag2_cpp::readers::SequentialReader>());
    reader.open(storage_options, converter_options);

    std::shared_ptr<rosbag2_storage::SerializedBagMessage> bag_message;

    while(reader.has_next()){
        bag_message = reader.read_next();

        // nanoseconds unix epoch
        auto timestamp = bag_message->time_stamp; 

        if (bag_message->topic_name == "/ouster/rawPointcloud") {
            std::cout << "message" << std::endl;
            rclcpp::Serialization<sensor_msgs::msg::PointCloud2> serialization;
            sensor_msgs::msg::PointCloud2 pcl;
            rclcpp::SerializedMessage serialized_extracted_tf(*bag_message->serialized_data);
            serialization.deserialize_message(&serialized_extracted_tf, &pcl);
            std::cout << "pcl: " << pcl.data.size() << std::endl;

            return pcl;
        }

    }

}

/**
 * @brief Returns XYZ and LiDAR intensities from a given pcl_msg
 * 
 * @param pcl_msg Pointcloud2 Message
 */
std::pair<Eigen::MatrixXf, Eigen::VectorXf> get_lidar_XYZ_intensities(sensor_msgs::msg::PointCloud2 pcl_msg){

    uint8_t* ptr = pcl_msg.data.data();
    int pcl_size = pcl_msg.data.size()/pcl_msg.point_step;

    // resize 4xN (x(i) y(i) z(i) intensities(i))
    lidar_xyz.resize(4,pcl_size);
    intensities.resize(pcl_size);

    for(int i=0; i<pcl_size; i++){
        lidar_xyz(0,i) = *((float*)(ptr + i*pcl_msg.point_step)); // X
        lidar_xyz(1,i) = *((float*)(ptr + i*pcl_msg.point_step)); // Y
        lidar_xyz(2,i) = *((float*)(ptr + i*pcl_msg.point_step)); // Z
        lidar_xyz(3,i) = 1;                                       // Homogenous 1  

        intensities(i) = *((float*)(ptr + i*pcl_msg.point_step + 16));  // LiDAR intensitites
    }

    return std::make_pair(lidar_xyz, intensities);

}

/**
 * @brief Returns the camera matrix (K) and the distortion coefficients (D) of each camera, from a yaml file
 * 
 * @param camera_index Camera index : 0->left, 1->center, 2->right
 * @return std::pair<Eigen::Matrix3f, Eigen::Array<float, 1, 5>> 
 */
std::pair<Eigen::Matrix3f, Eigen::Array<float, 1, 5>> read_intrinsic_params(int camera_index){


    // TODO : INSTALL /settings folder
    // std::string intrinsic_filename = "/home/minakosm/turtle_ws/src/turtle_calibration/settings/intrinsic_params.yaml";
    std::string package_share_path = ament_index_cpp::get_package_share_directory("turtle_calibration");
    std::string intrinsic_filename = package_share_path + "/settings/intrinsic_params" + std::to_string(camera_index) + ".yaml";

    auto yaml_root = YAML::LoadFile(intrinsic_filename);

    for(YAML::const_iterator it = yaml_root.begin(); it!=yaml_root.end(); it++){
        
        const std::string &key = it->first.as<std::string>();

        YAML::Node attributes = it->second;
        int rows = attributes["rows"].as<int>();
        int cols = attributes["cols"].as<int>();

        const std::vector<float> data = attributes["data"].as<std::vector<float>>();

        for(int i=0; i<rows; i++){
            for(int j=0; j<cols; j++){
                if(key =="K"){
                    intrinsic_K(i,j) = data[i*cols + j];
                } else if(key == "D"){
                    intrinsic_D(j) = data[j];
                }else{
                    std::cout<<"Invalid Yaml Key"<<std::endl;
                    break;
                }
            }
        }

    }
    return std::make_pair(intrinsic_K, intrinsic_D);
}


/**
 * @brief Calculates the transformation matrix [R|t] by giving each coefficient (r,p,y) and (x,y,z) from an .ini file
 * 
 * @param filename Path to the .ini file 
 * @return The 4x4 [R|t] matrix
 */
Eigen::MatrixXf get_transformation_matrix(std::string filename){
    float roll, pitch, yaw;
    float t_x, t_y, t_z;

    Eigen::Matrix3f rotation_matrix;
    Eigen::Vector3f translation_vector;
    Eigen::MatrixXf transformation_matrix;

    boost::property_tree::ptree pt;
    boost::property_tree::ini_parser::read_ini(filename, pt);

    std::string label_roll = "Camera"+std::to_string(camera_index)+".roll";
    std::string label_pitch = "Camera"+std::to_string(camera_index)+".pitch";
    std::string label_yaw = "Camera"+std::to_string(camera_index)+".yaw";

    std::string label_t_x = "Camera"+std::to_string(camera_index)+".t_x";
    std::string label_t_y = "Camera"+std::to_string(camera_index)+".t_y";
    std::string label_t_z = "Camera"+std::to_string(camera_index)+".t_z";
    
    roll = pt.get<float>(label_roll);
    pitch = pt.get<float>(label_pitch);
    yaw = pt.get<float>(label_yaw);

    t_x = pt.get<float>(label_t_x);
    t_y = pt.get<float>(label_t_y);
    t_z = pt.get<float>(label_t_z);

    translation_vector << t_x, 
                          t_y, 
                          t_z;

    rotation_matrix = Eigen::AngleAxisf(roll, Eigen::Vector3f::UnitX())
                      *Eigen::AngleAxisf(pitch, Eigen::Vector3f::UnitY())
                      *Eigen::AngleAxisf(yaw, Eigen::Vector3f::UnitZ());

    transformation_matrix.resize(4,4);
    transformation_matrix << rotation_matrix, translation_vector,
                             Eigen::Matrix<float, 1, 3>::Zero(), 1;

    std::cout<<"Transformation matrix = "<<std::endl;
    std::cout<<transformation_matrix<<std::endl;

    return transformation_matrix;
}

/**
 * @brief This function calculates the 3d object points described in the camera frame 
 * 
 * @param lidar_xyz 3D object points described in LiDAR Frame
 * @return Eigen::MatrixXf 
 */

// TODO : install in /settings folder through CMAKE
Eigen::MatrixXf get_camera_3d_points(Eigen::MatrixXf lidar_xyz){
    Eigen::MatrixXf transformation_matrix;
    std::string filename = ament_index_cpp::get_package_share_directory("turtle_calibration");
    filename = filename + "/settings/lidar_camera_extrinsic.ini";

    transformation_matrix = get_transformation_matrix(filename);

    camera_3d_points.resize(4,lidar_xyz.cols());
    camera_3d_points = transformation_matrix * lidar_xyz;

    return camera_3d_points;
}

/**
 * @brief Calculate the cprrespondent pixel points (u,v) in the given camera frame from the 3D camera object points 
 * 
 * @param camera_3d_points 3D object points described in Camera Frame
 * @param intrinsic_K 3x3 Camera Matrix 
 * @return Eigen::MatrixXf 
 */
Eigen::MatrixXf get_pixel_points(Eigen::MatrixXf camera_3d_points, Eigen::Matrix3f intrinsic_K){
    Eigen::MatrixXf opencv_transformation_matrix;
    Eigen::Matrix3f opencv_rotation;

    Eigen::MatrixXf pixel_homeogenous_points;

    opencv_rotation << 0, 0, 1,
                      -1, 0, 0,
                       0, -1, 0;

    opencv_transformation_matrix.resize(3,4);
    opencv_transformation_matrix << opencv_rotation, Eigen::Vector3f::Zero();

    pixel_homeogenous_points.resize(3, camera_3d_points.cols());
    pixel_homeogenous_points = intrinsic_K *opencv_transformation_matrix * camera_3d_points;

    for(int i=0; i<pixel_homeogenous_points.cols(); i++){
        pixel_homeogenous_points(0,i) = pixel_homeogenous_points(0,i)/pixel_homeogenous_points(2,i);
        pixel_homeogenous_points(1,i) = pixel_homeogenous_points(0,i)/pixel_homeogenous_points(2,i);
        pixel_homeogenous_points(2,i) = 1;
    }

    return pixel_homeogenous_points;
}

/**
 * @brief Process the image and project the 3d LiDAR points on the image plane
 * 
 * @param image_filename Path to the image file that corresponds to the pointcloud2 msg 
 * @param pixel_points The calculated 2D pixel points that matches the 3D LiDAR points
 * 
 */
void image_processing(std::string image_filename,Eigen::MatrixXf pixel_points){;

    cv::Mat img;
    img = cv::imread(image_filename);
    
    cv::Rect2f boundaries(0,0,img.cols,img.rows);

    std::vector<cv::Point2f> px(pixel_points.cols());

    for(int i=0; i<px.size(); i++){
        px[i].x = pixel_points(0,i);
        px[i].y = pixel_points(1,i);

        if(px[i].inside(boundaries)){
            img.at<cv::viz::Color>(px[i].y, px[i].y) = cv::viz::Color::red();
        }
    }

    std::cout<<"Projecting LiDAR points to image"<<std::endl;
    cv::imshow("projected",img);
    
    cv::waitKey(0);
    std::cout<<"Saving image..."<<std::endl;
    cv::imwrite("3D projection",img);
    
}

int main(int argc, char* argv[]){

    if(argc < 2){
        std::cout<<"Please give the rosbag2 filename as an argument"<<std::endl<<"Terminating ..."<<std::endl;
        return 1;
    }
    sensor_msgs::msg::PointCloud2 pcl_msg = get_pcl_from_rosbag(argv[1]);

    auto lidar_points = get_lidar_XYZ_intensities(pcl_msg);
    
    for(camera_index=0; camera_index<CAMERA_N; camera_index++){
    
        auto intrinsic_params = read_intrinsic_params(camera_index);
        auto camera_points = get_camera_3d_points(lidar_points.first);  

        auto pixel_points = get_pixel_points(camera_points, intrinsic_params.first);       

        // TODO : CREATE IMAGES FOLDER AND INSTALL IT THROUGH CMake
        std::string image_filename = ament_index_cpp::get_package_share_directory("turtle_calibration");
        image_filename = image_filename + "/images/image" + std::to_string(camera_index) + ".jpg";

        image_processing(image_filename, pixel_points);
    }
    return 0;
}
