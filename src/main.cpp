// #include "projectPointcloud.cpp"
#include "projectPointcloud.h"
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp" 

#ifndef OFFLINE
#define OFFLINE 1 
#endif

int main(int argc, char * argv[]){
    rclcpp::init(argc, argv);
    std::cout<<"DEBUG3"<<std::endl;
    if(OFFLINE){
        std::cout<<"DEBUG2"<<std::endl;
        rclcpp::spin(std::make_shared<projectPointcloud>((int)OFFLINE));
        std::cout<<"DEBUG1"<<std::endl;
        rclcpp::shutdown();
    }
    std::cout<<"DEBUG4"<<std::endl;
    return 0;
}
