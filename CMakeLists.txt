cmake_minimum_required(VERSION 3.8)
project(turtle_calibration)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(ament_index_cpp)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(rosbag2_storage REQUIRED)
find_package(rcutils REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(PCL REQUIRED QUIET COMPONENTS core common features filters io segmentation surface)
find_package(pcl_conversions REQUIRED)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include
                    ${OpenCV_INCLUDE_DIRS}
                    ${YAML_CPP_INCLUDE_DIRS})

add_executable(ros2bag_to_pcd src/ros2bag_to_pcd.cpp)
target_link_libraries(ros2bag_to_pcd Eigen3::Eigen ${YAML_CPP_LIBRARIES} ${OpenCV_LIBRARIES} ${PCL_LIBRARIES})
ament_target_dependencies(ros2bag_to_pcd rclcpp std_msgs ament_index_cpp tf2_ros rosbag2_cpp rosbag2_storage rcutils sensor_msgs pcl_conversions PCL)

install(TARGETS ros2bag_to_pcd
        DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
        settings
        images
        rosbags
        DESTINATION share/${PROJECT_NAME})
# install(FILES 
#         lidar_camera_extrinsic.ini
#         intrinsic_params0.yaml
#         intrinsic_params1.yaml
#         intrinsic_params2.yaml
#         DESTINATION share/${PROJECT_NAME})    

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
