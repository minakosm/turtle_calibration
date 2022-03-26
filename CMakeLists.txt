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


include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include
                    ${OpenCV_INCLUDE_DIRS}
                    ${YAML_CPP_INCLUDE_DIRS})

add_library(calibration src/projectPointcloud.cpp)
target_link_libraries(calibration Eigen3::Eigen)
target_link_libraries(calibration ${YAML_CPP_LIBRARIES})
target_link_libraries(calibration ${OpenCV_LIBRARIES})
ament_target_dependencies(calibration rclcpp std_msgs sensor_msgs ament_index_cpp tf2_ros)

add_executable(lidar_camera_project src/main.cpp)
target_link_libraries(lidar_camera_project calibration)
ament_target_dependencies(lidar_camera_project rclcpp rosbag2_cpp rosbag2_storage rcutils sensor_msgs)

install(TARGETS lidar_camera_project 
        DESTINATION lib/${PROJECT_NAME})

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
