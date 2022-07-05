#include "rclcpp/serialization.hpp"
#include "rclcpp/serialized_message.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "rosbag2_cpp/reader.hpp"
#include "rosbag2_cpp/readers/sequential_reader.hpp"
#include "rosbag2_cpp/writer.hpp"
#include "rosbag2_cpp/writers/sequential_writer.hpp"
#include "rosbag2_storage/serialized_bag_message.hpp"
#include <iostream>

sensor_msgs::msg::PointCloud2 get_pcl_from_rosbag() {
    rosbag2_storage::StorageOptions in_storage_options;
    // in_storage_options.uri = "/home/eleft/ros2_sim/src/turtle_calibration/rosbags/lidar_bag_chess";
    in_storage_options.uri = "/home/minakosm/git_projects/SensorsCalibration/lidar2camera/manual_calib/fusion_data/lidar_bag_center_camera";

    in_storage_options.storage_id = "sqlite3";
    in_storage_options.max_bagfile_size = 0;  // default
    in_storage_options.max_cache_size = 0;    // default

    rosbag2_cpp::ConverterOptions converter_options;
    converter_options.input_serialization_format = "cdr";
    converter_options.output_serialization_format = "cdr";

    rosbag2_cpp::Reader reader(std::make_unique<rosbag2_cpp::readers::SequentialReader>());
    reader.open(in_storage_options, converter_options);

    std::shared_ptr<rosbag2_storage::SerializedBagMessage> bag_message;

    while (reader.has_next()) {
        bag_message = reader.read_next();

        // auto timestamp = bag_message->time_stamp;  // nanoseconds since unix epoch

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
    sensor_msgs::msg::PointCloud2 temp;
    return temp;
}