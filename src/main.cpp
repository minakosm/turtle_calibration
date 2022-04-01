#include "projectPointcloud.h"
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp" 

#include "rosbag2_cpp/reader.hpp"
#include "rosbag2_cpp/readers/sequential_reader.hpp"
#include "rosbag2_cpp/writer.hpp"
#include "rosbag2_cpp/writers/sequential_writer.hpp"
#include "rosbag2_storage/serialized_bag_message.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/serialization.hpp"
#include "rclcpp/serialized_message.hpp"

sensor_msgs::msg::PointCloud2 get_pcl_from_rosbag() {
    rosbag2_storage::StorageOptions in_storage_options;
    in_storage_options.uri = "/home/minakosm/turtle_track_data/track_data_19_11_2021/rosbag2_2021_11_19_09-48-35_autoX";
    //in_storage_options.uri = "/media/panos/Kingston16G/Aristurtle_Flash/track_data_19_11_2021/rosbag2_2021_11_19_09-48-35_autoX";
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

        auto timestamp = bag_message->time_stamp;  // nanoseconds since unix epoch

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

int main(int argc, char* argv[]) {

    sensor_msgs::msg::PointCloud2 pclMsg = get_pcl_from_rosbag();
    std::cout <<"pcl point step: "<<pclMsg.point_step<<std::endl;
    // sensor_msgs::msg::PointCloud2::SharedPtr p(new sensor_msgs::msg::PointCloud2(pclMsg));
    //std::cout <<"p point step: "<<p->point_step<<std::endl;
    transform_pointcloud(pclMsg);
    return 0;
}
