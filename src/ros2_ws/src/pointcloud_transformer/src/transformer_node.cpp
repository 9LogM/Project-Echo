#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "yaml-cpp/yaml.h"
#include <string>
#include <functional>
#include <fstream>

class PointCloudTransformer : public rclcpp::Node {
public:
    PointCloudTransformer() : Node("pointcloud_transformer")
    {
        YAML::Node config = YAML::LoadFile("/app/src/config.yaml");

        ros_topic_raw_ = config["ros_topic_raw"].as<std::string>();
        ros_topic_clean_ = config["ros_topic_clean"].as<std::string>();

        isaac_x_ = config["coordinate_mapping"]["isaac_x_axis"].as<std::string>();
        isaac_y_ = config["coordinate_mapping"]["isaac_y_axis"].as<std::string>();
        isaac_z_ = config["coordinate_mapping"]["isaac_z_axis"].as<std::string>();
        
        rclcpp::QoS qos_profile{rclcpp::KeepAll()};

        publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(ros_topic_clean_, qos_profile);
        subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            ros_topic_raw_, qos_profile,
            std::bind(&PointCloudTransformer::topic_callback, this, std::placeholders::_1));
            
        
        RCLCPP_INFO(this->get_logger(), "Subscribing to '%s', Publishing to '%s'", ros_topic_raw_.c_str(), ros_topic_clean_.c_str());
        RCLCPP_INFO(this->get_logger(), "Mapping: Isaac X=%s, Y=%s, Z=%s", isaac_x_.c_str(), isaac_y_.c_str(), isaac_z_.c_str());
    }

private:
    float get_value(const std::string& axis_map, float x, float y, float z) const {
        if (axis_map == "ros_x") return x; if (axis_map == "-ros_x") return -x;
        if (axis_map == "ros_y") return y; if (axis_map == "-ros_y") return -y;
        if (axis_map == "ros_z") return z; if (axis_map == "-ros_z") return -z;
        return 0.0f;
    }

    void topic_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) const {
        sensor_msgs::msg::PointCloud2 transformed_msg = *msg;
        sensor_msgs::PointCloud2Iterator<float> iter_x(transformed_msg, "x"), iter_y(transformed_msg, "y"), iter_z(transformed_msg, "z");
        sensor_msgs::PointCloud2ConstIterator<float> iter_orig_x(*msg, "x"), iter_orig_y(*msg, "y"), iter_orig_z(*msg, "z");

        for (; iter_orig_x != iter_orig_x.end(); ++iter_orig_x, ++iter_orig_y, ++iter_orig_z, ++iter_x, ++iter_y, ++iter_z) {
            const float ox = *iter_orig_x, oy = *iter_orig_y, oz = *iter_orig_z;
            *iter_x = get_value(isaac_x_, ox, oy, oz);
            *iter_y = get_value(isaac_y_, ox, oy, oz);
            *iter_z = get_value(isaac_z_, ox, oy, oz);
        }
        publisher_->publish(transformed_msg);
    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
    std::string ros_topic_raw_, ros_topic_clean_;
    std::string isaac_x_, isaac_y_, isaac_z_;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PointCloudTransformer>());
    rclcpp::shutdown();
    return 0;
}