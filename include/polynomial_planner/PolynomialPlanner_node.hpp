#pragma once

// #include <nav_msgs/msg/path.hpp>

#include "opencv2/core/types.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
// #include "std_msgs/msg/string.hpp"

class PolynomialPlanner : public rclcpp::Node {
private:
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub;

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub;

    // std::optional<nav_msgs::msg::Path>
    std::unique_ptr<std::optional<nav_msgs::msg::Path>> Backend;

    // TF2 stuff
    td::unique_ptr<tf2_ros::TransformListener> tf2_listener;
    td::unique_ptr<tf2_ros::Buffer> tf2_buffer;

public:
    PolynomialPlanner(const rclcpp::NodeOptions& options);

    /// subscriber callback
    void sub_cb(std_msgs::msg::String::SharedPtr msg);
};
