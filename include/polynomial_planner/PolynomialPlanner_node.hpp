#pragma once

#include <nav_msgs/msg/path.hpp>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class PolynomialPlanner : public rclcpp::Node {
private:
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub;

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub;

    // std::optional<nav_msgs::msg::Path>
    std::unique_ptr<std::optional<nav_msgs::msg::Path>> Backend;

    // TF2 stuff
    // std::unique_ptr<tf2_ros::TransformListener> tf2_listener;
    // std::unique_ptr<tf2_ros::Buffer> tf2_buffer;
public:
    PolynomialPlanner(const rclcpp::NodeOptions& options);

    /// subscriber callback
    void sub_cb(std_msgs::msg::String::SharedPtr msg);
};
