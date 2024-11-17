#pragma once

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class TODO_NODE_NAME : public rclcpp::Node {
private:
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub;

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub;

public:
    TODO_NODE_NAME(const rclcpp::NodeOptions& options);

    /// subscriber callback
    void sub_cb(std_msgs::msg::String::SharedPtr msg);
};
