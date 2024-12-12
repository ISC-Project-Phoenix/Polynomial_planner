#include "polynomial_planner/PolynomialPlanner.hpp"

// For _1
using namespace std::placeholders;

polynomial_planner::polynomial_planner(const rclcpp::NodeOptions& options) : Node("polynomial_planner", options) {
    // Parameters
    float x = this->declare_parameter<float>("foo", -10.0);

    // Pub Sub
    // this is were we get the subscription
    this->sub =
        this->create_subscription<std_msgs::msg::String>("/str", 1, std::bind(&polynomial_planner::sub_cb, this, _1));
    this->pub = this->create_publisher<std_msgs::msg::String>("/run_folder", 1);

    // Log a sample log
    RCLCPP_INFO(this->get_logger(), "You passed %f", x);

    // Send a sample message
    std_msgs::msg::String msg{};
    msg.data = std::string{"Hello World!"};
    pub->publish(msg);

    // Random params
    // test latency is used for the Odom Data I think
    this->declare_parameter("test_latency", false);
    auto debug = this->declare_parameter("debug", true);

    // Frame params
    // I believe this is the Odom frame for the kart to follow.
    this->declare_parameter("path_frame", "odom");
}

void polynomial_planner::tracks_cb(const std_msgs::msg::String::SharedPtr msg) {
    // Echo message
    this->pub->publish(*msg);
}
