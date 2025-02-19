#include "polynomial_planner/PolynomialPlannerAi_node.hpp"

// Required for doTransform
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

// For _1
using namespace std::placeholders;

PolynomialPlannerAi::PolynomialPlannerAi(const rclcpp::NodeOptions& options) : Node("polynomial_planner_ai", options) {
    this->poly_sub = this->create_subscription<std_msgs::msg::Float32MultiArray>(
        "/road/polynomial", 1, std::bind(&PolynomialPlannerAi::polynomial_cb, this, _1));

    this->path_pub = this->create_publisher<nav_msgs::msg::Path>("/path", 5);

    // Copy rgb camera info directly into model
    this->rgb_info_sub = this->create_subscription<sensor_msgs::msg::CameraInfo>(
        "/camera/mid/rgb/camera_info", 1,
        [this](sensor_msgs::msg::CameraInfo::ConstSharedPtr ci) { this->rgb_model.fromCameraInfo(ci); });
}

void PolynomialPlannerAi::polynomial_cb(std_msgs::msg::Float32MultiArray::SharedPtr msg) {
    RCLCPP_INFO(this->get_logger(), "My log message %d", 4);
}