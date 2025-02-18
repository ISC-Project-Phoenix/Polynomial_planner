#pragma once

#include <nav_msgs/msg/path.hpp>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "image_geometry/pinhole_camera_model.h"


class PolynomialPlannerAi : public rclcpp::Node {
private:
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub;

    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr poly_sub;

    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr rgb_info_sub;

    image_geometry::PinholeCameraModel rgb_model;

public:
    PolynomialPlannerAi(const rclcpp::NodeOptions& options);

    /// subscriber callback
    void polynomial_cb(std_msgs::msg::Float32MultiArray::SharedPtr msg);
};
