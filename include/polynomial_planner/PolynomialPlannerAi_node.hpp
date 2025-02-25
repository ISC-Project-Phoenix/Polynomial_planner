#pragma once

#include <opencv2/core/types.hpp>
#include <string>
#include <vector>

#include "image_geometry/pinhole_camera_model.h"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "std_msgs/msg/float32_multi_array.hpp"

class PolynomialPlannerAi : public rclcpp::Node {
private:
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub;

    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr poly_sub;

    // Camera info sub & model vars
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr rgb_info_sub;
    image_geometry::PinholeCameraModel rgb_model;

    // std::optional<nav_msgs::msg::Path>
    std::unique_ptr<std::optional<nav_msgs::msg::Path>> Backend;

    // TF2 stuff
    std::unique_ptr<tf2_ros::TransformListener> tf2_listener;
    std::unique_ptr<tf2_ros::Buffer> tf2_buffer;

public:
    PolynomialPlannerAi(const rclcpp::NodeOptions& options);

    /// subscriber callback
    /// 'PolynomialPlannerAi::' is unnesscary
    void polynomial_cb(std_msgs::msg::Float32MultiArray::SharedPtr msg, image_geometry::PinholeCameraModel camera_rgb);
    void evaluate_polynomial(const std::vector<float>& coeffs, const std::vector<float>& x_values);
};
