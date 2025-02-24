#pragma once

#include <opencv2/core/types.hpp>
#include <vector>

#include "image_geometry/pinhole_camera_model.h"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

class PolynomialPlanner : public rclcpp::Node {
private:
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub;

    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr sub;

    // Camera info sub & model vars
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr rgb_info_sub;
    image_geometry::PinholeCameraModel rgb_model;

    // std::optional<nav_msgs::msg::Path>
    std::unique_ptr<std::optional<nav_msgs::msg::Path>> Backend;

    // TF2 stuff
    std::unique_ptr<tf2_ros::TransformListener> tf2_listener;
    std::unique_ptr<tf2_ros::Buffer> tf2_buffer;

public:
    PolynomialPlanner(const rclcpp::NodeOptions& options);

    /// subscriber callback
    void sub_cb(std_msgs::msg::String::SharedPtr msg);

    // camera transform
    nav_msgs::msg::Path cameraPixelToGroundPos(std::vector<cv::Point2d>& pixels);
};
