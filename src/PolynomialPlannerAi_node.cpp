#include "polynomial_planner/PolynomialPlannerAi_node.hpp"

#include <string>

#include "backend.cpp"

// Required for doTransform
#include <tf2/LinearMath/Quaternion.h>

#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "image_geometry/pinhole_camera_model.h"

using namespace std::placeholders;

PolynomialPlannerAi::PolynomialPlannerAi(const rclcpp::NodeOptions& options) : Node("polynomial_planner_ai", options) {
    // RGB_INFO PARAMETER DO NOT DELETE
    this->declare_parameter("camera_frame", "mid_cam_link");

    this->rgb_info_sub = this->create_subscription<sensor_msgs::msg::CameraInfo>(
        "/camera/mid/rgb/camera_info", 1,
        [this](sensor_msgs::msg::CameraInfo::ConstSharedPtr ci) { this->rgb_model.fromCameraInfo(ci); });

    this->poly_sub = this->create_subscription<std_msgs::msg::Float32MultiArray>(
        "/road/polynomial", 1, [this](const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
            this->polynomial_cb(msg, this->rgb_model);
        });  ///  TODO fix paras

    this->path_pub = this->create_publisher<nav_msgs::msg::Path>("/path", 5);

    RCLCPP_INFO(this->get_logger(), "PolynomialPlannerAi Node Started! Waiting for polynomial data...");
    // TF2 things
    this->tf2_buffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    this->tf2_listener = std::make_unique<tf2_ros::TransformListener>(*this->tf2_buffer);
}

void PolynomialPlannerAi::polynomial_cb(std_msgs::msg::Float32MultiArray::SharedPtr msg,
                                        image_geometry::PinholeCameraModel camera_rgb) {
    // fix msg->empty
    if (false) {
        RCLCPP_WARN(this->get_logger(), "Received empty polynomial (non-AI)");
        return;

    } else {
        std::vector<float> coeff{};
        int no_coeff = msg->data.size();

        for (int i = 0; i < no_coeff; i++) {
            coeff.push_back(msg->data[i]);
        }

        std::string p = std::to_string(camera_rgb.cx());
        RCLCPP_INFO(this->get_logger(), p.c_str());

        //std::string frame_id = this->get_parameter("camera_frame").as_string();
        //std::string frame_id = "notemptystring";
        // TODO camera frame_id is wrong
        auto frame_id = this->get_parameter(std::string("camera_frame")).as_string();
        std::optional<nav_msgs::msg::Path> path_optional = backend::create_path(coeff, camera_rgb, frame_id);
        nav_msgs::msg::Path path;

        if (path_optional.has_value()) {
            path = path_optional.value();
            path.header.frame_id = this->get_parameter(std::string("camera_frame")).as_string();
            this->path_pub->publish(path);  // error invalid operator *path
                                            // Extract and print coefficients
            // RCLCPP_INFO(this->get_logger(), "Received Polynomial Coefficients:");
            for (size_t i = 0; i < msg->data.size(); i++) {
                // RCLCPP_INFO(this->get_logger(), "Coefficient[%zu] = %.15e", i, msg->data[i]);
            }
        }
        return;
    }
}

void PolynomialPlannerAi::evaluate_polynomial(const std::vector<float>& coeffs, const std::vector<float>& x_values) {
    for (float x : x_values) {
        float y = 0.0;
        for (size_t i = 0; i < coeffs.size(); i++) {
            y += coeffs[i] * std::pow(x, i);
        }
        // RCLCPP_INFO(this->get_logger(), "P(%f) = %f", x, y);
    }
}
