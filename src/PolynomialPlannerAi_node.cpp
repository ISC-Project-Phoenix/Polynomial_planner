#include "polynomial_planner/PolynomialPlannerAi_node.hpp"

// Required for doTransform
#include <tf2/LinearMath/Quaternion.h>

#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"




using namespace std::placeholders;

PolynomialPlannerAi::PolynomialPlannerAi(const rclcpp::NodeOptions& options) : Node("polynomial_planner_ai", options) {
    this->poly_sub = this->create_subscription<std_msgs::msg::Float32MultiArray>(
        "/road/polynomial", 1, std::bind(&PolynomialPlannerAi::polynomial_cb, this, _1));

    this->path_pub = this->create_publisher<nav_msgs::msg::Path>("/path", 5);

    this->rgb_info_sub = this->create_subscription<sensor_msgs::msg::CameraInfo>(
        "/camera/mid/rgb/camera_info", 1,
        [this](sensor_msgs::msg::CameraInfo::ConstSharedPtr ci) { this->rgb_model.fromCameraInfo(ci); });

    RCLCPP_INFO(this->get_logger(), "PolynomialPlannerAi Node Started! Waiting for polynomial data...");
    // TF2 things
    this->tf2_buffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    this->tf2_listener = std::make_unique<tf2_ros::TransformListener>(*this->tf2_buffer);
}

void PolynomialPlannerAi::polynomial_cb(std_msgs::msg::Float32MultiArray::SharedPtr msg) {
    if (msg->empty()) {
        RCLCPP_WARN(this->get_logger(), "Received empty polynomial (non-AI)");
        return;

    } else {
        std::vector<float> coeff{};
        int no_coeff = msg->data.size();

        for (int i = 0; i < no_coeff; i++) {
            coeff.push_back(msg->data[i]);
        }

        nav_msgs::msg::Path path = backend::create_path(coeff, frame);

        geometry_msgs::msg::PoseStamped path_poses[] = path.poses;

        bool do_logger = true;
        if (do_logger) {
            for (int i = 0; i < path_poses.size(); i++) {
                // RCLCPP_INFO(this->get_logger(), "");
            }
        }
        this->path_pub->publish(*path);
    }

    // Extract and print coefficients
    RCLCPP_INFO(this->get_logger(), "Received Polynomial Coefficients:");
    for (size_t i = 0; i < msg->data.size(); i++) {
        RCLCPP_INFO(this->get_logger(), "Coefficient[%zu] = %.15e", i, msg->data[i]);
    }
}

void PolynomialPlannerAi::evaluate_polynomial(const std::vector<float>& coeffs, const std::vector<float>& x_values) {
    for (float x : x_values) {
        float y = 0.0;
        for (size_t i = 0; i < coeffs.size(); i++) {
            y += coeffs[i] * std::pow(x, i);
        }
        RCLCPP_INFO(this->get_logger(), "P(%f) = %f", x, y);
    }
}
