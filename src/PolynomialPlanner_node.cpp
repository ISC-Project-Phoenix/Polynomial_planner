
#include "polynomial_planner/PolynomialPlanner_node.hpp"

// Required for doTransform
#include <tf2/LinearMath/Quaternion.h>

#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

// For _1
using namespace std::placeholders;

PolynomialPlanner::PolynomialPlanner(const rclcpp::NodeOptions& options) : Node("polynomial_planner", options) {
    // delcare params
    auto random_int = this->declare_parameter("random_int", 0);
    this->declare_parameter("camera_frame", "mid_cam_link");

    this->path_pub = this->create_publisher<nav_msgs::msg::Path>("/path", 5);

    // run once get pointer never used again
    this->rgb_info_sub = this->create_subscription<sensor_msgs::msg::CameraInfo>(
        "/camera/mid/rgb/camera_info", 1,
        [this](sensor_msgs::msg::CameraInfo::ConstSharedPtr ci) { this->rgb_model.fromCameraInfo(ci); });

    // TF2 things
    this->tf2_buffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    this->tf2_listener = std::make_unique<tf2_ros::TransformListener>(*this->tf2_buffer);
}

void polynomial_cb(std_msgs::msg::String::SharedPtr) {
    if (msg->empty()) {
        RCLCPP_WARN(this->get_logger(), "Received empty polynomial (non-AI)");
        return;

    } else {
        std::vector<float> coeff{};
        int no_coeff = msg->data.size();

        for (int i = 0; i < no_coeff; i++) {
            coeff.push_back(msg->data[i]);
        }
        // TODO fix params
        nav_msgs::msg::Path path = backend::create_path(coeff, camera_rgb, frame_id);

        geometry_msgs::msg::PoseStamped path_poses[] = path.poses;

        bool do_logger = true;
        if (do_logger) {
            for (int i = 0; i < path_poses.size(); i++) {
                RCLCPP_INFO(this->get_logger(), " we have polynomial");
            }
        }
        this->path_pub->publish(*path);
    }
}
