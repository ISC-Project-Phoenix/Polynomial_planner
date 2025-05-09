
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
    // RGB_INFO PARAMETER DO NOT DELETE
    this->declare_parameter("camera_frame", "mid_cam_link");

    // Frame params
    this->declare_parameter("path_frame", "odom");

    this->rgb_info_sub = this->create_subscription<sensor_msgs::msg::CameraInfo>(
        "/camera/mid/rgb/camera_info", 1,
        [this](sensor_msgs::msg::CameraInfo::ConstSharedPtr ci) { this->rgb_model.fromCameraInfo(ci); });

    this->poly_sub = this->create_subscription<phnx_msgs::msg::Contours>(
        "/road/Contours", 1, [this](phnx_msgs::msg::Contours::SharedPtr msg) {
            this->polynomial_cb(msg, this->rgb_model);
        });  ///  TODO fix paras

    this->path_pub = this->create_publisher<nav_msgs::msg::Path>("/path", 5);

    RCLCPP_INFO(this->get_logger(), "PolynomialPlanner Node Started! Waiting for contour data...");
    // TF2 things
    this->tf2_buffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    this->tf2_listener = std::make_unique<tf2_ros::TransformListener>(*this->tf2_buffer);
}

void polynomial_cb(polynomial_cb(phnx_msgs::msg::Contours::SharedPtr msg,
    image_geometry::PinholeCameraModel camera_rgb) {
    // fix msg->empty
    if (false) {
        RCLCPP_WARN(this->get_logger(), "Received empty polynomial (non-AI)");
        return;

    } else {
        std::vector<geometry_msgs::msg::Vector3> left = msg->left_contour;
        std::vector<geometry_msgs::msg::Vector3> right = msg->right_contour;

        std::vector<cv::Point2d> cv_points_left;
        std::vector<cv::Point2d> cv_points_right;

        for (const auto& vec : left) {
            cv_points_left.emplace_back(vec.x, vec.y);  // Efficient in-place construction
        }

        for (const auto& vec : right) {
            cv_points_right.emplace_back(vec.x, vec.y);  // Efficient in-place construction
        }

        std::string p = "left contour size " + std::to_string(left.size());
        RCLCPP_INFO(this->get_logger(), p.c_str());

        p = "right contour size " + std::to_string(right.size());
        RCLCPP_INFO(this->get_logger(), p.c_str());

        //std::string frame_id = this->get_parameter("camera_frame").as_string();
        //std::string frame_id = "notemptystring";
        // TODO camera frame_id is wrong
        auto frame_id = this->get_parameter(std::string("camera_frame")).as_string();
        std::optional<nav_msgs::msg::Path> path_optional =
            backend::create_path(cv_points_left, cv_points_right, camera_rgb, frame_id);
        nav_msgs::msg::Path path;

        if (path_optional.has_value()) {
            path = path_optional.value();
            std::string p = std::to_string(path.poses.size());
            RCLCPP_INFO(this->get_logger(), p.c_str());
            path.header.frame_id = this->get_parameter(std::string("camera_frame")).as_string();
            path.header.stamp = this->get_clock()->now();

            try {
                // Transform path to odom. While the path is in the same location, its now WRT where the kart started, which
                // means that as the kart moves and this path is transformed back WRT the kart, the path will have moved location.
                auto trans = this->tf2_buffer->lookupTransform(this->get_parameter("path_frame").as_string(),
                                                               path.header.frame_id, rclcpp::Time{});
                for (auto& pose : path.poses) {
                    tf2::doTransform(pose, pose, trans);
                    pose.header.frame_id = this->get_parameter("path_frame").as_string();
                    pose.header.stamp = this->get_clock()->now();
                }
                path.header.frame_id = this->get_parameter("path_frame").as_string();
            } catch (tf2::LookupException& e) {
                RCLCPP_INFO(this->get_logger(), "Could not look up odom!");
                return;
            }

            this->path_pub->publish(path);  // error invalid operator *path
                                            // Extract and print coefficients
            // RCLCPP_INFO(this->get_logger(), "Received Polynomial Coefficients:");
        } else {
            std::string p = " error no path ";
            RCLCPP_INFO(this->get_logger(), p.c_str());
        }
        return;
    }
}
