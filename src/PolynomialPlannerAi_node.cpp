#include "polynomial_planner/PolynomialPlannerAi_node.hpp"
#include "polynomial_planner/backend.hpp"

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

PolynomialPlannerAi::PolynomialPlannerAi(const rclcpp::NodeOptions& options) : Node("polynomial_planner", options) {
    // RGB_INFO PARAMETER DO NOT DELETE
    this->declare_parameter("camera_frame", "mid_cam_link");

    this->rgb_info_sub = this->create_subscription<sensor_msgs::msg::CameraInfo>(
        "/camera/mid/rgb/camera_info", 1,
        [this](sensor_msgs::msg::CameraInfo::ConstSharedPtr ci) { this->rgb_model.fromCameraInfo(ci); });


        // std::vector<geometry_msgs::msg::Vector3> left_contour;
        // std::vector<geometry_msgs::msg::Vector3> right_contour;

    this->poly_sub = this->create_subscription<phnx_msgs::msg::Contours>(
        "/road/polynomial", 1, [this](phnx_msgs::msg::Contours::SharedPtr msg) {
            this->polynomial_cb(msg, this->rgb_model);
        });  ///  TODO fix paras

    this->path_pub = this->create_publisher<nav_msgs::msg::Path>("/path", 5);

    RCLCPP_INFO(this->get_logger(), "PolynomialPlannerAi Node Started! Waiting for polynomial data...");
    // TF2 things
    this->tf2_buffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    this->tf2_listener = std::make_unique<tf2_ros::TransformListener>(*this->tf2_buffer);
}

void PolynomialPlannerAi::polynomial_cb(phnx_msgs::msg::Contours::SharedPtr msg, image_geometry::PinholeCameraModel camera_rgb) {
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

        for (const auto& vec : left) {
            cv_points_right.emplace_back(vec.x, vec.y);  // Efficient in-place construction
        }

        std::string p = std::to_string(camera_rgb.cx());
        RCLCPP_INFO(this->get_logger(), p.c_str());

        //std::string frame_id = this->get_parameter("camera_frame").as_string();
        //std::string frame_id = "notemptystring";
        // TODO camera frame_id is wrong
        auto frame_id = this->get_parameter(std::string("camera_frame")).as_string();
        std::optional<nav_msgs::msg::Path> path_optional = backend::create_path(cv_points_left, cv_points_right, camera_rgb, frame_id);
        nav_msgs::msg::Path path;

        if (path_optional.has_value()) {
            path = path_optional.value();
            path.header.frame_id = this->get_parameter(std::string("camera_frame")).as_string();
            this->path_pub->publish(path);  // error invalid operator *path
                                            // Extract and print coefficients
            // RCLCPP_INFO(this->get_logger(), "Received Polynomial Coefficients:");
        }
        return;
    }
}

void PolynomialPlannerAi::polynomial_pb(phnx_msgs::msg::Contours::SharedPtr msg, image_geometry::PinholeCameraModel camera_rgb) {
    /*
    // fix msg->empty
    if (false) {
        RCLCPP_WARN(this->get_logger(), "Received empty polynomial (non-AI)");
        return;

    } else {
        std::vector<geometry_msgs::msg::Vector3> left = msg->left_contour;
        std::vector<geometry_msgs::msg::Vector3> right = msg->right_contour;

        std::string p = std::to_string(camera_rgb.cx());
        RCLCPP_INFO(this->get_logger(), p.c_str());

        //std::string frame_id = this->get_parameter("camera_frame").as_string();
        //std::string frame_id = "notemptystring";
        // TODO camera frame_id is wrong
        auto frame_id = this->get_parameter(std::string("camera_frame")).as_string();
        std::optional<nav_msgs::msg::Path> ground_path = backend::create_path(left, right, camera_rgb, frame_id);
        nav_msgs::msg::Path path;

        if (ground_path.has_value()) {
            // Convert from cv types to nav::msg
            nav_msgs::msg::Path msg{};
            // msg.header.frame_id = frame;
            // for (cv::Point2d ground_points : ground_path) {
            // std::
            //  }
            // converting <x,y> to message type in ROS
            std::transform(ground_path.begin(), ground_path.end(), std::back_inserter(msg.poses),
                           [&frame](const cv::Point2d& point) {
                               geometry_msgs::msg::PoseStamped pose{};
                               // frame = "redto0 isn't sure if we use this";
                               // redto0 is SURE that we use this update and fix ASAP
                               pose.header.frame_id = frame;  // literally is "notaemptystring"
                               pose.pose.position.x = point.x;
                               pose.pose.position.y = point.y;
                               // pose.pose.position.z = point.z;

                               return pose;
                           });
            path.header.frame_id = this->get_parameter(std::string("camera_frame")).as_string();
            this->path_pub->publish(*msg);  // error invalid operator *path
                                            // Extract and print coefficients
            // RCLCPP_INFO(this->get_logger(), "Received Polynomial Coefficients:");
        }
        return;
    }
        */
}