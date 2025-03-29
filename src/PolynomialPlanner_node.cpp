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

    this->rgb_info_sub = this->create_subscription<sensor_msgs::msg::CameraInfo>(
        "/camera/mid/rgb/camera_info", 1,
        [this](sensor_msgs::msg::CameraInfo::ConstSharedPtr ci) { this->rgb_model.fromCameraInfo(ci); });

    // std::vector<geometry_msgs::msg::Vector3> left_contour;
    // std::vector<geometry_msgs::msg::Vector3> right_contour;

    this->poly_sub = this->create_subscription<phnx_msgs::msg::Contours>(
        "/road/polynomial", 1, [this](const phnx_msgs::msg::Contours::SharedPtr msg) {
            this->polynomial_cb(msg, this->rgb_model);
        });  ///  TODO fix paras

    this->path_pub = this->create_publisher<nav_msgs::msg::Path>("/path", 5);

    RCLCPP_INFO(this->get_logger(), "PolynomialPlannerAi Node Started! Waiting for polynomial data...");
    // TF2 things
    this->tf2_buffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    this->tf2_listener = std::make_unique<tf2_ros::TransformListener>(*this->tf2_buffer);
}

void polynomial_cb(std_msgs::msg::Float32MultiArray::SharedPtr msg, image_geometry::PinholeCameraModel camera_rgb) {
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
        std::optional<nav_msgs::msg::Path> path_optional = backend::create_path(left, right, camera_rgb, frame_id);
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

void polynomial_pb(std_msgs::msg::Float32MultiArray::SharedPtr msg, image_geometry::PinholeCameraModel camera_rgb) {
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
        std::optional<nav_msgs::msg::Path> ground_path = backend::cameraPixelToGroundPos(coeff, camera_rgb, frame_id);
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
            for (size_t i = 0; i < msg->data.size(); i++) {
                // RCLCPP_INFO(this->get_logger(), "Coefficient[%zu] = %.15e", i, msg->data[i]);
            }
        }
        return;
    }
}

/* not so obvious reference:
geometry_msgs::msg::PoseArray PolynomialPlanner::project_to_world(const std::vector<cv::Point2d>& object_locations,
                                                                  const cv::Mat& depth) {
    geometry_msgs::msg::PoseArray poses{};
    poses.header.frame_id = this->get_parameter(std::string{"camera_frame"}).as_string();

    // Rotation that rotates left 90 and backwards 90.
    // This converts from camera coordinates in OpenCV to ROS coordinates
    tf2::Quaternion optical_to_ros{};
    optical_to_ros.setRPY(-M_PI / 2, 0.0, -M_PI / 2);

    for (const cv::Point2d& center : object_locations) {
        if (!this->rgb_model.initialized()) {
            continue;
        }

        // Project pixel to camera space
        auto ray = this->rgb_model.projectPixelTo3dRay(center);
        // The oak-d uses shorts in mm, sim uses f32 in m
        float dist = depth.type() == 2 ? (float)depth.at<short>(center) / 1000 : depth.at<float>(center);

        // If depth unavailable, then skip
        if (dist == INFINITY || dist >= 10 || dist <= 0) {
            continue;
        }

        // Just resize the ray to be a vector at the distance of the depth pixel. This is in camera space
        auto point_3d = dist / cv::norm(ray) * ray;

        // Convert from camera space to ros coordinates ("World" but wrt camera mount)
        tf2::Vector3 tf_vec{point_3d.x, point_3d.y, point_3d.z};
        auto world_vec = tf2::quatRotate(optical_to_ros, tf_vec);

        geometry_msgs::msg::Pose p{};
        p.position.x = world_vec.x();
        p.position.y = world_vec.y();
        p.position.z = world_vec.z();
        poses.poses.push_back(p);
    }

    poses.header.stamp = this->get_clock()->now();
    return poses;
}
    */