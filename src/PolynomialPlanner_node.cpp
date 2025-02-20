#include "polynomial_planner/PolynomialPlanner_node.hpp"

// Required for doTransform
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include <tf2/LinearMath/Quaternion.h>

// For _1
using namespace std::placeholders;

PolynomialPlanner::PolynomialPlanner(const rclcpp::NodeOptions& options) : Node("polynomial_planner", options) {
    // delcare params
    auto random_int = this->declare_parameter("random_int", 0);

    this->path_pub = this->create_publisher<nav_msgs::msg::Path>("/path", 5);

    this->rgb_info_sub = this->create_subscription<sensor_msgs::msg::CameraInfo>(
        "/camera/mid/rgb/camera_info", 1,
        [this](sensor_msgs::msg::CameraInfo::ConstSharedPtr ci) { this->rgb_model.fromCameraInfo(ci); });

    // TF2 things
    this->tf2_buffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    this->tf2_listener = std::make_unique<tf2_ros::TransformListener>(*this->tf2_buffer);
}

void PolynomialPlanner::sub_cb(std_msgs::msg::String::SharedPtr msg) {}

// why are the pointer things the way they are
// TODO: make it not die when z is too smallf
//       or make z not too small
std::vector<cv::Point2d> PolynomialPlanner::cameraPixelToGroundPos(std::vector<cv::Point2d>& pixels) {

    // Rotation that rotates left 90 and backwards 90.
        // This converts from camera coordinates in OpenCV to ROS coordinates
        tf2::Quaternion optical_to_ros{};
        // set the Roll Pitch YAW
        optical_to_ros.setRPY(-M_PI / 2, 0.0, -M_PI / 2);

    std::vector<cv::Point2d> rwpoints;

    for (const cv::Point2d& pixel : pixels) {
        // gotta rectify the pixel before we raycast
        cv::Point2d rectPixel = this->rgb_model.rectifyPoint(pixel);
        cv::Point3d ray = this->rgb_model.projectPixelTo3dRay(rectPixel);
        

        // ask zach for the trig, extend ray to the floor. 
        ray /= ray.z / -0.6;
        // ray.setZ(ray.getZ() * -1); we don't really care abt z, since it -will- *should* always just be cameraHeight
        tf2::Vector3 tf_vec{ray.x, ray.y, ray.z};
        tf2::Vector3 world_vec = tf2::quatRotate(optical_to_ros, tf_vec);

        //return type world_vec, use this is 

        cv::Point2d dvector(world_vec.x, world_vec.y);
        
        

        // push back vectors
        rwpoints.push_back(dvector);
    }

    return rwpoints;
}

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