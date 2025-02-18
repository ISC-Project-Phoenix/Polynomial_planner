#include "polynomial_planner/PolynomialPlanner_node.hpp"

// Required for doTransform
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/camera_info.hpp"

#include "std_msgs/msg/string.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

// For _1
using namespace std::placeholders;

PolynomialPlanner::PolynomialPlanner(const rclcpp::NodeOptions& options) : Node("polynomial_planner", options) {
    // delcare params
    auto random_int = this->declare_parameter("random_int", 0);

    pub = this->create_publisher<std_msgs::msg::String>("topic", 10);

    this->rgb_info_sub = this->create_subscription<sensor_msgs::msg::CameraInfo>(
        "/camera/mid/rgb/camera_info", 1,
        [this](sensor_msgs::msg::CameraInfo::ConstSharedPtr ci) { this->rgb_model.fromCameraInfo(ci); });

    // TF2 things
    //  this->tf2_buffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    // this->tf2_listener = std::make_unique<tf2_ros::TransformListener>(*this->tf2_buffer);
}

void PolynomialPlanner::sub_cb(std_msgs::msg::String::SharedPtr msg) {}

// why are the pointer things the way they are
// TODO: make it not die when z is too small
//       or make z not too small
nav_msgs::msg::Path PolynomialPlanner::cameraPixelToGroundPos(std::vector<cv::Point2d>& pixels) {
    std::vector<cv::Point2d> rwpoints;

    for (const cv::Point2d& pixel : pixels) {
        // gotta rectify the pixel before we raycast
        cv::Point2d rectPixel = this->rgb_model.rectifyPoint(pixel);
        cv::Point3d ray = this->rgb_model.projectPixelTo3dRay(rectPixel);

//        TODO:
//        Vector3D intersectPoint(Vector3D rayVector, Vector3D rayPoint, Vector3D planeNormal, Vector3D planePoint) {
//            Vector3D diff = rayPoint - planePoint;
//            double prod1 = diff.dot(planeNormal);
//            double prod2 = rayVector.dot(planeNormal);
//            double prod3 = prod1 / prod2;
//            return rayPoint - rayVector * prod3;
//        }


    }

    return rwpoints;
}