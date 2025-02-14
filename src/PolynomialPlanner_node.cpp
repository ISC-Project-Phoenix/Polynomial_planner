#include "polynomial_planner/PolynomialPlanner_node.hpp"

// Required for doTransform
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

// For _1
using namespace std::placeholders;

PolynomialPlanner::PolynomialPlanner(const rclcpp::NodeOptions& options) : Node("polynomial_planner", options) {
    // delcare params
    auto random_int = this->declare_parameter("random_int", 0);

    pub = this->create_publisher<std_msgs::msg::String>("topic", 10);

    // TF2 things
    //  this->tf2_buffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    // this->tf2_listener = std::make_unique<tf2_ros::TransformListener>(*this->tf2_buffer);
}

void PolynomialPlanner::sub_cb(std_msgs::msg::String::SharedPtr msg) {}