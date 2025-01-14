#include "polynomial_planner/PolynomialPlanner_node.hpp"

// Required for doTransform
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

// For _1
using namespace std::placeholders;

PolynomialPlanner::PolynomialPlanner(const rclcpp::NodeOptions& options) : Node("polynomial_planner", options) {

}

void PolynomialPlanner::sub_cb(std_msgs::msg::String::SharedPtr msg) {

}