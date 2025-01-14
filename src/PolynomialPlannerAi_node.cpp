#include "polynomial_planner/PolynomialPlannerAi_node.hpp"

// Required for doTransform
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

// For _1
using namespace std::placeholders;

PolynomialPlannerAi::PolynomialPlannerAi(const rclcpp::NodeOptions& options) : Node("polynomial_planner_ai", options) {

}

void PolynomialPlannerAi::sub_cb(std_msgs::msg::String::SharedPtr msg) {

}