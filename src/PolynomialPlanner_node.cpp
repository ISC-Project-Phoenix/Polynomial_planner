#include "polynomial_planner/PolynomialPlanner.hpp"

// Required for doTransform
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

// For _1
using namespace std::placeholders;

polynomial_planner::polynomial_planner(const rclcpp::NodeOptions& options) : Node("polynomial_planner", options) {
    // Parameters
    // float x = this->declare_parameter<float>("foo", -10.0);

    // Random params (for testing but idk whtat else it is for)
    this->declare_parameter("test_latency", false);
    auto debug = this->declare_parameter("debug", true);

    // Frame params
    this->declare_parameter("path_frame", "odom");

    this->tracks_sub = this->create_subscription<geometry_msgs::msg::PoseArray>(
        "/tracks", 5, std::bind(&ObjPlannerNode::tracks_cb, this, _1));
    this->path_pub = this->create_publisher<nav_msgs::msg::Path>("/path", 5);

    // TF2 things
    this->tf2_buffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    this->tf2_listener = std::make_unique<tf2_ros::TransformListener>(*this->tf2_buffer);


    // Initialize strategies
    

    if (debug) {
        cv::namedWindow("Hull");
    }

    // Initialize strategies
    this->backend = std::make_unique<SimpleBackEnd>(SimpleBackEndParams{});
    this->frontEnd = std::make_unique<ConvexMethod>(cm_p, this->get_logger());

    // Pub Sub
    // this is were we get the subscription
    this->sub =
        this->create_subscription<std_msgs::msg::String>("/str", 1, std::bind(&polynomial_planner::sub_cb, this, _1));
    this->pub = this->create_publisher<std_msgs::msg::String>("/run_folder", 1);

    // Log a sample log
    RCLCPP_INFO(this->get_logger(), "You passed %f", x);

    // Send a sample message
    std_msgs::msg::String msg{};
    msg.data = std::string{"Hello World!"};
    pub->publish(msg);

    // Random params
    // test latency is used for the Odom Data I think
    this->declare_parameter("test_latency", false);
    auto debug = this->declare_parameter("debug", true);

    // Frame params
    // I believe this is the Odom frame for the kart to follow.
    this->declare_parameter("path_frame", "odom");
}

void polynomial_planner::tracks_cb(const std_msgs::msg::String::SharedPtr msg) {
    // Echo message
    this->pub->publish(*msg);
}

// latency calulation for the 
void ObjPlannerNode::calc_latency(long ms) const {
    static std::array<uint64_t, 300> measurements;
    static uint64_t index = 0;
    measurements[index] = ms;
    index = index + 1 > measurements.size() - 1 ? 0 : index + 1;

    // Calc statistics
    double mean = 0;
    for (uint64_t i = 0; i != index; ++i) {
        mean += (double)measurements[i];
    }
    mean /= (double)index + 1;

    double mean2 = 0;
    for (uint64_t i = 0; i != index; ++i) {
        mean2 += std::pow((double)measurements[i], 2);
    }
    mean2 /= (double)index + 1;

    double std_dev = sqrt(mean2 - std::pow(mean, 2));

    RCLCPP_INFO(get_logger(), "Mean: %fms Std-dev: %fms", mean * 1e-6, std_dev * 1e-6);
}