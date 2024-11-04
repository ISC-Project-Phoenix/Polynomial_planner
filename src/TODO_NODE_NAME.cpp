#include "TODO_PACKAGE_NAME/TODO_NODE_NAME_node.hpp"

int main(int argc, char** argv) {
    // Setup runtime
    rclcpp::init(argc, argv);
    rclcpp::executors::MultiThreadedExecutor exec;
    rclcpp::NodeOptions options;

    // Add nodes to executor
    auto node = std::make_shared<TODO_NODE_NAME>(options);
    exec.add_node(node);

    // Run
    exec.spin();
    rclcpp::shutdown();
    return 0;
}
