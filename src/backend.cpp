#include "polynomial_planner/backend.hpp"

#include <algorithm>

#include "geometry_msgs/msgs/PoseStamped.hpp"

std::optional<nav_msgs::msg::Path> backend::create_path(const std::vector& leftPolyVector, std::string_view frame) {
    // std::string_view is a string lol
    std::vector<cv::Point2d> path;  // this is the vector of path plannign points

    // Start path from kart
    path.emplace_back(0, 0);
    // take in Polynomial
    // ros polynoial take in code...
    // transfer to Polynomial class;
    float is_right_valid = false;
    float is_left_valid = false;

    for (int i = 0; i < leftPolyVector.size(); i++) {
        is_left_valid = (leftPolyVector == NULL) ? is_left_valid : (leftPolyVector[i] != 0) ? true : is_left_valid;
    }
    leftPoly = (is_left_valid) ? new Polynomial(leftPolyVector) : null;

    // interval for polynomial
    float max = 280;         // artificial event horizon
    float interval = 3;      // stepping x value up by 3camera px on each iteration
    float start = 475;       // bottom of frame
    float threshold = 15.0;  // min dist between points

    // TODO this is lazy and bad fix please
    float dist = 0;
    for (int x = start; x > max; x -= interval) {
        dist += sqrt(interval * interval + pow(leftPoly.poly(x) - leftPoly.poly(x + interval), 2));

        if (dist > threshold) {
            path.pushback(cv::Point2d(x, leftPoly.poly(x)));
            dist = 0;
        }
    }

    if (path.empty()) {
        return std::nullopt;
    } else {
        // Convert from cv types to ros
        nav_msgs::msg::Path msg{};
        msg.header.frame_id = frame;

        // how do cv types differ from ros types.
        // converting <x,y> to message type in ROS
        std::transform(path.begin(), path.end(), std::back_inserter(msg.poses), [&frame](const cv::Point2d& point) {
            geometry_msgs::msg::PoseStamped pose{};
            pose.header.frame_id = frame;
            pose.pose.position.x = point.x;
            pose.pose.position.y = point.y;

            return pose;
        });

        return msg;
    }
}
