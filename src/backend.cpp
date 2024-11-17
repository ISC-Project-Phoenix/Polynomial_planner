#include "polynomial_planner/backend.hpp"

#include <algorithm>

std::optional<nav_msgs::msg::Path> SimpleBackEnd::create_path(const LeftRightResults& detections_org,
                                                              std::string_view frame) {

    std::vector<cv::Point2d> path;  // this is the array for cone points
                                    // this will make the
    // Start path from kart
    path.emplace_back(0, 0);        // what is path? how do we know it references the kart

    //Sets vec_more and vec_less to proper side
    auto& vec_more = (detections.left_detections.size() <= detections.right_detections.size())
                         ? detections.right_detections
                         : detections.left_detections;
    auto& vec_less = (detections.left_detections.size() > detections.right_detections.size())
                         ? detections.right_detections
                         : detections.left_detections;

    // Pairs points from the longer side with the closest point on the shorter side
    for (auto& more_point : vec_more) {
        std::vector<double> distance;   // this is unnesscary for us
        cv::Point2d nearest;            // also unnesscary

        // Aggregate distances between sides to find the closest point
        // instead we can take the mid points for polynomials
        for (auto& less_point : vec_less) {
            // push the raw x and y?
            distance.push_back(std::hypot(more_point.x - less_point.x, more_point.y - less_point.y));
        }

        // unnesscary
        //finds index of vec_less where the distance is shortest
        auto min_value = std::min_element(distance.begin(), distance.end());

        //Tests if cones are too far apart
                // we will not need this lol
        if (*min_value <= params.tolerance_value) {

            // dont worry we are feeding raw lol
            nearest = vec_less[std::distance(distance.begin(), min_value)];

            // Path point is the midpoint between the sides
                // we could use this if we generate points too compare from both polynomials
            cv::Point2d temp;
            temp.x = (more_point.x + nearest.x) / 2;
            temp.y = (more_point.y + nearest.y) / 2;
            path.push_back(temp);

        }
    }

    if (path.empty()) {
        return std::nullopt;
    } else {
        // Convert from cv types to ros
        nav_msgs::msg::Path msg{};
        msg.header.frame_id = frame;

        // how do cv tyoes differ from ros types.
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
