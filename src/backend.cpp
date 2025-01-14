#include "polynomial_planner/backend.hpp"
#include "geometry_msgs/msgs/PoseStamped.hpp"

#include <algorithm>

std::optional<nav_msgs::msg::Path> backend::create_path(const std::vector& leftPoly,
                                                        const std::vector& rightPoly
                                                        std::string_view frame) {
// std::string_view is a string lol
    std::vector<cv::Point2d> path;  // this is the array for cone points
                                    // this will make the
    // Start path from kart
    path.emplace_back(0, 0);        
    // take in Polynomial
    // ros polynoial take in code...
    // transfer to Polynomial class;
    float is_right_valid = false;
    float is_left_valid = false;

    for (int i = 0; i  < /* listenerArry.length*/; i++){

        is_left_valid   = ( leftPoly  == NULL) ? is_left_valid : ( leftPoly[i]  != 0 ) ? true : is_left_valid;

        is_right_valid  = ( rightPoly == NULL) ? is_right_valid : ( rightPoly[i] != 0 ) ? true : is_right_valid;
    }
    leftPoly = (is_left_valid) ? new Polynomial( /* vector from ros listener */) : null;
    if(is_left_valid){
        Polynomial leftPoly = new Polynomial( /* vector from ros listener */);
    }else {
        // TODO this is lazy and bad fix please
        Polynomial leftPoly = null;
    }
    rightPoly = (is_right_valid) ? new Polynomial( /* vector from ros listener */) : null;
    if(is_right_valid){
        Polynomial rightPoly = new Polynomial( /* vector from ros listener */);
    } else {
        // TODO this is lazy and bad fix please
        Polynomial rightPoly = null;
    }

    // interval for polynomial
    float max       =   10;
    float interval  =   0.25;
    float start     =   0;
    std::vector<cv::Point2d> nav_points;
    for (float i = start; i < max; i += interval){
        // generate points
        nav_points.pushback();
        if (leftPoly != null){
            // do left poly math;
            // <y, -x>
            float dx = leftPoly.polyDirvative(x);
            // calulate dx were dy is 1
            // becuase I dont want to use a wrapper class
            float dy = 1;
            // set dy to 1
            float l = std::sqrt( dx*dx + dy*dy);
            // find the magnitude
            dx = dx/l;  // normalize
            dy = dy/l;  // normalize
            cv::Point2d temp;
            float x = i;                // define x 
            float y = leftPoly.poly(x); // define y
            temp.x = (x + 7 * dy);      // project x
            temp.y = (y - 7 * dx )      // project y
            nav_points[i].pushback(temp);
            // return vector as < y, -x >
        }
        if (rightPoly != null){
            // do right poly math
            // <-y, x>
            // same steps different numbers
            float dx = rightPoly.polyDirvative(x);
            float dy = 1;
            float l = std::sqrt( dx*dx + dy*dy);
            l = std::hypot(dx, dy);
            dx = dx/l;
            dy = dy/l;
            cv::Point2d temp;
            float x = i;
            float y = leftPoly.poly(x);
            temp.x = (x - 7 * dy);
            temp.y = (y + 7 * dx )
            nav_points[i].pushback(temp);
            // return vector as < -y , x >
        }
        // rememebr to return

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
