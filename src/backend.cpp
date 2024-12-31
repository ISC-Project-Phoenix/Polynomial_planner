#include "polynomial_planner/backend.hpp"

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
        is_left_valid   = ( listenerArry[i] ) ? true : is_left_valid;
        is_right_valid  = ( listenerArry[i] ) ? true : is_right_valid;
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

    // Pairs points from the longer side with the closest point on the shorter side
    for (auto& more_point : vec_more) {
        std::vector<double> distance;   // this is unnesscary for us
        cv::Point2d nearest;            // also unnesscary
        // instead we can take the mid points for polynomials

        // Aggregate distances between sides to find the closest point
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
