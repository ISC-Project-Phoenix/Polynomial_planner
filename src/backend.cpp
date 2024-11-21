#include "polynomial_planner/backend.hpp"

#include <algorithm>

std::optional<nav_msgs::msg::Path> SimpleBackEnd::create_path(const LeftRightResults& detections_org,
                                                              std::string_view frame) {

    std::vector<cv::Point2d> path;  // this is the array for cone points
                                    // this will make the
    // Start path from kart
    path.emplace_back(0, 0);        // what is path? how do we know it references the kart

    // take in Polynomial
    // ros polynoial take in code...
    // transfer to Polynomial class;
    float sumRight = 0;
    float sumLeft = 0;
    for (int i = 0; i  < /* listenerArry.length*/; i++){
        sumLeft  +=  /* listenerArrayLeft[i]*/;
        sumRight +=  /* listenerArryRight[i]*/;
    }
    if(sumLeft != 0){
        auto rightPoly = new Polynomial( /* vector from ros listener */);
    }else {
        // TODO this is lazy and bad fix please
        auto rightPoly = null;
    }
    if(sumRight 1= 0){
        auto leftPoly = new Polynomial( /* vector from ros listener */);
    } else {
        // TODO this is lazy and bad fix please
        auto leftPoly = null;
    }
    class Polynomial {
        public:
            // TODO what does std::pmr::vec mean compared to std::vec ???
            Polynomial(std::pmr::vector<float>& vect) {
                this->vect = vect;
            }
            Polynomial() = default;
            // store the vector
        private:
            std::pmr::vector<float> vect;
        public:

            // returns the y value at a given X.
            float poly(float x) {
                float result = 0;
                for( int i = 0; i < vect.size(); i++ ) {
                    int power = vect.size() - i;
                    // TODO this pow function might be wrong
                    result += vect[i] * pow(x, power); // a[n] * x ^ n
                }
                return result;
            }
            float polyDirvative(float x) {
                float result = 0;
                for( int i = 0; i < vect.size() -1; i++ ) {
                    int power = vect.size() - i - 1;
                    // todo finish?
                    result += vect[i] * i * pow(x, power); // a[n] * n * x ^ (n - 1)
                }
            }
    };// end polynomial_class


    // interval for polynomial
    float max = 10;
    float interval = 0.1;
    for ( float i = 0; i < max; i += interval){
        // generate points
        if (leftPoly != null){
            // do left poly math;
        }
        if (rightPoly != null){
            // do right poly math
        }
        // then we go from where

    }

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
