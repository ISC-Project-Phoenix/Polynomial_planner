#include <cmath>
#include <fstream>
#include <iostream>
#include <optional>
#include <sensor_msgs/msg/camera_info.hpp>
#include <string_view>
#include <vector>

#include "image_geometry/pinhole_camera_model.h"
#include "nav_msgs/msg/path.hpp"

class Polynomial {
public:
    // TODO what does std::pmr::vec mean compared to std::vec ???
    Polynomial(std::vector<float>& vect) { this->vect = vect; }
    Polynomial() = default;
    // store the vector
private:
    std::vector<float> vect;

public:
    // returns the y value at a given X.
    float poly(float x) {
        float result = 0;
        for (int i = 0; i < vect.size(); i++) {
            int power = vect.size() - i - 1;
            // TODO this pow function might be wrong
            // inproper way of accessing vect index
            result += vect[i] * pow(x, power);  // a[n] * x ^ n
        }
        return result;
    }
    float polyDirvative(float x) {
        float result = 0;
        for (int i = 0; i < vect.size() - 1; i++) {
            int power = vect.size() - i - 2;
            // todo finish?
            result += vect[i] * (power + 1) * pow(x, power);  // a[n] * n * x ^ (n - 1)
        }
        return result;
    }
};

namespace backend {
std::vector<cv::Point2d> cameraPixelToGroundPos(std::vector<cv::Point2d>& pixels,
                                                image_geometry::PinholeCameraModel& rgb_info_sub);
nav_msgs::msg::Path cameraPixelToGroundPath(std::vector<cv::Point2d>& pixels,
                                            const image_geometry::PinholeCameraModel& rgb_info_sub, float camera_height,
                                            std::string frame_id);
std::optional<nav_msgs::msg::Path> create_path(std::vector<cv::Point2d>& left_contours,
                                               std::vector<cv::Point2d>& right_contours,
                                               image_geometry::PinholeCameraModel& camera_info, std::string frame_id);
}  // namespace backend
