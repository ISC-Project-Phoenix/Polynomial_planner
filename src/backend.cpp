#include "polynomial_planner/backend.hpp"

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>

#include <algorithm>
#include <cmath>
#include <polynomial_planner/polyfit.hpp>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "image_geometry/pinhole_camera_model.h"

std::optional<nav_msgs::msg::Path> backend::create_path(std::vector<cv::Point2d>& left_contours,
                                                        std::vector<cv::Point2d>& right_contours,
                                                        image_geometry::PinholeCameraModel rgb_info_sub,
                                                        std::string_view frame_id) {
    // take in contours
    // DO NOT THE Polynomials
    // Match the pointd...
    // shortest side first larger side second. 
    // translate to normal space. 

    // takes in two arrays of x, y, and degree.pushback();
    // returns coefficients
    // polyfit::FitPolynomial();

    // std::string_view is a string lol
    std::vector<cv::Point2d> ground_path;   // this is the vector of path plannign points in cart space
    std::vector<cv::Point2d> cam_path;      // this is the vector of path plannign points in camera space 
    ground_path.emplace_back(cv::Point2d(0, 0));

    int width = rgb_info_sub.fullResolution().width;    // camera space sizes!
    int height = rgb_info_sub.fullResolution().height;  // Camera space sizes!

    bool is_right_valid = true;  // stores if Polynomial was intizatized!
    bool is_left_valid = true;   // left and right respectively


    if (left_contours.empty()) {
        // for any and all checks regarding data cleaning!
        // is_left_valid = false;
        return std::nullopt;
    }
    if (right_contours.empty()) {
        // is_right_valid = false;
        return std::nullopt;
    }
    std::vector<cv::Point2d> bigger_array;
    std::vector<cv::Point2d> smaller_array;

    bool is_left_bigger = left_contours.size() > right_contours.size();

    if (is_left_bigger) {
        bigger_array = left_contours;
        smaller_array = right_contours;
    } else {
        smaller_array = left_contours;
        bigger_array = right_contours;
    }

    if (is_left_bigger) {
        bigger_array = left_contours;
        smaller_array = right_contours;
    } else {
        bigger_array = right_contours;
        smaller_array = left_contours;
    }
    
    for (int i = 0; i < smaller_array.size(); i++) {
        float old_dist = 1000000;
        int lucky_index = -1;
    
        for (int j = 0; j < bigger_array.size(); j++) {
            double dx = smaller_array[i].x - bigger_array[j].x;
            double dy = smaller_array[i].y - bigger_array[j].y;
            double new_dist = std::sqrt(dx * dx + dy * dy);
            if (new_dist < old_dist) {
                old_dist = new_dist;
                lucky_index = j;
            }
        }
    
        if (lucky_index >= 0) {
            double x = (bigger_array[lucky_index].x + smaller_array[i].x) / 2;
            double y = (bigger_array[lucky_index].y + smaller_array[i].y) / 2;
    
            cam_path.push_back(cv::Point2d(x, y));
    
            bigger_array.erase(bigger_array.begin() + lucky_index);
        }
    }
    
    ground_path = cameraPixelToGroundPos(cam_path, rgb_info_sub);
    if (ground_path.empty()) {
        return std::nullopt;
    } else {
        // Convert from cv types to nav::msg

        // convert from camera to ground
        // done further ahead already
        // std::vector<cv::Point2d> ground_path = cameraPixelToGroundPos(cam_path, rgb_info_sub);

        nav_msgs::msg::Path msg{};
        // msg.header.frame_id = frame;
        // for (cv::Point2d ground_points : ground_path) {
        // std::
        //  }
        // converting <x,y> to message type in ROS
        std::transform(ground_path.begin(), ground_path.end(), std::back_inserter(msg.poses),
                       [&frame_id](const cv::Point2d& point) {
                           geometry_msgs::msg::PoseStamped pose{};
                           // frame = "redto0 isn't sure if we use this";
                           // redto0 is SURE that we use this update and fix ASAP
                           pose.header.frame_id = frame_id;  // literally is "notaemptystring"
                           pose.pose.position.x = point.x;
                           pose.pose.position.y = point.y;
                           // pose.pose.position.z = point.z;

                           return pose;
                       });

        return msg;
    }
}
// why are the pointer things the way they are
// TODO: make it not die when z is too small
//       or make z not too small
std::vector<cv::Point2d> backend::cameraPixelToGroundPos(std::vector<cv::Point2d>& pixels,
                                                         image_geometry::PinholeCameraModel rgb_info_sub) {
    // Rotation that rotates left 90 and backwards 90.
    // This converts from camera coordinates in OpenCV to ROS coordinates
    tf2::Quaternion optical_to_ros{};
    // set the Roll Pitch YAW
    /// optical_to_ros.setRPY(0.0, 0.0, 0.0);
    optical_to_ros.setRPY(-M_PI / 2, 0.0, -M_PI / 2);

    std::vector<cv::Point2d> rwpoints;
    const double camera_height = 0.6;  // meters

    for (cv::Point2d& pixel : pixels) {
        // gotta rectify the pixel before we raycast
        // pixel.y += 120;
        // pixel.x += 320;
        // cv::Point2d rectPixel = rgb_info_sub.rectifyPoint(pixel);
        cv::Point3d ray = rgb_info_sub.projectPixelTo3dRay(pixel);

        // safety check
        if (fabs(ray.y) < 1e-6) {  // Near zero check
            // RCLCPP_WARN(rclcpp::get_logger("backend"), "Invalid ray projection (y near zero)");
            continue;  // Skip this point
        }

        // -- CAMERA COORDINATES --
        //      positive x = +X TO CAMERA
        //      positive y = STRAIGHT TO GROUND
        //      positive z = OUT OF CAMERA
        //      hopefully

        // ask zach for the trig, extend ray to the floor.
        double divisor = ray.y / camera_height;  // divide by how high off the ground the camera is!
        ray.x = ray.x / divisor;
        ray.y = ray.y / divisor;
        ray.z = ray.z / divisor;
        // ray /= ray.z / -0.6;
        // ray.z = (ray.z * -1);  // we don't really care abt z, since it -will- *should* always just be cameraHeight
        // tf2::Vector3 tf_vec{ray.z, -ray.x, -ray.y};
        tf2::Vector3 tf_vec{ray.x, ray.y, ray.z};
        tf_vec = tf2::quatRotate(optical_to_ros, tf_vec);

        //return type world_vec, use this is

        cv::Point2d dvector(tf_vec.x(), tf_vec.y());

        // push back vectors
        rwpoints.push_back(dvector);
    }

    return rwpoints;
}