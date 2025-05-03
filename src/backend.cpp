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
                                                        image_geometry::PinholeCameraModel& camera_info,
                                                        std::string_view frame_id) {
    // take in contours
    // create your own polynomials
    // project from the points using the polynmials
    // translate to normal space.

    // takes in two arrays of x, y, and degree.pushback();
    // returns coefficients
    // polyfit::FitPolynomial();

    // std::string_view is a string lol
    std::vector<cv::Point2d> ground_path;  // this is the vector of path plannign points in cart space
    std::vector<cv::Point2d> cam_path;     // this is the vector of path plannign points in camera space
    // ground_path.emplace_back(cv::Point2d(0, 0));

    int width = camera_info.fullResolution().width;    // camera space sizes!
    int height = camera_info.fullResolution().height;  // Camera space sizes!

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
        bigger_array = cameraPixelToGroundPos(left_contours, camera_info);
        smaller_array = cameraPixelToGroundPos(right_contours, camera_info);
    } else {
        smaller_array = cameraPixelToGroundPos(left_contours, camera_info);
        bigger_array = cameraPixelToGroundPos(right_contours, camera_info);
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

            ground_path.push_back(cv::Point2d(x, y));

            bigger_array.erase(bigger_array.begin() + lucky_index);
        }
    }

    if (ground_path.empty()) {
        return std::nullopt;
    } else {
        // std::vector<cv::Point2d> ground_path = cameraPixelToGroundPos(cam_path, camera_info);

        nav_msgs::msg::Path msg{};
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

cv::Vec3f intersectPoint(cv::Vec3f rayVector, cv::Vec3f rayPoint, cv::Vec3f planeNormal, cv::Vec3f planePoint) {
    cv::Vec3f diff = rayPoint - planePoint;
    double prod1 = diff.dot(planeNormal);
    double prod2 = rayVector.dot(planeNormal);
    double prod3 = prod1 / prod2;
    return rayPoint - rayVector * prod3;
}

// TODO: make it not die when z is too small
//       or make z not too small
std::vector<cv::Point2d> backend::cameraPixelToGroundPos(std::vector<cv::Point2d>& path_points,
                                                         image_geometry::PinholeCameraModel& camera_info) {
    // Rotation that rotates left 90 and backwards 90.
    // This converts from camera coordinates in OpenCV to ROS coordinates
    tf2::Quaternion optical_to_ros{};
    optical_to_ros.setRPY(-M_PI / 2, 0.0, -M_PI / 2);
    // -- CAMERA COORDINATES --
    //      positive x = +X TO CAMERA
    //      positive y = STRAIGHT TO GROUND
    //      positive z = OUT OF CAMERA
    //      hopefully
    std::vector<cv::Point2d> rwpoints;

    auto ray_point = cv::Vec3f{0, 0.527, 0};

    for (cv::Point2d& pixel : path_points) {
        if (!camera_info.initialized()) {
            continue;
        }

        // Project pixel to camera space
        auto ray = camera_info.projectPixelTo3dRay(pixel);
        auto ray_vect = cv::Vec3f(ray.x, ray.y, ray.z);
        // The oak-d uses shorts in mm, sim uses f32 in m
        auto point_3d = intersectPoint(ray_vect, ray_point, cv::Vec3f(0, -1, 0),
                                       cv::Vec3f(0, 0, 0));  // returned form ray plane function

        // Convert from camera space to ros coordinates ("World" but wrt camera mount)
        tf2::Vector3 tf_vec{point_3d[0], point_3d[1], point_3d[2]};
        tf2::Vector3 world_vec = tf2::quatRotate(optical_to_ros, tf_vec);

        cv::Point2d dvector(-world_vec.x(), -world_vec.y());

        rwpoints.push_back(dvector);
    }

    return rwpoints;
}
