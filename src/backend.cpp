#include "polynomial_planner/backend.hpp"

#include <algorithm>
#include <geometry_msgs/msg/detail/pose_stamped__struct.hpp>
#include <memory>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "image_geometry/pinhole_camera_model.h"

std::optional<nav_msgs::msg::Path> backend::create_path(std::vector<cv::Point2d>& left_contours,
                                                        std::vector<cv::Point2d>& right_contours,
                                                        std::vector<float> center_poly,
                                                        image_geometry::PinholeCameraModel& camera_info,
                                                        std::string frame_id) {
    // take in contours
    // DO NOT THE Polynomials
    // Match the pointd...
    // shortest side first larger side second.
    // returns coefficients
    // polyfit::FitPolynomial();

    // std::string_view is a string lol
    nav_msgs::msg::Path ground_points;     // this is the vector of path plannign points in camera space

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
    nav_msgs::msg::Path bigger_array;
    nav_msgs::msg::Path smaller_array;

    bool is_left_bigger = left_contours.size() > right_contours.size();

    if (is_left_bigger) {
        bigger_array = backend::cameraPixelToGroundPos(left_contours, camera_info, 0.6, frame_id);
        smaller_array = backend::cameraPixelToGroundPos(right_contours, camera_info, 0.6, frame_id);
    } else {
        smaller_array = backend::cameraPixelToGroundPos(left_contours, camera_info, 0.6, frame_id);
        bigger_array = backend::cameraPixelToGroundPos(right_contours, camera_info, 0.6, frame_id);
    }

    for (int i = 0; i < smaller_array.poses.size(); i++) {
        float old_dist = 1000000;
        int lucky_index = -1;

        for (int j = 0; j < bigger_array.poses.size(); j++) {
            double dx = smaller_array.poses[i].pose.position.x - bigger_array.poses[j].pose.position.x;
            double dy = smaller_array.poses[i].pose.position.y - bigger_array.poses[j].pose.position.y;
            double new_dist = std::sqrt(dx * dx + dy * dy);
            if (new_dist < old_dist) {
                old_dist = new_dist;
                lucky_index = j;
            }
        }

        if (lucky_index >= 0) {
            double x = (bigger_array.poses[lucky_index].pose.position.x + smaller_array.poses[i].pose.position.x) / 2;
            double y = (bigger_array.poses[lucky_index].pose.position.y + smaller_array.poses[i].pose.position.y) / 2;

            geometry_msgs::msg::PoseStamped p{};
            // TODO solve why -world_vec.x needs to be invereted and let andy know why pls
            p.pose.position.x = x;
            p.pose.position.y = y;
            p.pose.position.z = 0;
            p.header.frame_id = frame_id;
    
            ground_points.poses.push_back(p);

            bigger_array.poses.erase(bigger_array.poses.begin() + lucky_index);
        }
    }

    if (ground_points.poses.empty() && ground_points.poses.size() < 1000) {
        return std::nullopt;
    } else {
        // Convert from cv types to nav::msg
        // too few args
        // TODO use tf2 to fnd the hieght
        return ground_points;
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
nav_msgs::msg::Path backend::cameraPixelToGroundPos(std::vector<cv::Point2d>& path_points,
                                                    const image_geometry::PinholeCameraModel& camera_info,
                                                    float camera_height, std::string frame_id) {
    // Rotation that rotates left 90 and backwards 90.
    // This converts from camera coordinates in OpenCV to ROS coordinates
    tf2::Quaternion optical_to_ros{};
    optical_to_ros.setRPY(-M_PI / 2, 0.0, -M_PI / 2);
    // -- CAMERA COORDINATES --
    //      positive x = +X TO CAMERA
    //      positive y = STRAIGHT TO GROUND
    //      positive z = OUT OF CAMERA
    //      hopefully
    nav_msgs::msg::Path rwpoints{};

    auto ray_point = cv::Vec3f{0, camera_height, 0};

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
        auto world_vec = tf2::quatRotate(optical_to_ros, tf_vec);

        geometry_msgs::msg::PoseStamped p{};
        // TODO solve why -world_vec.x needs to be invereted and let andy know why pls
        p.pose.position.x = -world_vec.x();
        p.pose.position.y = -world_vec.y();
        p.pose.position.z = world_vec.z();
        p.header.frame_id = frame_id;

        rwpoints.poses.push_back(p);
    }
    rwpoints.header.frame_id = frame_id;

    return rwpoints;
}