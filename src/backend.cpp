#include "polynomial_planner/backend.hpp"

#include <algorithm>
#include <geometry_msgs/msg/detail/pose_stamped__struct.hpp>
#include <memory>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "image_geometry/pinhole_camera_model.h"

std::optional<nav_msgs::msg::Path> backend::create_path(std::vector<cv::Point2d>& left_contours,
                                                        std::vector<cv::Point2d>& right_contours,
                                                        image_geometry::PinholeCameraModel& camera_info,
                                                        std::string frame_id) {
    // take in contours
    // DO NOT THE Polynomials
    // Match the pointd...
    // shortest side first larger side second.
    // returns coefficients
    // polyfit::FitPolynomial();

    // std::string_view is a string lol
    std::vector<cv::Point2d> ground_path;  // this is the vector of path plannign points in cart space
    std::vector<cv::Point2d> cam_path;     // this is the vector of path plannign points in camera space
    ground_path.emplace_back(cv::Point2d(0, 0));

    int width = camera_info.fullResolution().width;    // camera space sizes!
    int height = camera_info.fullResolution().height;  // Camera space sizes!

    bool is_right_valid = true;  // stores if Polynomial was intizatized!
    bool is_left_valid = true;   // left and right respectively

    if (left_contours.empty()) {
        // for any and all checks regarding data cleaning!
        // is_left_valid = false;
        return std::nullopt;
    }

    std::vector<cv::Point2d> bigger_array;
    std::vector<cv::Point2d> smaller_array;

    bigger_array = backend::cameraPixelToGroundPos(left_contours, camera_info);

    for (int i = 0; i < bigger_array.size(); i++) {
        double x = bigger_array[i].x;
        double y = bigger_array[i].y - 1.80;

        cam_path.push_back(cv::Point2d(x, y));
    }

    if (cam_path.empty()) {
        return std::nullopt;
    } else {
        // Convert from cv types to nav::msg
        // too few args
        // TODO use tf2 to fnd the hieght
        // auto ground_points = backend::cameraPixelToGroundPos(cam_path, camera_info, 0.6, frame_id);
        nav_msgs::msg::Path msg{};
        std::transform(cam_path.begin(), cam_path.end(), std::back_inserter(msg.poses),
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

nav_msgs::msg::Path backend::cameraPixelToGroundPath(std::vector<cv::Point2d>& path_points,
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