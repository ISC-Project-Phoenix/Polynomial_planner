#include "polynomial_planner/backend.hpp"

#include <algorithm>
#include <geometry_msgs/msg/detail/pose_stamped__struct.hpp>
#include <memory>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "image_geometry/pinhole_camera_model.h"

std::optional<nav_msgs::msg::Path> backend::create_path(std::vector<float>& leftPolyVector,
                                                        const image_geometry::PinholeCameraModel& camera_info,
                                                        const std::string_view& frame) {
    // std::string_view is a string lol
    std::vector<cv::Point2d> cam_path;  // this is the vector of path plannign points

    int width = camera_info.fullResolution().width;    // camera space sizes!
    int height = camera_info.fullResolution().height;  // Camera space sizes!

    auto leftPoly = std::make_unique<Polynomial>(leftPolyVector);

    // interval for polynomial
    float max = height - height * .40;     // artificial event horizon,
                                           // the x value in which path points are no longer allowed to cross.
    float interval = 3;                    // stepping x value up by 3camera px on each iteration
    float start = height - height * 0.20;  // bottom of frame
    float threshold = 10.0;                // min dist between points

    float dist = 0;  // the value between the last published point and the current point
    for (int x = start; x > max; x -= interval) {
        dist += sqrt(interval * interval + pow(leftPoly->poly(x) - leftPoly->poly(x + interval), 2));
        // TODO distance

        if (dist > threshold) {
            int translate = height - height * 0.45;
            translate = 0;
            float camX = leftPoly->poly(x);
            float camY = x + translate;

            cam_path.push_back(cv::Point2d(camX, camY));
            dist = 0;
        }
    }

    if (cam_path.empty()) {
        return std::nullopt;
    } else {
        // Convert from cv types to nav::msg
        // too few args
        // TODO use tf2 to fnd the hieght
        return cameraPixelToGroundPos(cam_path, camera_info, 0.6, frame);
    }
}

cv::Vec3f intersectPoint(cv::Vec3f rayVector, cv::Vec3f rayPoint, cv::Vec3f planeNormal, cv::Vec3f planePoint) {
    cv::Vec3f diff = rayPoint - planePoint;
    double prod1 = diff.dot(planeNormal);
    double prod2 = rayVector.dot(planeNormal);
    double prod3 = prod1 / prod2;
    return rayPoint - rayVector * prod3;
}

// why are the pointer things the way they are
// TODO: make it not die when z is too small
//       or make z not too small
nav_msgs::msg::Path backend::cameraPixelToGroundPos(std::vector<cv::Point2d>& path_points,
                                                    const image_geometry::PinholeCameraModel& camera_info,
                                                    float camera_height, const std::string_view& frame) {
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
        p.header.frame_id = frame;

        rwpoints.poses.push_back(p);
    }
    rwpoints.header.frame_id = frame;

    return rwpoints;
}