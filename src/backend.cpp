#include "polynomial_planner/backend.hpp"

#include <algorithm>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "image_geometry/pinhole_camera_model.h"

std::optional<nav_msgs::msg::Path> backend::create_path(std::vector<float>& leftPolyVector,
                                                        image_geometry::PinholeCameraModel rgb_info_sub,
                                                        std::string_view frame) {
    // std::string_view is a string lol
    std::vector<cv::Point2d> cam_path;  // this is the vector of path plannign points

    // take in Polynomial
    // ros polynoial take in code...
    // transfer to Polynomial class;
    // float is_right_valid = false;
    // float is_left_valid = false;

    auto leftPoly = new Polynomial(leftPolyVector);

    // interval for polynomial
    float max = 480 - 480 *0.45;          // artificial event horizon,
                             // the x value in which path points are no longer allowed to cross.
    float interval = 3;      // stepping x value up by 3camera px on each iteration
    float start = 480;       // bottom of frame
    float threshold = 10.0;  // min dist between points

    float dist = 0;  // the value between the last published point and the current point
    for (int x = start; x > max; x -= interval) {
        dist += sqrt(interval * interval + pow(leftPoly->poly(x) - leftPoly->poly(x + interval), 2));

        if (dist > threshold) {
            int translate = 0.0;
            // translate =0;
            float camX = leftPoly->poly(x + translate);
            float camY = x + translate;
            if (camY >= 240 && camY <= 480 && camX >= 0 && camX <= 640) {
                // CV point should be x,y
                // the poly is P(y)
                // translate should apply to the Y values?
                cam_path.push_back(cv::Point2d(camX, camY));
            }
            dist = 0;
        }
    }

    for (int i = 55; i < 200; i++) {
        float camX = 320;
        float camY = (i) + 240;
        if (camY >= 240 && camY <= 480 && camX >= 0 && camX <= 640) {
            // cam_path.push_back(cv::Point2d(camX, camY));
        }
    }

    if (cam_path.empty()) {
        return std::nullopt;
    } else {
        // Convert from cv types to nav::msg
        // too few args
        std::vector<cv::Point2d> ground_path = cameraPixelToGroundPos(cam_path, rgb_info_sub);

        nav_msgs::msg::Path msg{};
        // msg.header.frame_id = frame;
        // for (cv::Point2d ground_points : ground_path) {
        // std::
        //  }
        // converting <x,y> to message type in ROS
        std::transform(ground_path.begin(), ground_path.end(), std::back_inserter(msg.poses),
                       [&frame](const cv::Point2d& point) {
                           geometry_msgs::msg::PoseStamped pose{};
                           // frame = "redto0 isn't sure if we use this";
                           // redto0 is SURE that we use this update and fix ASAP
                           pose.header.frame_id = frame;  // literally is "notaemptystring"
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
    // optical_to_ros.setRPY(0.0, 0.0, -M_PI / 2);
    // optical_to_ros.setRPY(-M_PI / 2, 0.0, -M_PI / 2);
    // optical_to_ros.setRPY(0.0, 0.0, 0.0);

    std::vector<cv::Point2d> rwpoints;

    for (cv::Point2d& pixel : pixels) {
        // gotta rectify the pixel before we raycast
        pixel.y += 120;
        pixel.x += 320;
        // cv::Point2d rectPixel = rgb_info_sub.rectifyPoint(pixel);
        cv::Point3d ray = rgb_info_sub.projectPixelTo3dRay(pixel);

        // -- CAMERA COORDINATES --
        //      positive x = +X TO CAMERA
        //      positive y = STRAIGHT TO GROUND
        //      positive z = OUT OF CAMERA
        //      hopefully

        // ask zach for the trig, extend ray to the floor.
        float divisor = ray.y / 0.6;
        ray.x = ray.x / divisor;
        ray.y = ray.y / divisor;
        ray.z = ray.z / divisor;
        // ray /= ray.z / -0.6;
        // ray.z = (ray.z * -1);  // we don't really care abt z, since it -will- *should* always just be cameraHeight
        tf2::Vector3 tf_vec{ray.z, -ray.x, -ray.y};
        //tf2::Vector3 tf_vec{ray.x, ray.y, ray.z};
        //tf2::Vector3 world_vec = tf2::quatRotate(optical_to_ros, tf_vec);

        //return type world_vec, use this is

        cv::Point2d dvector(tf_vec.x(), tf_vec.y());

        // push back vectors
        rwpoints.push_back(dvector);
    }

    return rwpoints;
}
