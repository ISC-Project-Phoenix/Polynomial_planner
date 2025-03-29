#include "polynomial_planner/backend.hpp"

#include <algorithm>
#include <polynomial_planner/polyfit.hpp>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "image_geometry/pinhole_camera_model.h"

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2/LinearMath/Matrix3x3.h>

std::optional<nav_msgs::msg::Path> backend::create_path(std::vector<cv::Point2d>& left_contours,
                                                        std::vector<cv::Point2d>& right_contours,
                                                        image_geometry::PinholeCameraModel rgb_info_sub,
                                                        std::string_view frame_id) {
    // take in coutours
    // take in Polynomial
    // ros polynoial take in code...
    // transfer to Polynomial class;

    // takes in two arrays of x, y, and degree.pushback();
    // returns coefficients
    // polyfit::FitPolynomial();

    // std::string_view is a string lol
    std::vector<cv::Point2d> cam_path;  // this is the vector of path plannign points

    int width = rgb_info_sub.fullResolution().width;
    int height = rgb_info_sub.fullResolution().height;
    float max_left = 0;
    float max_right = 0;
    float min_left = height;
    float min_right = height;
    bool is_right_valid = true;
    bool is_left_valid = true;
    Polynomial rightPoly;
    Polynomial leftPoly;
    std::vector<cv::Point2d> left_contours_ground;
    std::vector<cv::Point2d> right_contours_ground;

    if ( left_contours.empty() ) {

    }
    if ( right_contours.empty() ){

    }

    // Loop through each point and convert to ground position
    if (is_left_valid) {
                    // convert element to ground and push back to ground array
                    left_contours_ground = cameraPixelToGroundPos(left_contours, rgb_info_sub);
        for (const auto& element : left_contours_ground) {
            if (element.y < min_left) min_left = element.y;
            if (element.y > max_left) max_left = element.y;
        }
        // run regression on ground contours
    }
    // Loop through each point and convert to ground position
    if (is_right_valid) {
            // convert element to ground and push back to ground array
            right_contours_ground = cameraPixelToGroundPos(right_contours, rgb_info_sub);
        for (const auto& element : right_contours_ground) {
            if (element.y < min_right) min_right = element.y;
            if (element.y > max_right) max_right = element.y;
        }
        // run regression on ground contours
    }
    // interval for polynomial
    float max = 480 - 480 * 0.40;    // artificial event horizon, 45
                                     // the x value in which path points are no longer allowed to cross.
    float interval = 3;              // stepping x value up by 3camera px on each iteration
    float start = 480 - 480 * 0.10;  // bottom of frame
    float threshold = 10.0;          // min dist between points (in pixels)
    float projection = 2.3;          // projection distance in kartspace

    float dist = 0;  // the value between the last published point and the current point
    for (int x = start; x > max; x -= interval) {
        dist += sqrt(interval * interval + pow(leftPoly.poly(x) - leftPoly.poly(x + interval), 2));

        if (dist > threshold) {
            // TODO figure out how to null value
            if (is_left_valid) {
                // do left poly math;
                // <y, -x>
                float dx = leftPoly.polyDirvative(x);
                // calulate dx were dy is 1
                // becuase I dont want to use a wrapper class
                float dy = 1;
                // set dy to 1
                float l = std::sqrt(dx * dx + dy * dy);
                // find the magnitude
                dx = dx / l;                     // normalize
                dy = dy / l;                     // normalize
                float p_x = x;                   // define x
                float P_y = leftPoly.poly(p_x);  // define y
                float camX = (p_x + projection * dy);     // project x
                float camY = (P_y - projection * dx);     // project y
                // TODO refractor with Camera Space pixels
                // OR do check if distance is greater than 8ish meters
                if (camY >= 240 && camY <= 480 && camX >= 0 && camX <= 640) {
                    cam_path.push_back(cv::Point2d(camX, camY));
                }
                // return vector as < y, -x >
            }
            // reset the distance counter
            dist = 0;
        }
    }
    for (int x = start; x > max; x -= interval) {
        dist += sqrt(interval * interval + pow(leftPoly.poly(x) - leftPoly.poly(x + interval), 2));

        if (dist > threshold) {
            // TODO figure out how to null value
            if (is_right_valid) {
                // do right poly math
                // <-y, x>
                // same steps different numbers
                float dx = rightPoly.polyDirvative(x);
                float dy = 1;
                float l = std::sqrt(dx * dx + dy * dy);
                l = std::hypot(dx, dy);
                dx = dx / l;
                dy = dy / l;
                float p_x = x;
                float p_y = leftPoly.poly(x);
                float camX = (p_x - 7 * dy);
                float camY = (p_y + 7 * dx);
                // TODO refractor with Camera Space pixels
                // OR do check if distance is greater than 8ish meters
                if (camY >= 240 && camY <= 480 && camX >= 0 && camX <= 640) {
                    cam_path.push_back(cv::Point2d(camX, camY));
                }
                // return vector as < -y , x >
            }
            // reset the distance counter
            dist = 0;
        }
    }
    // Debug looper
    for (int i = 55; i < 200; i++) {
        float camX = 640;
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
    optical_to_ros.setRPY(0.0, 0.0, 0.0);
    // optical_to_ros.setRPY(0.0, 0.0, -M_PI / 2);
    optical_to_ros.setRPY(-M_PI / 2, 0.0, -M_PI / 2);

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