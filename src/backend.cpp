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
    // take in Polynomial
    // ros polynoial take in code...
    // transfer to Polynomial class;

    // takes in two arrays of x, y, and degree.pushback();
    // returns coefficients
    // polyfit::FitPolynomial();

    // std::string_view is a string lol
    std::vector<cv::Point2d> ground_path;  // this is the vector of path plannign points

    int width = rgb_info_sub.fullResolution().width;    // camera space sizes!
    int height = rgb_info_sub.fullResolution().height;  // Camera space sizes!

    int global_degree = 2;  // the global degree for all polynomial regession
    double max_left = 0;
    double max_right = 0;
    double min_left = height;
    double min_right = height;
    bool is_right_valid = true;  // stores if Polynomial was intizatized!
    bool is_left_valid = true;   // left and right respectively
    Polynomial rightPoly;        // sets the scope to global for the function
    Polynomial leftPoly;
    std::vector<cv::Point2d> left_contours_ground;   // stores the translated points from
    std::vector<cv::Point2d> right_contours_ground;  // camera space to ground space

    if (left_contours.empty()) {
        // for any and all checks regarding data cleaning!
        // is_left_valid = false;
    }
    if (right_contours.empty()) {
        // is_right_valid = false;
    }

    // Loop through each point and convert to ground position
    if (is_left_valid) {
        // convert element to ground and push back to ground array
        left_contours_ground = cameraPixelToGroundPos(left_contours, rgb_info_sub);
        std::vector<double> x;  // array for holding x
        std::vector<double> y;  //        and y values
        for (const auto& element : left_contours_ground) {
            if (element.y < min_left) min_left = element.y;
            if (element.y > max_left) max_left = element.y;
            x.push_back(element.x);
            y.push_back(element.y);
        }
        // run regression on ground contours
        leftPoly = Polynomial{polyfit::FitPolynomial(y, x, global_degree)};
    }
    // Loop through each point and convert to ground position
    if (is_right_valid) {
        // convert element to ground and push back to ground array
        right_contours_ground = cameraPixelToGroundPos(right_contours, rgb_info_sub);
        std::vector<double> x;  // array for holding x
        std::vector<double> y;  //        and y values
        for (const auto& element : right_contours_ground) {
            if (element.y < min_right) min_right = element.y;
            if (element.y > max_right) max_right = element.y;
            x.push_back(element.x);
            y.push_back(element.y);
        }
        // run regression on ground contours
        rightPoly = Polynomial{polyfit::FitPolynomial(y, x, global_degree)};
    }
    // interval for polynomial
    // TODO REPLACE ALL CASES OF 480 and 640 with there respective camera spcae coordinates
    double max = 480 - 480 * 0.40;    // artificial event horizon, 45
                                      // the x value in which path points are no longer allowed to cross.
    double interval = 0.3;            // stepping x value up by 3camera px on each iteration
    double start = 480 - 480 * 0.10;  // bottom of frame
    double threshold = 0.50;          // min dist between points (in pixels)
    double projection = 2.1336;       // projection distance in kartspace
    projection = 0;
    double dist = 0;  // the value between the last published point and the current point

    for (int i = 0; i < left_contours_ground.size(); i += 5) {
        // TODO figure out how to null value
        float x = left_contours_ground[i].y;
        if (is_left_valid) {
            // do left poly math;
            // <y, -x>
            double dy = leftPoly.polyDerivative(x);
            // calulate dx were dy is 1
            // becuase I dont want to use a wrapper class
            double dx = 1;
            // set dy to 1
            double l = std::sqrt(dx * dx + dy * dy);
            // find the magnitude
            dx = dx / l;                            // normalize
            dy = dy / l;                            // normalize
            double p_x = x;                         // define x
            double P_y = leftPoly.poly(p_x);        // define y
            double camY = (p_x + projection * dy);  // project x
            double camX = (P_y - projection * dx);  // project y
                                                    // TODO refractor with Camera Space pixels
                                                    // OR do check if distance is greater than 8ish meters
                                                    // if (camY >= 240 && camY <= 480 && camX >= 0 && camX <= 640) {
            if (std::isfinite(camX) && std::isfinite(camY) ) {
                // && camX > 0.0  && camX < 12.0 && camY > -8 && camY < 8) {
                ground_path.push_back(cv::Point2d(camX, camY));
            } else {
                // RCLCPP_WARN(rclcpp::get_logger("backend"), "Invalid path point detected");
            }
            // }
            // return vector as < y, -x >
        }
        // reset the distance counter
        i += 5;
    }

    for (int i = 0; i < left_contours_ground.size(); i += 5) {
        // TODO figure out how to null value
        float x = right_contours_ground[i].y;
        // TODO figure out how to null value
        if (is_right_valid) {
            // do right poly math
            // <-y, x>
            // same steps different numbers
            double dy = rightPoly.polyDerivative(x);
            double dx = 1;
            double l = std::sqrt(dx * dx + dy * dy);
            l = std::hypot(dx, dy);
            dx = dx / l;
            dy = dy / l;
            double p_x = x;
            double p_y = rightPoly.poly(x);
            double camY = (p_x - projection * dy);
            double camX = (p_y + projection * dx);
            // TODO refractor with Camera Space pixels
            // OR do check if distance is greater than 8ish meters
            // if (camY >= 240 && camY <= 480 && camX >= 0 && camX <= 640) {
            if (std::isfinite(camX) && std::isfinite(camY) ) {
                // && camX > 0.0  && camX < 12.0 && camY > -8 && camY < 8) {
                ground_path.push_back(cv::Point2d(camX, camY));
            } else {
                // RCLCPP_WARN(rclcpp::get_logger("backend"), "Invalid path point detected");
            }
            // }
            // return vector as < -y , x >
        }
        // reset the distance counter
        i += 5;
    }

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
    // optical_to_ros.setRPY(0.0, 0.0, -M_PI / 2);
    optical_to_ros.setRPY(-M_PI / 2, 0.0, -M_PI / 2);

    std::vector<cv::Point2d> rwpoints;

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
        double divisor = ray.y / 0.6;  // divide by how high off the ground the camera is!
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