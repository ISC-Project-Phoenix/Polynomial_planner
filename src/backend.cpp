#include "polynomial_planner/backend.hpp"

#include <algorithm>

#include "geometry_msgs/msg/pose_stamped.hpp"

std::optional<nav_msgs::msg::Path> backend::create_path(std::vector<float>& leftPolyVector, const sensor_msgs::msg::CameraInfo& rgb_info_sub,std::string frame) {
    // std::string_view is a string lol
    std::vector<cv::Point2d> cam_path;  // this is the vector of path plannign points

    // take in Polynomial
    // ros polynoial take in code...
    // transfer to Polynomial class;
    // float is_right_valid = false;
    // float is_left_valid = false;

    auto leftPoly = new Polynomial(leftPolyVector);

    // interval for polynomial
    float max = 280;         // artificial event horizon
    float interval = 3;      // stepping x value up by 3camera px on each iteration
    float start = 475;       // bottom of frame
    float threshold = 15.0;  // min dist between points

    float dist = 0;
    for (int x = start; x > max; x -= interval) {
        dist += sqrt(interval * interval + pow(leftPoly->poly(x) - leftPoly->poly(x + interval), 2));

        if (dist > threshold) {
            cam_path.push_back(cv::Point2d(x, leftPoly->poly(x)));
            dist = 0;
        }
    }

    if (cam_path.empty()) {
        return std::nullopt;
    } else {
        // Convert from cv types to nav::msg
        // too few args
        std::vector<cv::Point2d> ground_path = cameraPixelToGroundPos(cam_path, rgb_info_sub.cameraInfo());

        nav_msgs::msg::Path msg{};
        // msg.header.frame_id = frame;
        for (cv::Point2d ground_points : ground_path) {
            // std::
        }
        // converting <x,y> to message type in ROS
        std::transform(ground_path.begin(), ground_path.end(), std::back_inserter(msg.poses),
                       [frame](const cv::Point2d& point) {
                           geometry_msgs::msg::PoseStamped pose{};
                           // frame = "redto0 isn't sure if we use this";
                           pose.header.frame_id = frame;
                           pose.pose.position.x = point.x;
                           pose.pose.position.y = point.y;

                           return pose;
                       });

        return msg;
    }
}

// why are the pointer things the way they are
// TODO: make it not die when z is too smallf
//       or make z not too small
std::vector<cv::Point2d> cameraPixelToGroundPos(std::vector<cv::Point2d>& pixels,
                                                const sensor_msgs::msg::CameraInfo& rgb_info_sub) {
    // Rotation that rotates left 90 and backwards 90.
    // This converts from camera coordinates in OpenCV to ROS coordinates
    tf2::Quaternion optical_to_ros{};
    // set the Roll Pitch YAW
    optical_to_ros.setRPY(-M_PI / 2, 0.0, -M_PI / 2);

    std::vector<cv::Point2d> rwpoints;

    for (const cv::Point2d& pixel : pixels) {
        // gotta rectify the pixel before we raycast
        cv::Point2d rectPixel = rgb_info_sub->rgb_model.rectifyPoint(pixel);
        cv::Point3d ray = rgb_info_sub->rgb_model.projectPixelTo3dRay(rectPixel);

        // ask zach for the trig, extend ray to the floor.
        float divisor = ray.z / -0.6;
        ray.x = ray.x / divisor;
        ray.y = ray.y / divisor;
        ray.z = ray.z / divisor;
        // ray /= ray.z / -0.6;
        // ray.setZ(ray.getZ() * -1); we don't really care abt z, since it -will- *should* always just be cameraHeight
        tf2::Vector3 tf_vec{ray.x, ray.y, ray.z};
        tf2::Vector3 world_vec = tf2::quatRotate(optical_to_ros, tf_vec);

        //return type world_vec, use this is

        cv::Point2d dvector(world_vec.x(), world_vec.y());

        // push back vectors
        rwpoints.push_back(dvector);
    }

    return rwpoints;
}
