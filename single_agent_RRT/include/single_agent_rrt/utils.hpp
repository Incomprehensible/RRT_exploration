#ifndef UTILS_H
#define UTILS_H

#include <memory>
#include <cmath>
#include <float.h>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace utils {
    inline double euclidian_dist(const geometry_msgs::msg::Point& p1, const geometry_msgs::msg::Point& p2)
    {
        return sqrt(pow(p1.x - p2.x, 2) + pow(p1.y - p2.y, 2));
    }

    inline bool comparable(double a, double b)
    {
        return std::fabs(a - b) <= DBL_EPSILON;
    }

    geometry_msgs::msg::TransformStamped::SharedPtr get_robot_position(tf2::BufferCore* tf_buffer)
    {
        geometry_msgs::msg::TransformStamped robot2map;

        try {
            robot2map = tf_buffer->lookupTransform("map", "base_footprint", tf2::TimePointZero);
        } catch (tf2::TransformException& e) {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Map to robot transform not found: %s", e.what());
            return nullptr;
        }
        return std::make_shared<geometry_msgs::msg::TransformStamped>(std::move(robot2map));
    }
}

#endif