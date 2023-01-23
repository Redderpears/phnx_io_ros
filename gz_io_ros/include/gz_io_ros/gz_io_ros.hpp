#pragma once

#include "optional"
#include <rclcpp/rclcpp.hpp> 
#include "nav_msgs/msg/odometry.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"

namespace gir{

class GzIoRos : public rclcpp::Node{
    public:
        explicit GzIoRos(rclcpp::NodeOptions options);
    private:
        std::optional<std::shared_ptr<rclcpp::Subscription<nav_msgs::msg::Odometry>>> _odom_sub =
          std::nullopt;

        std::optional<std::shared_ptr<rclcpp::Subscription<geometry_msgs::msg::TwistStamped>>> _cmd_vel_sub =
          std::nullopt;
          
        std::optional<std::shared_ptr<rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>>>
          _odom_acks_pub = std::nullopt;

        double _max_throttle_speed{};
        double _max_braking_speed{};

        void odom_cb(nav_msgs::msg::Odometry::SharedPtr msg);
        void cmd_vel_cb(geometry_msgs::msg::TwistStamped::SharedPtr msg);
};

}