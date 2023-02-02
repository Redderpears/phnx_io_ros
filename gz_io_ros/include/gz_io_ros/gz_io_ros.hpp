#pragma once

#include "optional"
#include <rclcpp/rclcpp.hpp>
#include "nav_msgs/msg/odometry.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "message_filters/sync_policies/approximate_time.h"

namespace gir {
    typedef message_filters::sync_policies::ApproximateTime<nav_msgs::msg::Odometry, geometry_msgs::msg::TwistStamped> sync_policy;

    class GzIoRos : public rclcpp::Node {
    public:
        explicit GzIoRos(rclcpp::NodeOptions options);

    private:
        std::optional<std::shared_ptr<rclcpp::Publisher<ackermann_msgs::msg::AckermannDrive>>>
                _odom_acks_pub = std::nullopt;

        double _max_throttle_speed{};
        double _max_braking_speed{};
        double _wheelbase{};

        void convert_data(nav_msgs::msg::Odometry::ConstSharedPtr odom,
                          geometry_msgs::msg::TwistStamped::ConstSharedPtr cmd_vel);
        double convert_trans_rot_vel_to_steering_angle(double vel, double omega, double wheelbase);


    };

}