#include <functional>
#include <rclcpp/rclcpp.hpp>
#include "message_filters/subscriber.h"
#include "gz_io_ros/gz_io_ros.hpp"

using namespace message_filters;

gir::GzIoRos::GzIoRos(rclcpp::NodeOptions options)
        : Node("gz_io_ros", options) {

    _max_throttle_speed = this->declare_parameter("max_throttle_speed", 1.0);
    _max_braking_speed = this->declare_parameter("max_brake_speed", 1.0);

    Subscriber<nav_msgs::msg::Odometry> odom_sub;
    odom_sub.subscribe(this, "/odom");

    Subscriber<geometry_msgs::msg::TwistStamped> cmd_vel_sub;
    cmd_vel_sub.subscribe(this, "/robot/cmd_vel");

    Synchronizer<sync_policy> sync{sync_policy(10), odom_sub, cmd_vel_sub};
    sync.registerCallback(std::bind(&gir::GzIoRos::convert_data, this, std::placeholders::_1, std::placeholders::_2));
}

void gir::GzIoRos::convert_data(nav_msgs::msg::Odometry::ConstSharedPtr odom,
                                geometry_msgs::msg::TwistStamped::ConstSharedPtr cmd_vel) {}
