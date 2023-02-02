#include <functional>
#include <rclcpp/rclcpp.hpp>
#include "message_filters/subscriber.h"
#include "gz_io_ros/gz_io_ros.hpp"

using namespace message_filters;

double gir::GzIoRos::convert_trans_rot_vel_to_steering_angle(double vel, double omega, double wheelbase) {
    if (omega == 0 || vel == 0) {
        return 0;
    }

    // Remove negative so steering doesn't reverse when reversing.
    vel = std::abs(vel);

    auto rad = vel / omega;
    return std::atan(wheelbase / rad);
}

gir::GzIoRos::GzIoRos(rclcpp::NodeOptions options)
        : Node("gz_io_ros", options) {

    _max_throttle_speed = this->declare_parameter("max_throttle_speed", 10.0);
    _max_braking_speed = this->declare_parameter("max_brake_speed", 10.0);
    _wheelbase = this->declare_parameter("wheelbase", 1.0);

    _odom_acks_pub = this->create_publisher<ackermann_msgs::msg::AckermannDrive>("/odom_ack", 10);

    Subscriber<nav_msgs::msg::Odometry> odom_sub;
    odom_sub.subscribe(this, "/odom");

    Subscriber<geometry_msgs::msg::TwistStamped> cmd_vel_sub;
    cmd_vel_sub.subscribe(this, "/robot/cmd_vel");

    Synchronizer<sync_policy> sync{sync_policy(10), odom_sub, cmd_vel_sub};
    sync.registerCallback(std::bind(&gir::GzIoRos::convert_data, this, std::placeholders::_1, std::placeholders::_2));
}

void gir::GzIoRos::convert_data(nav_msgs::msg::Odometry::ConstSharedPtr odom,
                                geometry_msgs::msg::TwistStamped::ConstSharedPtr cmd_vel) {
    //Convert odom msg to ackermann drive msg
    ackermann_msgs::msg::AckermannDrive msg;
    msg.steering_angle = this->convert_trans_rot_vel_to_steering_angle(odom->twist.twist.linear.x,
                                                                       odom->twist.twist.angular.z, this->_wheelbase);
    if (odom->twist.twist.linear.x < 0) {
        //set to brake
        msg.acceleration = odom->twist.twist.linear.x / _max_braking_speed;
    } else {
        msg.acceleration = odom->twist.twist.linear.x / _max_throttle_speed;
    }
    msg.speed = cmd_vel->twist.linear.x;

    this->_odom_acks_pub->get()->publish(msg);

}
