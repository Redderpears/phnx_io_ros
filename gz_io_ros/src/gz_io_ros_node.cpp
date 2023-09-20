#include <functional>
#include <rclcpp/rclcpp.hpp>

#include "gz_io_ros/gz_io_ros.hpp"
#include "libackermann/libackermann.hpp"

gir::GzIoRos::GzIoRos(rclcpp::NodeOptions options) : Node("gz_io_ros", options) {
    _wheelbase = this->declare_parameter("wheelbase", 1.0);

    // ack odom out
    _odom_acks_pub = this->create_publisher<ackermann_msgs::msg::AckermannDrive>("/odom_ack", 10);

    // gazebo
    _drive_pub = this->create_publisher<geometry_msgs::msg::Twist>("/robot/cmd_vel", 10);

    // inputs
    _ack_sub = this->create_subscription<ackermann_msgs::msg::AckermannDrive>(
        "/robot/ack_vel", rclcpp::QoS(10).reliable(), std::bind(&GzIoRos::ack_cb, this, std::placeholders::_1));
    _odom_sub = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom", rclcpp::QoS(10).reliable(), std::bind(&GzIoRos::odom_cb, this, std::placeholders::_1));
}

ackermann_msgs::msg::AckermannDrive gir::GzIoRos::convert_data(nav_msgs::msg::Odometry::ConstSharedPtr odom,
                                                               const ackermann_msgs::msg::AckermannDrive& ack) const {
    //Convert odom + ack msg to ackermann odom msg
    ackermann_msgs::msg::AckermannDrive msg{};

    auto ratio = ack::get_inverse_steering_ratio(ack::Project::Phoenix);

    // ack odom messages carry steering wheel in steering field, rather than ackermann wheel angle
    msg.steering_angle = ratio(ack.steering_angle);

    // Throttle percent (encoded as a speed as per a twist message) is in the accel field
    msg.acceleration = ack.speed;

    // Encoder values (odom in sims case) is in the speed field
    msg.speed = static_cast<float>(odom->twist.twist.linear.x);
    msg.jerk = 0.0;
    return msg;
}

void gir::GzIoRos::publish(ackermann_msgs::msg::AckermannDrive msg) { _odom_acks_pub->publish(msg); }

void gir::GzIoRos::odom_cb(nav_msgs::msg::Odometry::SharedPtr odom) {
    // Odom is paired with the last known throttle and steering input state
    publish(convert_data(odom, this->last_msg));
}

void gir::GzIoRos::ack_cb(ackermann_msgs::msg::AckermannDrive::SharedPtr ack) {
    // Because odom is constant in sim, we can just store the last steering value as a known state, rather than queuing
    this->last_msg = *ack;

    // Convert ackermann to twist for gazebo
    AckermannCommand nack {ack->speed, ack->steering_angle};
    auto twist = ack::ackermann_to_twist(nack, static_cast<float>(this->_wheelbase));

    geometry_msgs::msg::Twist drive{};
    drive.linear.x = twist.v_linear_x;
    drive.angular.z = twist.v_angular_yaw;
    this->_drive_pub->publish(drive);
}