#pragma once

#include <rclcpp/rclcpp.hpp>

#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/float64.hpp"

namespace gir {

class GzIoRos : public rclcpp::Node {
public:
    explicit GzIoRos(rclcpp::NodeOptions options);

    ///@brief Aggregate ackermann message and odom message into an ackermann odom message
    ///@param odom Odom message to convert
    ///@param ack Twist message to convert
    ackermann_msgs::msg::AckermannDrive convert_data(nav_msgs::msg::Odometry::ConstSharedPtr odom,
                                                     const ackermann_msgs::msg::AckermannDrive& ack) const;

    ///@breif Callback for new odom messages. Aggregates these with the current input state to create
    /// ackermann odom messages.
    ///@param odom New message from topic
    void odom_cb(nav_msgs::msg::Odometry::SharedPtr odom);

    ///@breif Callback for new ack messages. This updates the current state of inputs. Also converts command
    /// into separate steering and drive commands for gazebo.
    ///@param ack New message from topic
    void ack_cb(ackermann_msgs::msg::AckermannDrive::SharedPtr ack);

    ///@breif publishes ackermann message
    ///@param msg message to publish
    void publish(ackermann_msgs::msg::AckermannDrive msg);

private:
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDrive>::SharedPtr _odom_acks_pub;

    /// Gazebo steering system
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr _steering_pub;
    /// Gazebo drive system
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr _drive_pub;

    rclcpp::Subscription<ackermann_msgs::msg::AckermannDrive>::SharedPtr _ack_sub;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr _odom_sub;

    /// The last ackermann command received. This is the assumed state of the throttle and steering wheel.
    ackermann_msgs::msg::AckermannDrive last_msg{};
    double _wheelbase{};
};

}  // namespace gir