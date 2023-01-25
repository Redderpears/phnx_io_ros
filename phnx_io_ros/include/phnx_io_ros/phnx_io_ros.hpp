#pragma once

#include "optional"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/logger.hpp"
#include "phnx_io_ros/serial.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"

namespace pir{

class PhnxIoRos : public rclcpp::Node{
    public:
        explicit PhnxIoRos(rclcpp::NodeOptions options);
    private:

        std::optional<std::shared_ptr<rclcpp::Subscription<ackermann_msgs::msg::AckermannDriveStamped>>> _acks_sub =
          std::nullopt;

        std::optional<std::shared_ptr<rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>>>
          _odom_acks_pub = std::nullopt;

        std::string _port{};
        int _baud_rate{};
        serial::serial port{};
        double _max_throttle_speed{};
        double _max_brake_speed{};

        void send_can_cb(ackermann_msgs::msg::AckermannDriveStamped::SharedPtr msg);

};


} //namespace pir