#pragma once

#include "optional"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/logger.hpp"
#include "phnx_io_ros/serial.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"

namespace pir {

    enum CanMappings {
        KillAuton = 0x0,
        SetBrake = 0x1,
        LockBrake = 0x2,
        UnlockBrake = 0x3,
        SetAngle = 0x4,
        SetThrottle = 0x6,
        TrainingMode = 0x8,
    };

    class PhnxIoRos : public rclcpp::Node {
    public:
        explicit PhnxIoRos(rclcpp::NodeOptions options);

        ~PhnxIoRos() override;

    private:

        std::optional<std::shared_ptr<rclcpp::Subscription<ackermann_msgs::msg::AckermannDrive>>> _acks_sub =
                std::nullopt;

        std::optional<std::shared_ptr<rclcpp::Publisher<ackermann_msgs::msg::AckermannDrive>>>
                _odom_acks_pub = std::nullopt;

        std::string _port{};
        int _baud_rate{};
        serial::serial port{};
        double _max_throttle_speed{};
        double _max_brake_speed{};
        ackermann_msgs::msg::AckermannDrive last_ack{};

        void send_can_cb(ackermann_msgs::msg::AckermannDrive::SharedPtr msg);

    };


} //namespace pir