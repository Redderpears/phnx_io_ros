#pragma once

#include "ackermann_msgs/msg/ackermann_drive.hpp"
#include "optional"
#include "phnx_io_ros/serial.hpp"
#include "rclcpp/logger.hpp"
#include "rclcpp/rclcpp.hpp"
#include "robot_state_msgs/srv/set_state.hpp"
#include <glob.h>

namespace pir {

enum CanMappings {
    KillAuton = 0x0,
    SetBrake = 0x1,
    LockBrake = 0x2,
    UnlockBrake = 0x3,
    SetAngle = 0x4,
    SetThrottle = 0x6,
    EncoderTick = 0x7,
    TrainingMode = 0x8,
};

/// Contains the device name as well as the serial object that acts as that device's handler for IO
/// This allows for multiple connected teensy devices to be handled in the future.
struct device_info {
    std::string port_name;
    serial::serial *handler;
};

class PhnxIoRos : public rclcpp::Node {
public:
    explicit PhnxIoRos(rclcpp::NodeOptions options);

    ///@breif Reads data of size serial::message from connected port
    void read_data(serial::message m);

    ~PhnxIoRos() override;

private:
    std::optional<std::shared_ptr<rclcpp::Subscription<ackermann_msgs::msg::AckermannDrive>>> _acks_sub = std::nullopt;

    std::optional<std::shared_ptr<rclcpp::Publisher<ackermann_msgs::msg::AckermannDrive>>> _odom_acks_pub =
        std::nullopt;

    rclcpp::Client<robot_state_msgs::srv::SetState>::SharedPtr _robot_state_client;

    // Serial port params
    std::string _port_pattern{};
    std::list<serial::enc_msg> enc_msgs;
    long _baud_rate{};
    device_info cur_device;
    int FAILURE_TOLERANCE{5};

    // Kart control params
    double _max_throttle_speed{};
    double _max_brake_speed{};
    ackermann_msgs::msg::AckermannDrive last_ack{};
    robot_state_msgs::srv::SetState::Request::SharedPtr request =
        std::make_shared<robot_state_msgs::srv::SetState::Request>();

    ///@brief Convert ackermann messages into CAN messages and send them to the
    /// CAN bus
    ///@param msg Ackermann drive message to convert
    void send_can_cb(ackermann_msgs::msg::AckermannDrive::SharedPtr msg);

    ///@brief Closes open serial connection
    void close();

    ///@brief Attempts to find devices given a certain pattern.
    int find_devices();
};

}  // namespace pir