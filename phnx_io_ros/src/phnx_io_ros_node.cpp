#include <rclcpp/rclcpp.hpp>
#include "phnx_io_ros/phnx_io_ros.hpp"

pir::PhnxIoRos::PhnxIoRos(rclcpp::NodeOptions options)
        : Node("phnx_io_ros", options) {
    _port = this->declare_parameter("port_search_term", "/dev/ttyACM*");
    _baud_rate = this->declare_parameter("baud_rate", 115200);
    _max_throttle_speed = this->declare_parameter("max_throttle_speed", 1.0);
    _max_brake_speed = this->declare_parameter("max_brake_speed", 1.0);

    _odom_acks_pub = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("/odom_ack", 10);
    _acks_sub = this->create_subscription<ackermann_msgs::msg::AckermannDriveStamped>(
            "/ack_vel", 10, std::bind(&PhnxIoRos::send_can_cb, this, std::placeholders::_1));

    //Setup serial connection
    port.setup_port(_port.c_str(), _baud_rate, this->get_logger());
}

void pir::PhnxIoRos::send_can_cb(ackermann_msgs::msg::AckermannDriveStamped::SharedPtr msg) {}
