#include <rclcpp/rclcpp.hpp>
#include "phnx_io_ros/phnx_io_ros.hpp"

pir::PhnxIoRos::PhnxIoRos(rclcpp::NodeOptions options)
        : Node("phnx_io_ros", options) {
    _port = this->declare_parameter("port_search_term", "/dev/ttyACM*");
    _baud_rate = this->declare_parameter("baud_rate", 115200);
    _max_throttle_speed = this->declare_parameter("max_throttle_speed", 1.0);
    _max_brake_speed = this->declare_parameter("max_brake_speed", 1.0);

    _odom_acks_pub = this->create_publisher<ackermann_msgs::msg::AckermannDrive>("/odom_ack", 10);
    _acks_sub = this->create_subscription<ackermann_msgs::msg::AckermannDrive>(
            "/ack_vel", 10, std::bind(&PhnxIoRos::send_can_cb, this, std::placeholders::_1));

    port = serial::serial(this->get_logger());

    //Setup serial connection
    port.setup_port(_port.c_str(), _baud_rate, this->get_logger());
}

///Convert ackermann messages into CAN messages and send them to the CAN bus
void pir::PhnxIoRos::send_can_cb(ackermann_msgs::msg::AckermannDrive::SharedPtr msg) {

    //Copy the newest message into last_ack and move speed field into acceleration field
    this->last_ack.acceleration = msg->speed;
    serial::message ser_msg{};
    ser_msg.length = 1;

    //Get percentage brake and throttle and send their respective messages
    if (msg->speed < 0) {
        uint8_t percent_brake = msg->speed  / _max_brake_speed;
        ser_msg.type = pir::CanMappings::SetBrake;
        ser_msg.data[0] = percent_brake;
    } else {
        uint8_t percent_throttle = msg->speed / _max_throttle_speed;
        ser_msg.type = pir::CanMappings::SetThrottle;
        ser_msg.length = 1;
        ser_msg.data[0] = percent_throttle;
    }

    port.write_packet(reinterpret_cast<uint8_t *>(&ser_msg), 4);

    //send steering angle message
    ser_msg.type = pir::CanMappings::SetAngle;
    ser_msg.length = 1;
    ser_msg.data[0] = msg->steering_angle;
    port.write_packet(reinterpret_cast<uint8_t *>(&ser_msg), 4);
}

pir::PhnxIoRos::~PhnxIoRos() {
    //Clean up serial connection
    port.close_connection(this->get_logger());
}
