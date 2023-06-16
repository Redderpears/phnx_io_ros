#include <rclcpp/rclcpp.hpp>

#include "libackermann/libackermann.hpp"
#include "phnx_io_ros/phnx_io_ros.hpp"

pir::PhnxIoRos::PhnxIoRos(rclcpp::NodeOptions options) : Node("phnx_io_ros", options) {
    this->_port_pattern =
        this->declare_parameter("port_search_pattern", "/dev/serial/by-id/usb-Teensyduino_USB_Serial*");
    this->_baud_rate = this->declare_parameter("baud_rate", 115200);
    this->_max_throttle_speed = this->declare_parameter("max_throttle_speed", 2.0);
    this->_max_brake_speed = this->declare_parameter("max_braking_speed", -2.0);

    _odom_acks_pub = this->create_publisher<ackermann_msgs::msg::AckermannDrive>("/odom_ack", 10);
    _acks_sub = this->create_subscription<ackermann_msgs::msg::AckermannDrive>(
        "/robot/ack_vel", 10, std::bind(&PhnxIoRos::send_can_cb, this, std::placeholders::_1));
    _robot_state_client = this->create_client<robot_state_msgs::srv::SetState>("/robot/set_state");

    while (find_devices() != 0) {
        rclcpp::sleep_for(std::chrono::milliseconds(500));
    }

    //Create a link to the callback function to process new incoming messages
    std::function<void(serial::message)> read_callback = [this](auto&& PH1) {
        read_data(std::forward<decltype(PH1)>(PH1));
    };

    // Create serial handler for this device
    cur_device.handler = new serial::serial(this->get_logger(), read_callback);

    while (cur_device.handler->open_connection(cur_device.port_name, this->_baud_rate) != 0) {
        RCLCPP_ERROR(this->get_logger(), "Error opening device!");
        rclcpp::sleep_for(std::chrono::milliseconds(500));
    }
}

int pir::PhnxIoRos::find_devices() {
    // DO NOT INITIALIZE THIS WILL BREAK GLOB
    glob64_t gstruct;

    int result = glob64(_port_pattern.c_str(), GLOB_ERR, NULL, &gstruct);

    // Ensure we actually found a serial port
    if (result != 0) {
        if (result == GLOB_NOMATCH) {
            RCLCPP_ERROR(this->get_logger(), "Failed to find serial device using search pattern!");
            return -1;
        } else {
            RCLCPP_ERROR(this->get_logger(), "Unknown glob error!");
            return -1;
        }
    }

    // Set first found device as our current device
    this->cur_device.port_name = *gstruct.gl_pathv;
    return 0;
}

void pir::PhnxIoRos::send_can_cb(ackermann_msgs::msg::AckermannDrive::SharedPtr msg) {
    auto ratio = ack::get_inverse_steering_ratio(ack::Project::Phoenix);

    // Copy the newest message into last_ack and move speed field into
    // acceleration field
    ackermann_msgs::msg::AckermannDrive pub_msg;
    pub_msg.acceleration = msg->speed;
    pub_msg.steering_angle = ratio(msg->steering_angle);

    // If we've received any encoder messages from the CAN bus, we grab its speed value otherwise set it to zero
    if (!enc_msgs.empty()) {
        pub_msg.speed = enc_msgs.front().speed;
        enc_msgs.pop_front();
    } else {
        pub_msg.speed = 0.0;
    }
    _odom_acks_pub->get()->publish(pub_msg);

    // Steering/Drive message to send over
    serial::drive_msg drv_msg{};
    serial::steer_msg st_msg{};

    // Get percentage brake and throttle and send their respective messages
    if (msg->speed < 0) {
        auto percent_brake = static_cast<uint8_t>((msg->speed / _max_brake_speed) * 100);
        drv_msg.type = pir::CanMappings::SetBrake;
        drv_msg.speed = percent_brake;
    } else {
        auto percent_throttle = static_cast<uint8_t>((msg->speed / _max_throttle_speed) * 100);
        drv_msg.type = pir::CanMappings::SetThrottle;
        drv_msg.speed = percent_throttle;
    }
    drv_msg.length = 1;

    // Send a drive message to the interface device to publish onto the CAN bus
    RCLCPP_INFO(this->get_logger(), "Sending drive msg with speed: %u", drv_msg.speed);
    if (cur_device.handler->write_packet(reinterpret_cast<uint8_t*>(&drv_msg), sizeof(serial::drive_msg)) == -1) {
        RCLCPP_ERROR(this->get_logger(), "Failed to write message to device: %s with fd: %d",
                     cur_device.port_name.c_str(), cur_device.handler->get_fd());
    }

    // Create a steering message
    st_msg.type = pir::CanMappings::SetAngle;
    st_msg.length = 8;
    st_msg.angle = ratio(msg->steering_angle);
    st_msg.position = 0.0;

    // Send a steering message to the interface device to publish onto the CAN bus
    RCLCPP_INFO(this->get_logger(), "Sending steer msg with angle: %f, position: %f", st_msg.angle, st_msg.position);
    if (cur_device.handler->write_packet(reinterpret_cast<uint8_t*>(&st_msg), sizeof(serial::steer_msg)) == -1) {
        RCLCPP_ERROR(this->get_logger(), "Failed to write message to device: %s with fd: %d",
                     cur_device.port_name.c_str(), cur_device.handler->get_fd());
    }
}

void pir::PhnxIoRos::read_data(serial::message m) {
    auto kill = std::make_shared<robot_state_msgs::srv::SetState::Request>();
    serial::enc_msg* msg;
    RCLCPP_INFO(this->get_logger(), "Received message:\n Type: %u\n Length: %u\n Data: \n", m.type, m.length);
    for (int i = 0; i < m.length; i++) {
        RCLCPP_INFO(this->get_logger(), "%u", m.data[i]);
    }

    // If we receive an auton kill message, we send a service request to drive mode switch to switch the kart to a killed state
    // If we receive an encoder tick, we add it to our vector to be used when the next control message arrives
    switch (m.type) {
        case CanMappings::KillAuton:
            RCLCPP_WARN(this->get_logger(), "Auton kill signal received!");
            kill->state.state = robot_state_msgs::msg::State::KILL;
            this->_robot_state_client->async_send_request(kill);
            break;
        case CanMappings::EncoderTick:
            msg = reinterpret_cast<serial::enc_msg*>(m.data);
            enc_msgs.push_back(*msg);
            RCLCPP_INFO(this->get_logger(), "Received encoder tick!");
            break;
        default:
            RCLCPP_INFO(this->get_logger(), "Received non encoder tick/auton kill message!");
            break;
    }
}

void pir::PhnxIoRos::close() {
    // Clean up serial connection
    if (cur_device.handler->connected()) {
        cur_device.handler->close_connection();
    }
}

pir::PhnxIoRos::~PhnxIoRos() { close(); }