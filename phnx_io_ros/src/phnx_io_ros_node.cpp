#include <cmath>
#include <rclcpp/rclcpp.hpp>

#include "libackermann/libackermann.hpp"
#include "phnx_io_ros/phnx_io_ros.hpp"

pir::PhnxIoRos::PhnxIoRos(rclcpp::NodeOptions options) : Node("phnx_io_ros", options) {
    this->_port_pattern =
        this->declare_parameter("port_search_pattern", "/dev/serial/by-id/usb-Teensyduino_USB_Serial*");
    this->_baud_rate = this->declare_parameter("baud_rate", 115200);

    _odom_pub = this->create_publisher<nav_msgs::msg::Odometry>("/odom_can", 10);
    _acks_sub = this->create_subscription<ackermann_msgs::msg::AckermannDrive>(
        "/robot/ack_vel", 10, std::bind(&PhnxIoRos::send_can_cb, this, std::placeholders::_1));
    _filtered_odom_sub = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom", 10, std::bind(&PhnxIoRos::filtered_odom_cb, this, std::placeholders::_1));
    _robot_state_client = this->create_client<robot_state_msgs::srv::SetState>("/robot/set_state");

    // Find connected interface ECU connected to a USB port
    while (find_devices() != 0) {
        rclcpp::sleep_for(std::chrono::milliseconds(500));
    }

    //Create a link to the callback function to process new incoming messages
    std::function<void(serial::message)> read_callback = [this](auto&& PH1) {
        read_data(std::forward<decltype(PH1)>(PH1));
    };

    // Start CAN read thread
    cur_device.handler = new serial::serial(this->get_logger(), read_callback);

    while (cur_device.handler->open_connection(cur_device.port_name, this->_baud_rate) != 0) {
        RCLCPP_ERROR(this->get_logger(), "Error opening device!");
        rclcpp::sleep_for(std::chrono::milliseconds(500));
    }

    RCLCPP_INFO(this->get_logger(), "Connected to device!");

    // Start pid thread
    this->pid = std::make_unique<PidInterface>(std::bind(&PhnxIoRos::handle_pid_update, this, std::placeholders::_1));

    /* Now we have three threads:
     * 1) Main node thread subs and pubs
     * 2) CAN read thread
     * 3) PID control thread, which writes to CAN
     */
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
    // Send speed command to PID
    this->pid->set_command(*msg);

    // Set steering directly, as control is handled on device
    auto ratio = ack::get_inverse_steering_ratio(ack::Project::Phoenix);

    serial::steer_msg st_msg{};
    st_msg.type = pir::CanMappings::SetAngle;

    // ROS steering is in rad, bus is in degrees
    st_msg.angle = ratio(msg->steering_angle) / M_PI * 180;

    // Send a steering message to the interface device to publish onto the CAN bus
    RCLCPP_INFO(this->get_logger(), "Sending steer msg with angle: %f", st_msg.angle);
    if (cur_device.handler->write_packet(reinterpret_cast<uint8_t*>(&st_msg), sizeof(serial::steer_msg)) == -1) {
        RCLCPP_ERROR(this->get_logger(), "Failed to write message to device: %s with fd: %d",
                     cur_device.port_name.c_str(), cur_device.handler->get_fd());
    }
}

// Keep in mind, this occurs in the read thread
void pir::PhnxIoRos::read_data(serial::message m) {
    auto kill = std::make_shared<robot_state_msgs::srv::SetState::Request>();
    serial::enc_msg* msg;

    // If we receive an auton kill message, we send a service request to drive mode switch to switch the kart to a killed state
    // If we receive an encoder tick, we add it to our vector to be used when the next control message arrives
    nav_msgs::msg::Odometry odom{};
    switch (m.type) {
        case CanMappings::KillAuton:
            RCLCPP_WARN(this->get_logger(), "Auton kill signal received!");
            kill->state.state = robot_state_msgs::msg::State::KILL;
            this->_robot_state_client->async_send_request(kill);

            // Set PID to 0 set speed on estop, to avoid accumulating error when stopped.
            this->pid->set_command(ackermann_msgs::msg::AckermannDrive{});
            break;
        case CanMappings::EncoderTick:
            // Publish odom from encoder
            msg = reinterpret_cast<serial::enc_msg*>(m.data);

            odom.twist.twist.linear.x = msg->speed;
            odom.header.stamp = this->get_clock()->now();
            this->_odom_pub->publish(odom);
            break;
        default:
            RCLCPP_INFO(this->get_logger(), "Received non encoder tick/auton kill message!");
            break;
    }
}

void pir::PhnxIoRos::filtered_odom_cb(nav_msgs::msg::Odometry::ConstSharedPtr msg) {
    this->pid->add_feedback(*msg);
    RCLCPP_INFO(this->get_logger(), "Speed: %f", msg->twist.twist.linear.x);
}

void pir::PhnxIoRos::handle_pid_update(std::tuple<double, phnx_control::SpeedController::Actuator> control) {
    auto [val, actuator] = control;

    RCLCPP_INFO(this->get_logger(), "Sending drive msg with level: %f and actuator: %u", val, uint32_t(actuator));

    serial::drive_msg throttle{};
    serial::drive_msg brake{};

    // Remove risk of underflow
    if (val < 0) {
        val = 0;
    }

    if (actuator == phnx_control::SpeedController::Actuator::Throttle) {
        // Set throttle to control, and zero brake
        throttle.type = CanMappings::SetThrottle;
        throttle.speed = uint8_t(val * 100);

        brake.type = CanMappings::SetBrake;
        brake.speed = 0;

        // Send commands to can
        RCLCPP_INFO(this->get_logger(), "Sending throttle command: %f", val);
        this->cur_device.handler->write_packet(reinterpret_cast<uint8_t*>(&throttle), sizeof(throttle));
        this->cur_device.handler->write_packet(reinterpret_cast<uint8_t*>(&brake), sizeof(brake));
    } else {
        // Set brake to control, and zero throttle
        throttle.type = CanMappings::SetThrottle;
        throttle.speed = 0;

        brake.type = CanMappings::SetBrake;
        brake.speed = uint8_t(val * 100);

        // Send commands to can
        RCLCPP_INFO(this->get_logger(), "Sending brake command: %f", val);
        this->cur_device.handler->write_packet(reinterpret_cast<uint8_t*>(&throttle), sizeof(throttle));
        this->cur_device.handler->write_packet(reinterpret_cast<uint8_t*>(&brake), sizeof(brake));
    }
}

void pir::PhnxIoRos::close() {
    // Clean up serial connection
    if (cur_device.handler->connected()) {
        cur_device.handler->close_connection();
    }
}

pir::PhnxIoRos::~PhnxIoRos() { close(); }