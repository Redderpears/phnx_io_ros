#include "phnx_io_ros/serial.hpp"
#include <iostream>
#include <glob.h>
#include <filesystem>

///Search for and connect/configure a serial port
void serial::serial::setup_port(const char *search_term, int baud_rate, const rclcpp::Logger &log) {
    int termios_baud{};
    switch (baud_rate) {
        case 9600:
            termios_baud = B9600;
            break;
        case 115200:
            termios_baud = B115200;
            break;
        case 1152000:
            termios_baud = B1152000;
            break;
        default:
            termios_baud = B115200;
            break;
    }

    //DO NOT INITIALIZE THIS WILL BREAK GLOB
    glob_t gstruct;

    int result = glob(search_term, GLOB_ERR, NULL, &gstruct);

    //Ensure we actually found a serial port
    if (result != 0) {
        if (result == GLOB_NOMATCH) {
            RCLCPP_ERROR(log, "Failed to find serial device using search pattern %s!", search_term);
        } else {
            RCLCPP_ERROR(log, "Unknown glob error!");
        }
    }
    RCLCPP_INFO(log, "Found port: %s", *gstruct.gl_pathv);

    //Connect to the found serial port
    serial::connect(*gstruct.gl_pathv, termios_baud, log);

    //Clean up after connection has finished
    globfree(&gstruct);
}

///Connect to a serial port
void serial::serial::connect(const char *port, int baud, const rclcpp::Logger &log) {
    std::cout << "Attempting to connect to port: " << port << std::endl;
    port_number = open(port, O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (port_number < 0) {
        RCLCPP_ERROR(log, "Error connecting to the serial port!");
        return;
    }
    RCLCPP_INFO(log, "Connected to serial port!");
    configure(baud, log);
}

///Configure a serial port
void serial::serial::configure(int baud, const rclcpp::Logger &log) {
    //Get params from port
    if (tcgetattr(port_number, &tty) != 0) {
        RCLCPP_ERROR(log, "Error from tcgetattr!");
        return;
    }
    //Set port parameters
    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag |= CS8; // 8 bits per byte (most common)
    tty.c_cflag &= ~CRTSCTS;
    tty.c_cflag |= CREAD | CLOCAL;
    tty.c_lflag &= ~ICANON;
    tty.c_lflag &= ~ECHO; // Disable echo
    tty.c_lflag &= ~ECHOE; // Disable erasure
    tty.c_lflag &= ~ECHONL; // Disable new-line echo
    tty.c_lflag &= ~ISIG;
    tty.c_iflag &= ~(IXON | IXOFF | IXANY);
    tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL);
    tty.c_oflag &= ~ONLCR;
    tty.c_oflag &= ~OPOST;

    //Set baud rate
    cfsetispeed(&tty, baud);

    if (tcsetattr(port_number, TCSANOW, &tty) != 0) {
        RCLCPP_ERROR(log, "Error from tcsetattr!");
        return;
    }
    RCLCPP_INFO(log, "Serial port configured!");
}

///Read data from the serial port return is number of bytes read
uint32_t serial::serial::read_packet(char *buf, int length) {
    uint32_t len = read(port_number, buf, length);
    return len;
}

///Write data to a serial port return is number of bytes written
uint32_t serial::serial::write_packet(uint8_t *buf, int length) {
    return write(port_number, buf, length);
}

void serial::serial::close_connection(const rclcpp::Logger &log) {
    int result = close(port_number);
    if (result != 0) {
        RCLCPP_ERROR(log, "Failed to properly close connection!");
    }
}
