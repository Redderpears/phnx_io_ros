#include "phnx_io_ros/serial.hpp"
#include <iostream>
#include <glob.h>

serial::serial::serial() {}

serial::serial::serial(rclcpp::Logger logger) {
    this->log = &logger;
}

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
    glob64_t gstruct;

    int result = glob64(search_term, GLOB_ERR, NULL, &gstruct);

    //Ensure we actually found a serial port
    if (result != 0) {
        if (result == GLOB_NOMATCH) {
            logger("Failed to find serial device using search pattern!", -1);
        } else {
            logger("Unknown glob error!", -1);
        }
    }
    logger("Found ports using pattern!", 0);

    //Connect to the found serial port
    serial::connect(*gstruct.gl_pathv, termios_baud, log);

    //Clean up after connection has finished
    globfree64(&gstruct);
}

///Connect to a serial port
void serial::serial::connect(const char *port, int baud, const rclcpp::Logger &log) {
    std::cout << "Attempting to connect to port: " << port << std::endl;
    port_number = open(port, O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (port_number < 0) {
        logger("Error connecting to the serial port!", -1);
        return;
    }
    logger("Connected to serial port!", 0);
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
        logger("Error from tcsetattr!", -1);
        return;
    }
    logger("Serial port configured!", 0);
}

///Read data from a connected serial port
///@return number of bytes read
uint32_t serial::serial::read_packet(char *buf, int length) const {
    uint32_t len = read(port_number, buf, length);
    return len;
}

///Write data to a connected serial port
///@return number of bytes written
uint32_t serial::serial::write_packet(uint8_t *buf, int length) const {
    return write(port_number, buf, length);
}

///Closes connection to a serial port
void serial::serial::close_connection(const rclcpp::Logger &log) const {
    int result = close(port_number);
    if (result != 0) {
        logger("Failed to properly close connection!", -1);
    }
}

void serial::serial::logger(std::string str, int severity) const {
    if (this->log) {
        if (severity < 0) {
            RCLCPP_ERROR(*(this->log), "%s", str.c_str());
        } else {
            RCLCPP_INFO(*(this->log), "%s", str.c_str());
        }
    } else {
        if (severity < 0) {
            fprintf(stdout, "%s", str.c_str());
        } else {
            fprintf(stderr, "%s", str.c_str());
        }
    }
}

