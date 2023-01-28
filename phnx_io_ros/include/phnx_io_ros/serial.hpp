#pragma once

#include <fcntl.h> // Contains file controls like O_RDWR
#include <cerrno> // Error integer and strerror() function
#include <termios.h> // Contains POSIX terminal control definitions
#include <unistd.h> // write(), read(), close()
#include <string>
#include <cstdint>
#include <iostream>
#include <vector>
#include <glob.h>
#include <rclcpp/rclcpp.hpp>


namespace serial {

    struct message {
        uint8_t type;
        uint16_t length;
        uint8_t data[512];
    }__attribute__((packed));

    class serial {
    private:
        struct termios tty;
        int port_number;
        rclcpp::Logger *log = nullptr;

        void logger(std::string str, int severity) const;

    public:
        serial();

        explicit serial(rclcpp::Logger log);

        //Find and connect to a serial port
        void setup_port(const char *search_term, int baud_rate, const rclcpp::Logger &log);

        //Connect to a serial port
        void connect(const char *port, int baud, const rclcpp::Logger &log);

        //Configure a serial port
        void configure(int baud, const rclcpp::Logger &log);

        //Close all connected serial ports
        void close_connection(const rclcpp::Logger &log) const;

        //Read data from the serial port
        uint32_t read_packet(char *buf, int length) const;

        uint32_t write_packet(uint8_t *buf, int length) const;
    };
}