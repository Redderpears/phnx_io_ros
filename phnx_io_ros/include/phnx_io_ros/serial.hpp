#pragma once

#include <fcntl.h>  // Contains file controls like O_RDWR
#include <libudev.h>
#include <sys/ioctl.h>
#include <termios.h>  // Contains POSIX terminal control definitions
#include <unistd.h>   // write(), read(), close()

#include <cerrno>  // Error integer and strerror() function
#include <cstdint>
#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <thread>
#include <vector>

namespace serial {

/// Drive message to can bus, either brake or throttle
struct drive_msg {
    uint8_t header = 0x54;
    uint8_t type;
    uint16_t length = 1;
    /// Actuator engagement as a 0-100 percent
    uint8_t speed;
} __attribute__((packed));

/// Steering message to can bus
struct steer_msg {
    uint8_t header = 0x54;
    uint8_t type;
    uint16_t length = 4;
    /// Angle in degrees to set the actuator, left positive.
    float angle;
} __attribute__((packed));

/// Received encoder message
struct enc_msg {
    uint16_t ticks;
    float speed;
} __attribute__((packed));

/// Raw received can message
struct message {
    uint8_t header;
    uint8_t type;
    uint16_t length;
    uint8_t data[512];
};

class serial {
private:
    struct termios tty {};
    rclcpp::Logger* log{nullptr};
    int fd{-1};
    std::atomic<bool> read_exit_flag{false};
    std::thread* readThr{nullptr};
    bool is_connected{false};
    std::vector<uint8_t> tempData;
    std::function<void(message)> msgCallback;

    /// Mutex that synchronizes writes
    std::mutex write_mtx;

    /// Logs with either RCLCPP logs or normal stdout/stderr
    ///@param str string to write to the log
    ///@param severity severity of the log, 1 is warning, 0 is info, -1 is error,
    ///and -2 is fatal
    void logger(const std::string& str, int severity) const;

public:
    explicit serial(rclcpp::Logger log, std::function<void(message)> callback);

    ///Opens and configures a serial port
    ///@param port_name string name of file descriptor
    ///@param baud baud rate to use
    int open_connection(const std::string& port_name, long baud);

    /// Closes connection to a serial port
    ///@param port_num file descriptor for a connected port
    void close_connection();

    ///@return returns fd of current open port
    int get_fd() const;

    bool connected() const;

    /// Read data from a connected serial port
    ///@param buf buffer to store read data in
    ///@param length length of data to read
    ///@param port_num file descriptor for a connected port
    ///@return number of bytes read, -1 returned on error
    static void read_process(void* param);

    /// Write data to a connected serial port. This is atomic and threadsafe.
    ///@param buf data to write to the port
    ///@param length length of data
    ///@return number of bytes written, -1 returned on error
    int32_t write_packet(uint8_t* buf, uint32_t length);

    /// Processes raw byte stream into messages
    ///@param buf raw buffer from read call
    ///@param len number of bytes read as returned by read call
    void process_packet(char* buf, int len);
};
}  // namespace serial