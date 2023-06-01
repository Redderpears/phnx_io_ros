#include "phnx_io_ros/serial.hpp"

#include <iostream>

serial::serial::serial(rclcpp::Logger logger) { this->log = &logger; }

int serial::serial::get_fd() const { return this->fd; }

int serial::serial::open_connection(const std::string& str, long baud) {
    speed_t termios_baud;
    this->fd = open(str.c_str(), O_RDWR | O_NOCTTY | O_SYNC);

    if (this->fd < 0) {
        logger("Error opening serial connection!", -2);
        return -1;
    }

    switch (baud) {
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
    // Get params from port
    if (tcgetattr(this->fd, &tty) != 0) {
        logger("Error from tcgetattr!", -2);
        return -1;
    }
    // Set port parameters
    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag |= CS8;  // 8 bits per byte (most common)
    tty.c_cflag &= ~CRTSCTS;
    tty.c_cflag |= CREAD | CLOCAL;
    tty.c_lflag &= ~ICANON;
    tty.c_lflag &= ~ECHO;    // Disable echo
    tty.c_lflag &= ~ECHOE;   // Disable erasure
    tty.c_lflag &= ~ECHONL;  // Disable new-line echo
    tty.c_lflag &= ~ISIG;
    tty.c_iflag &= ~(IXON | IXOFF | IXANY);
    tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL);
    tty.c_oflag &= ~ONLCR;
    tty.c_oflag &= ~OPOST;

    // Set baud rate
    cfsetispeed(&tty, termios_baud);

    if (tcsetattr(this->fd, TCSANOW, &tty) != 0) {
        logger("Error from tcsetattr!", -2);
        return -1;
    }

    this->is_connected = true;
    // Set up reading thread
    this->readThr = new std::thread(&serial::read_process, this);

    return 0;
}

void serial::serial::read_process(void* param) {
    auto* s = (serial*)param;
    char* read_buf = new char[sizeof(message) + 1];

    if (s->fd != -1) {
        fd_set readfds;
        FD_ZERO(&readfds);
        FD_SET(s->fd, &readfds);

        while (!s->read_exit_flag.load()) {
            int r = pselect(s->fd + 1, &readfds, NULL, NULL, NULL, NULL);
            if (r < 0) {
                if (errno == EINTR) {
                    // Select interrupted
                    //s->logger("Select interrupted!", -1);
                    continue;
                }
            } else if (r == 0) {
                // Timeout occurred
                //s->logger("Timeout occurred!", -1);
                continue;
            } else if (FD_ISSET(s->fd, &readfds)) {
                //Data is available to be read
                int l = (int32_t)read(s->fd, (uint8_t*)read_buf, sizeof(message));
                if(l < 0){
                    //s->logger("Reading error!", -1);
                }
                else{
                    //std::string str = "Read " + std::to_string(l) + " bytes!";
                    //s->logger("Read bytes!", 0);
                }
            }
        }
    }
    delete[] read_buf;
}

int32_t serial::serial::write_packet(uint8_t* buf, uint32_t length) const {
    return (int32_t)write(this->fd, buf, length);
}

void serial::serial::close_connection() {
    if (!is_connected) {
        return;
    }
    this->read_exit_flag = true;
    if (readThr != nullptr && readThr->joinable()) {
        readThr->join();
        delete readThr;
        readThr = nullptr;
    }
    int result = close(this->fd);
    if (result != 0) {
        logger("Failed to properly close connection!", -1);
    }
    this->is_connected = false;
}

void serial::serial::logger(const std::string& str, int severity) const {
    if (this->log) {
        switch (severity) {
            case 1:
                RCLCPP_WARN(*(this->log), "%s", str.c_str());
                break;
            case 0:
                RCLCPP_INFO(*(this->log), "%s", str.c_str());
                break;
            case -1:
                RCLCPP_ERROR(*(this->log), "%s", str.c_str());
                break;
            case -2:
                RCLCPP_FATAL(*(this->log), "%s", str.c_str());
                break;
            default:
                RCLCPP_INFO(*(this->log), "%s", str.c_str());
                break;
        }
    } else {
        if (severity < 0) {
            fprintf(stderr, "%s", str.c_str());
        } else {
            fprintf(stdout, "%s", str.c_str());
        }
    }
}
bool serial::serial::connected() const { return this->is_connected; }
