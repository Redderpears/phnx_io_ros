#include "phnx_io_ros/serial.hpp"
#include <iostream>
#include <glob.h>

namespace serial {
    
    ///Search for a serial port
    char ** serial::find_port(std::string search_term){
        glob_t * gstruct;
        int result = glob("/dev/ttyACM*", GLOB_MARK, NULL, gstruct);
        if(result != 0){
            if(result == GLOB_NOMATCH){
                fprintf(stderr, "Failed to find serial device on /dev/ttyACM*");
            }
            else{
                fprintf(stderr, "Unknown glob error!");
            }
        }
        printf("Found port: %s\n", *gstruct->gl_pathv);
        return gstruct->gl_pathv;
    }

    ///Connect to a serial port
    bool serial::connect(const char *port) {
        std::cout << "Attempting to connect to port: " << port << std::endl;
        port_number = open(port, O_RDWR);
        if (port_number < 0) {
            printf("Error %i connecting to the serial port!\n", errno);
            return false;
        }
        printf("Connected to serial port!\n");
        return true;
    }

    ///Configure a serial port
    void serial::configure() {
        //Get params from port
        if (tcgetattr(port_number, &tty) != 0) {
            printf("Error %i from tcgetattr\n", errno);
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
        cfsetispeed(&tty, B115200);

        if (tcsetattr(port_number, TCSANOW, &tty) != 0) {
            printf("Error %i from tcsetattr\n", errno);
        }
    }

    ///Read data from the serial port return is number of bytes read
    uint32_t serial::read_packet(char *buf, int length) {
        uint32_t len = read(port_number, buf, length);
        return len;
    }

    ///Write data to a serial port return is number of bytes written
    uint32_t serial::write_packet(uint8_t *buf, int length) {
        return write(port_number, buf, length);
    }

}//serial
