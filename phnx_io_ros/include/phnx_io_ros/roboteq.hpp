#pragma once

#define termios asmtermios
#include <CppLinuxSerial/SerialPort.hpp>
#undef termios

#include <functional>
#include <memory>
#include <regex>
#include <vector>

#include "glob.h"
#include "optional"

/// Interface to a Roboteq motor controller connected over USB.
class Roboteq {
    std::unique_ptr<mn::CppLinuxSerial::SerialPort> serial;
    /// 0-1 float to scale all commands by. Caps the speed.
    float power_scale;

    /// Finds the usb port for the roboteq
    std::optional<std::string> enumerate_port() {
        // DO NOT INITIALIZE THIS WILL BREAK GLOB
        glob64_t gstruct;

        int result = glob64("/dev/serial/by-id/usb-Roboteq_Motor_Controller*", GLOB_ERR, NULL, &gstruct);

        // Ensure we actually found a serial port
        if (result != 0) {
            if (result == GLOB_NOMATCH) {
                return std::nullopt;
            } else {
                return std::nullopt;
            }
        }

        // Set first found device as our current device
        return std::string{*gstruct.gl_pathv};
    }

public:
    explicit Roboteq(float power_scale) { this->power_scale = power_scale; }

    /// Connects to the roboteq over USB.
    bool connect() {
        auto maybe_port = this->enumerate_port();

        if (!maybe_port) {
            return false;
        }

        this->serial = std::make_unique<mn::CppLinuxSerial::SerialPort>(
            *maybe_port, mn::CppLinuxSerial::BaudRate::B_115200, mn::CppLinuxSerial::NumDataBits::EIGHT,
            mn::CppLinuxSerial::Parity::NONE, mn::CppLinuxSerial::NumStopBits::ONE);

        this->serial->SetTimeout(50);
        this->serial->Open();

        return this->serial->GetState() == mn::CppLinuxSerial::State::OPEN;
    }

    /// Sets the motor to the percent power, governed by power scaling.
    bool set_power(float percent) {
        // Scale output
        uint16_t level = percent * 1000 * this->power_scale;

        // Send go command to percent max power
        this->serial->Write(std::string{"!G 1 " + std::to_string(level) + " _"});

        std::string res{};
        this->serial->Read(res);

        return res[0] == '+';
    }

    /// Send a ping to the Roboteq, and check for its response.
    bool check_alive() {
        // Send ENQ
        std::vector<uint8_t> vec{0x5};
        this->serial->WriteBinary(vec);

        vec.clear();
        this->serial->ReadBinary(vec);

        // ACK
        return vec[0] == 0x6;
    }
};