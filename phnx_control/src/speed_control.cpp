#include "phnx_control/speed_control.hpp"

namespace phnx_control {

SpeedController::SpeedController() {
    // TODO use real values
    this->throttle_pid.initPid(0.2, 0.1, 0, 1, -1);
    this->brake_pid.initPid(0.2, 0.1, 0, 1, -1);
}

std::tuple<double, SpeedController::Actuator> SpeedController::update(double speed, const rclcpp::Time& stamp) {
    uint64_t dt;
    if (!this->last_feedback.has_value()) {
        dt = 0;
    } else {
        dt = (stamp - *this->last_feedback).nanoseconds();
    }
    this->last_feedback = stamp;

    double error = this->set_speed - speed;

    // If speed must be lowered by a large amount, use brakes
    if (error <= -2) {
        if (!this->brake_active) {
            this->disable_throttle();
        }

        // TODO make command in 0-1 (I assume this is implicit in gains?)
        double command = this->brake_pid.computeCommand(error, dt);
        return std::make_tuple(command, Actuator::Brake);
    }
    // Else, use throttle some amount
    else {
        if (this->brake_active) {
            this->disable_breaks();
        }

        double command = this->throttle_pid.computeCommand(error, dt);

        // Negative throttle values mean we should slow, but because we are on this path we have not used the brakes.
        // This means we should coast.
        if (command < 0) {
            command = 0;
        }

        return std::make_tuple(command, Actuator::Throttle);
    }
}

void SpeedController::update_set_speed(double speed) { this->set_speed = speed; }

void SpeedController::disable_breaks() {
    this->brake_pid.setCurrentCmd(0);  //TODO does this make sense to do? Or do we need a fake measurement
    this->brake_active = false;
}

void SpeedController::disable_throttle() {
    this->throttle_pid.setCurrentCmd(0);
    this->brake_active = true;
}
}  // namespace phnx_control