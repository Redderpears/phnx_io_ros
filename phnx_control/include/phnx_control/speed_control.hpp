#pragma once

#include "control_toolbox/pid.hpp"

namespace phnx_control {

/// Controls vehicle speed using throttle and brakes.
class SpeedController {
public:
    enum class Actuator { Brake, Throttle };

private:
    control_toolbox::Pid throttle_pid;
    control_toolbox::Pid brake_pid;
    /// Stamp of the last feedback
    std::optional<rclcpp::Time> last_feedback;
    /// Flag indicating if the brake is set
    bool brake_active = false;
    /// Speed we want to reach
    double set_speed = 0.0;

    void disable_breaks();
    void disable_throttle();

public:
    explicit SpeedController();

    /// Updates the controller, and creates a new command. The returned tuple contains a
    /// 0.0-1.0 value for the actuator, as well as the enum initiating the actuator it applies to.
    std::tuple<double, Actuator> update(double speed, const rclcpp::Time& stamp);

    void update_set_speed(double speed);
};

}  // namespace phnx_control