//
// Created by zhouj on 2022/12/13.
//

#include "DirectPidSpeedController.hpp"
namespace wibot::motor {

void DirectPidSpeedController::ibus_update(Motor& motor, float& i_bus) {
    i_bus = _pid.update(motor.reference.speed, motor.state.speed.v2);
}
Result DirectPidSpeedController::apply_config() {
    _pid.config.mode                    = PidControllerMode::Serial;
    _pid.config.Kp                      = config.kp;
    _pid.config.Ki                      = config.ki;
    _pid.config.Kd                      = config.kd;
    _pid.config.integrator_limit_enable = true;
    _pid.config.integrator_limit_max    = config.motor_parameter->i_bus_limit * 2;
    _pid.config.integrator_limit_min    = 0;
    _pid.config.output_limit_enable     = true;
    _pid.config.output_limit_max        = config.motor_parameter->i_bus_limit;
    _pid.config.output_limit_min        = 0;
    _pid.config.sample_time             = config.sample_time;
    return _pid.apply_config();
}

}  // namespace wibot::motor
