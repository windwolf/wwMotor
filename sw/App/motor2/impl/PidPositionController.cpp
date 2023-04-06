//
// Created by zhouj on 2022/11/17.
//

#include "PidPositionController.hpp"

namespace wibot::motor {
Result PidPositionController::apply_config() {
    this->config                            = config;
    _pid_pos.config.mode                    = PidControllerMode::Serial;
    _pid_pos.config.Kp                      = config.kp;
    _pid_pos.config.Ki                      = config.ki;
    _pid_pos.config.Kd                      = config.kd;
    _pid_pos.config.tau                     = 0;
    _pid_pos.config.output_limit_enable     = true;
    _pid_pos.config.output_limit_max        = config.motor_parameter->speed_limit / 60 * _2PI;
    _pid_pos.config.output_limit_min        = -config.motor_parameter->speed_limit / 60 * _2PI;
    _pid_pos.config.integrator_limit_enable = true;
    _pid_pos.config.integrator_limit_max    = config.motor_parameter->speed_limit / 60 * _2PI;
    _pid_pos.config.integrator_limit_min    = -config.motor_parameter->speed_limit / 60 * _2PI;
    _pid_pos.config.sample_time             = config.sample_time;
    return _pid_pos.apply_config();
};

void PidPositionController::speed_update(Motor& motor, float& speed) {
    speed = _pid_pos.update(motor.reference.position, motor.state.position.v2);
}

}  // namespace wibot::motor
