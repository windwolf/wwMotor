//
// Created by zhouj on 2022/11/17.
//

#include "FocPidSpeedController.hpp"
namespace wibot::motor {

Result FocPidSpeedController::apply_config() {
    float k = (float)config.motor_parameter->pole_pair * config.motor_parameter->flux /
              config.motor_parameter->interia * 3.0f / 2.0f;
    _pid_spd.config.mode                    = PidControllerMode::Serial;
    _pid_spd.config.Kp                      = config.bandWidth / config.delta / k;
    _pid_spd.config.Ki                      = config.bandWidth / config.delta / config.delta;
    _pid_spd.config.Kd                      = 0;
    _pid_spd.config.tau                     = 0;
    _pid_spd.config.output_limit_enable     = true;
    _pid_spd.config.output_limit_max        = config.motor_parameter->speed_limit / 60 * _2PI;
    _pid_spd.config.output_limit_min        = -config.motor_parameter->speed_limit / 60 * _2PI;
    _pid_spd.config.integrator_limit_enable = true;
    _pid_spd.config.integrator_limit_max    = config.motor_parameter->speed_limit / 60 * _2PI;
    _pid_spd.config.integrator_limit_min    = -config.motor_parameter->speed_limit / 60 * _2PI;
    _pid_spd.config.sample_time             = config.sample_time;
    return _pid_spd.apply_config();
}
void FocPidSpeedController::dq_current_update(Motor& motor, Vector2f& i_dq) {
    i_dq.v1 = 0.0f;
    i_dq.v2 = _pid_spd.update(motor.reference.speed, motor.state.speed.v2);
}

}  // namespace wibot::motor
