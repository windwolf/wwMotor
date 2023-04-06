//
// Created by zhouj on 2022/11/17.
//

#include "PidWithFeedforwardCurrentController.hpp"

#include "pid.hpp"

namespace wibot::motor {
using namespace wibot::control;
void PidWithFeedforwardCurrentController::dq_voltage_update(Motor& motor, Vector2<float>& u_dq) {
    Vector2f udq;
    udq.v1 = pid_d.update(motor.reference.i_dq.v1, motor.state.i_dq.v1);
    udq.v2 = pid_q.update(motor.reference.i_dq.v2, motor.state.i_dq.v2);

    if (!config.disableFeedforward) {
        udq.v1 += -motor.state.i_dq.v2 * config.motor_parameter->lq * motor.state.speed.v1;
        udq.v2 +=
            +(motor.state.i_dq.v1 * config.motor_parameter->ld + config.motor_parameter->flux) *
            motor.state.speed.v1;
    }
    u_dq = udq;
}

Result PidWithFeedforwardCurrentController::apply_config() {
    Result rst;
    pid_d.config.mode = PidControllerMode::Serial;
    if (config.useParams) {
        pid_d.config.Kp = config.params.bandWidth * config.motor_parameter->ld;
        pid_d.config.Ki = config.motor_parameter->rs / config.motor_parameter->ld;
        pid_d.config.Kd = 0;
    } else {
        pid_d.config.Kp = config.simple.p;
        pid_d.config.Ki = config.simple.i;
        pid_d.config.Kd = config.simple.d;
    }
    pid_d.config.tau                     = 0;
    pid_d.config.output_limit_enable     = true;
    pid_d.config.output_limit_max        = config.motor_parameter->u_bus_max;
    pid_d.config.output_limit_min        = -config.motor_parameter->u_bus_max;
    pid_d.config.integrator_limit_enable = true;
    pid_d.config.integrator_limit_max    = config.motor_parameter->u_bus_max;
    pid_d.config.integrator_limit_min    = -config.motor_parameter->u_bus_max;
    pid_d.config.sample_time             = config.sample_time;
    rst                                  = pid_d.apply_config();
    if (rst != Result::OK) {
        return rst;
    }
    pid_q.config.mode = PidControllerMode::Serial;
    if (config.useParams) {
        pid_q.config.Kp = config.params.bandWidth * config.motor_parameter->lq;
        pid_q.config.Ki = config.motor_parameter->rs / config.motor_parameter->lq;
        pid_q.config.Kd = 0;
    } else {
        pid_q.config.Kp = config.simple.p;
        pid_q.config.Ki = config.simple.i;
        pid_q.config.Kd = config.simple.d;
    }

    pid_q.config.tau                     = 0;
    pid_q.config.output_limit_enable     = true;
    pid_q.config.output_limit_max        = config.motor_parameter->u_bus_max;
    pid_q.config.output_limit_min        = -config.motor_parameter->u_bus_max;
    pid_q.config.integrator_limit_enable = true;
    pid_q.config.integrator_limit_max    = config.motor_parameter->u_bus_max;
    pid_q.config.integrator_limit_min    = -config.motor_parameter->u_bus_max;
    pid_q.config.sample_time             = config.sample_time;
    return pid_q.apply_config();
}

}  // namespace wibot::motor
