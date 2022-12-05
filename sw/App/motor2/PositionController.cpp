//
// Created by zhouj on 2022/11/17.
//

#include "PositionController.hpp"

namespace wibot::motor
{
	void PositionController::config_apply(PositionControllerConfig& config)
	{
		this->config = config;
		_pid_pos.config.mode = PidControllerMode::Serial;
		_pid_pos.config.Kp = config.kp;
		_pid_pos.config.Ki = config.kp;
		_pid_pos.config.Kd = config.kp;
		_pid_pos.config.tau = 0;
		_pid_pos.config.output_limit_enable = true;
		_pid_pos.config.output_limit_max = config.motor_parameter->speed_limit;
		_pid_pos.config.output_limit_min = -config.motor_parameter->speed_limit;
		_pid_pos.config.integrator_limit_enable = true;
		_pid_pos.config.integrator_limit_max = config.motor_parameter->speed_limit;
		_pid_pos.config.integrator_limit_min = -config.motor_parameter->speed_limit;
		_pid_pos.config.sample_time = config.sample_time;

	};

	void PositionController::speed_get(Motor& motor, float& speed)
	{
		speed = _pid_pos.update(motor.reference.position, motor.state.pos_spd_m.v1);
	}

} // wibot::motor
