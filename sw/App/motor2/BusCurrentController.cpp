//
// Created by zhouj on 2022/11/28.
//

#include "BusCurrentController.hpp"

namespace wibot::motor
{
	void BusCurrentController::config_apply(BusCurrentControllerConfig& config)
	{
		this->config = config;
		_pid.config.mode = wibot::control::PidControllerMode::Serial;
		_pid.config.Kp = config.Kp;
		_pid.config.Ki = config.Ki;
		_pid.config.Kd = config.Kd;
		_pid.config.sample_time = config.sample_time;
		_pid.config.tau = config.tau;
		_pid.config.output_limit_enable = true;
		_pid.config.output_limit_max = 1;
		_pid.config.output_limit_min = 0;
		_pid.config.integrator_limit_enable = true;
		_pid.config.integrator_limit_max = 1;
		_pid.config.integrator_limit_min = 0;
		
	}
	void BusCurrentController::duty_get(Motor& motor, float& duty)
	{
		duty = _pid.update(motor.reference.i_bus, motor.state.i_bus);

	}
} // wibot::motor
