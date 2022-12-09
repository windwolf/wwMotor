//
// Created by zhouj on 2022/11/28.
//

#include "BusCurrentController.hpp"

namespace wibot::motor
{
	void BusCurrentController::config_apply(BusCurrentControllerConfig& config)
	{
		this->config = config;
		control::PidControllerConfig pidCfg = {};
		pidCfg.mode = wibot::control::PidControllerMode::Serial;
		pidCfg.Kp = config.Kp;
		pidCfg.Ki = config.Ki;
		pidCfg.Kd = config.Kd;
		pidCfg.sample_time = config.sample_time;
		pidCfg.tau = config.tau;
		pidCfg.output_limit_enable = true;
		pidCfg.output_limit_max = 1;
		pidCfg.output_limit_min = 0;
		pidCfg.integrator_limit_enable = true;
		pidCfg.integrator_limit_max = 1;
		pidCfg.integrator_limit_min = 0;
		_pid.config_apply(pidCfg);

	}
	void BusCurrentController::duty_get(Motor& motor, float& duty)
	{
		duty = _pid.update(motor.reference.i_bus, motor.state.i_bus);

	}
} // wibot::motor
