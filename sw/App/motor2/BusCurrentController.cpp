//
// Created by zhouj on 2022/11/28.
//

#include "BusCurrentController.hpp"

namespace wibot::motor
{
	void BusCurrentController::config_apply(BusCurrentControllerConfig& config)
	{
		Configurable::config_apply(config);
		wibot::control::PidControllerConfig cfg;
		cfg.mode = wibot::control::PidControllerMode::Serial;
		cfg.Kp = config.Kp;
		cfg.Ki = config.Ki;
		cfg.Kd = config.Kd;
		cfg.sample_time = config.sample_time;
		cfg.tau = config.tau;
		cfg.output_limit_enable = true;
		cfg.output_limit_max = 1;
		cfg.output_limit_min = 0;
		cfg.integrator_limit_enable = true;
		cfg.integrator_limit_max = 1;
		cfg.integrator_limit_min = 0;
		_pid.config_apply(cfg);
	}
	void BusCurrentController::duty_get(Motor& motor, float& duty)
	{
		duty = _pid.update(motor.reference.i_bus, motor.state.i_bus);

	}
} // wibot::motor
