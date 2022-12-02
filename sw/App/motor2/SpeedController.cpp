//
// Created by zhouj on 2022/11/17.
//

#include "SpeedController.hpp"

namespace wibot::motor
{
	void SpeedController::config_apply(SpeedControllerConfig& config)
	{
		Configurable::config_apply(config);
		float k =
			(float)config.motor_parameter->pole_pair * config.motor_parameter->flux / config.motor_parameter->interia
				* 3.0f / 2.0f;

		PidControllerConfig cfg{
			.mode = PidControllerMode::Serial,
			.Kp = _config.bandWidth / _config.delta / k,
			.Ki = _config.bandWidth / _config.delta / _config.delta,
			.Kd = 0,
			.tau = 0,
			.output_limit_enable = true,
			.output_limit_max = config.motor_parameter->speed_limit,
			.output_limit_min = -config.motor_parameter->speed_limit,
			.integrator_limit_enable = true,
			.integrator_limit_max = config.motor_parameter->speed_limit,
			.integrator_limit_min = -config.motor_parameter->speed_limit,
			.sample_time = _config.sample_time
		};
		_pid_spd.config_apply(cfg);
	}
	void SpeedController::current_get(Motor& motor, Vector2f& i_dq)
	{
		i_dq.v1 = 0.0f;
		i_dq.v2 = _pid_spd.update(motor.reference.speed, motor.state.pos_spd_m.v2);
	}

} // wibot::motor
