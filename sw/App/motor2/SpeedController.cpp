//
// Created by zhouj on 2022/11/17.
//

#include "SpeedController.hpp"

namespace wibot::motor
{
	void SpeedController::config_apply(SpeedControllerConfig& config)
	{
		this->config = config;
		float k =
			(float)config.motor_parameter->pole_pair * config.motor_parameter->flux / config.motor_parameter->interia
				* 3.0f / 2.0f;
		PidControllerConfig pidCfg;
		pidCfg.mode = PidControllerMode::Serial;
		pidCfg.Kp = config.bandWidth / config.delta / k;
		pidCfg.Ki = config.bandWidth / config.delta / config.delta;
		pidCfg.Kd = 0;
		pidCfg.tau = 0;
		pidCfg.output_limit_enable = true;
		pidCfg.output_limit_max = config.motor_parameter->speed_limit / 60 * _2PI;
		pidCfg.output_limit_min = -config.motor_parameter->speed_limit / 60 * _2PI;
		pidCfg.integrator_limit_enable = true;
		pidCfg.integrator_limit_max = config.motor_parameter->speed_limit / 60 * _2PI;
		pidCfg.integrator_limit_min = -config.motor_parameter->speed_limit / 60 * _2PI;
		pidCfg.sample_time = config.sample_time;
		_pid_spd.config_apply(pidCfg);
	}
	void SpeedController::current_get(Motor& motor, Vector2f& i_dq)
	{
		i_dq.v1 = 0.0f;
		i_dq.v2 = _pid_spd.update(motor.reference.speed, motor.state.pos_spd_m.v2);
	}

} // wibot::motor
