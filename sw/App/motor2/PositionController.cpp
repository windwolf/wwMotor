//
// Created by zhouj on 2022/11/17.
//

#include "PositionController.hpp"

namespace wibot::motor
{
	void PositionController::config_apply(PositionControllerConfig& config)
	{
		Configurable::config_apply(config);
		// TODO: implement
//		float k =
//			(float)config.motor_parameter->pole_pair * config.motor_parameter->flux / config.motor_parameter->interia
//				* 3.0f / 2.0f;
//
//		PidControllerConfig cfg{
//			.mode = PidControllerMode::Serial,
//			.Kp = _config.bandWidth / _config.delta / k,
//			.Ki = _config.bandWidth / _config.delta / _config.delta,
//			.Kd = 0,
//			.tau = 0,
//			.output_limit_enable = true,
//			.output_limit_max = config.motor_parameter->speed_limit,
//			.output_limit_min = -config.motor_parameter->speed_limit,
//			.integrator_limit_enable = true,
//			.integrator_limit_max = config.motor_parameter->speed_limit,
//			.integrator_limit_min = -config.motor_parameter->speed_limit,
//			.sample_time = _config.sample_time
//		};
//		_pid_pos.config_apply(cfg);
	};

	void PositionController::speed_get(Motor& motor, float& speed)
	{
		speed = _pid_pos.update(motor.reference.position, motor.state.pos_spd_m.v1);
	}

} // wibot::motor
