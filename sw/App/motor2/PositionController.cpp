//
// Created by zhouj on 2022/11/17.
//

#include "PositionController.hpp"

namespace wibot::motor
{
	void PositionController::config_apply(PositionControllerConfig& config)
	{
		this->config = config;
		PidControllerConfig pidCfg;
		pidCfg.mode = PidControllerMode::Serial;
		pidCfg.Kp = config.kp;
		pidCfg.Ki = config.ki;
		pidCfg.Kd = config.kd;
		pidCfg.tau = 0;
		pidCfg.output_limit_enable = true;
		pidCfg.output_limit_max = config.motor_parameter->speed_limit / 60 * _2PI;
		pidCfg.output_limit_min = -config.motor_parameter->speed_limit / 60 * _2PI;
		pidCfg.integrator_limit_enable = true;
		pidCfg.integrator_limit_max = config.motor_parameter->speed_limit / 60 * _2PI;
		pidCfg.integrator_limit_min = -config.motor_parameter->speed_limit / 60 * _2PI;
		pidCfg.sample_time = config.sample_time;
		_pid_pos.config_apply(pidCfg);
	};

	void PositionController::speed_get(Motor& motor, float& speed)
	{
		speed = _pid_pos.update(motor.reference.position, motor.state.pos_spd_m.v1);
	}

} // wibot::motor
