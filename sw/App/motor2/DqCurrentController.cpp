//
// Created by zhouj on 2022/11/17.
//

#include "DqCurrentController.hpp"
#include "pid.hpp"

namespace wibot::motor
{
	using namespace wibot::control;
	void DqCurrentController::voltage_get(Motor& motor, Vector2<float>& u_dq)
	{
		u_dq.v1 = pid_d.update(motor.reference.i_dq.v1, motor.state.i_dq.v1);
		u_dq.v2 = pid_q.update(motor.reference.i_dq.v2, motor.state.i_dq.v2);

		if (!_config.disableFeedforward)
		{
			u_dq.v1 += -motor.state.i_dq.v2 * _config.motor_parameter->lq * motor.state.pos_spd_e.v2;
			u_dq.v2 += +(motor.state.i_dq.v1 * _config.motor_parameter->ld + _config.motor_parameter->flux)
				* motor.state.pos_spd_e.v2;

		}
	}

	void DqCurrentController::config_apply(CurrentControllerConfig& config)
	{
		Configurable::config_apply(config);
		PidControllerConfig cfg = {
			.mode = PidControllerMode::Serial,
			.Kp = _config.bandWidth * config.motor_parameter->ld,
			.Ki = config.motor_parameter->rs / config.motor_parameter->ld,
			.Kd = 0,
			.tau = 0,
			.output_limit_enable = true,
			.output_limit_max = config.motor_parameter->u_bus_max,
			.output_limit_min = -config.motor_parameter->u_bus_max,
			.integrator_limit_enable = true,
			.integrator_limit_max = config.motor_parameter->u_bus_max,
			.integrator_limit_min = -config.motor_parameter->u_bus_max,
			.sample_time = _config.sample_time,
		};

		pid_d.config_apply(cfg);
		cfg.Kp = _config.bandWidth * config.motor_parameter->lq;
		cfg.Ki = config.motor_parameter->rs / config.motor_parameter->lq;
		pid_q.config_apply(cfg);
	}
} // wibot::motor
