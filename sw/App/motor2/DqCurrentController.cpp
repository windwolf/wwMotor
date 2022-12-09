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

		if (!config.disableFeedforward)
		{
			u_dq.v1 += -motor.state.i_dq.v2 * config.motor_parameter->lq * motor.state.pos_spd_e.v2;
			u_dq.v2 += +(motor.state.i_dq.v1 * config.motor_parameter->ld + config.motor_parameter->flux)
				* motor.state.pos_spd_e.v2;

		}
	}

	void DqCurrentController::config_apply(CurrentControllerConfig& config)
	{
		this->config = config;
		PidControllerConfig pidCfg;
		pidCfg.mode = PidControllerMode::Serial;
		pidCfg.Kp = config.bandWidth * config.motor_parameter->ld;
		pidCfg.Ki = config.motor_parameter->rs / config.motor_parameter->ld;
		pidCfg.Kd = 0;
		pidCfg.tau = 0;
		pidCfg.output_limit_enable = true;
		pidCfg.output_limit_max = config.motor_parameter->u_bus_max;
		pidCfg.output_limit_min = -config.motor_parameter->u_bus_max;
		pidCfg.integrator_limit_enable = true;
		pidCfg.integrator_limit_max = config.motor_parameter->u_bus_max;
		pidCfg.integrator_limit_min = -config.motor_parameter->u_bus_max;
		pidCfg.sample_time = config.sample_time;
		pid_d.config_apply(pidCfg);

		pidCfg.Kp = config.bandWidth * config.motor_parameter->lq;
		pidCfg.Ki = config.motor_parameter->rs / config.motor_parameter->lq;
		pid_q.config_apply(pidCfg);
	}
} // wibot::motor
