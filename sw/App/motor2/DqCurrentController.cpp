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
		pid_d.config.mode = PidControllerMode::Serial;
		pid_d.config.Kp = config.bandWidth * config.motor_parameter->ld;
		pid_d.config.Ki = config.motor_parameter->rs / config.motor_parameter->ld;
		pid_d.config.Kd = 0;
		pid_d.config.tau = 0;
		pid_d.config.output_limit_enable = true;
		pid_d.config.output_limit_max = config.motor_parameter->u_bus_max;
		pid_d.config.output_limit_min = -config.motor_parameter->u_bus_max;
		pid_d.config.integrator_limit_enable = true;
		pid_d.config.integrator_limit_max = config.motor_parameter->u_bus_max;
		pid_d.config.integrator_limit_min = -config.motor_parameter->u_bus_max;
		pid_d.config.sample_time = config.sample_time;

		pid_q.config = pid_d.config;
		pid_q.config.Kp = config.bandWidth * config.motor_parameter->lq;
		pid_q.config.Ki = config.motor_parameter->rs / config.motor_parameter->lq;

	}
} // wibot::motor
