#include "current_controller.hpp"

namespace wwMotor
{
	Vector2f CurrentController::torque_to_current(float torque)
	{
		return Vector2f(torque, 0);
	};

	void CurrentController::_config_apply()
	{
		auto cfg = PidControllerConfig{
			.mode = PidControllerMode::Serial,
			.Kp = _config.bandWidth * _motor.ld,
			.Ki = _motor.rs / _motor.ld,
			.Kd = 0,
			.tau = 0,
			.output_limit_enable = true,
			.output_limit_max = _motor.u_bus_max,
			.output_limit_min = -_motor.u_bus_max,
			.integrator_limit_enable = true,
			.integrator_limit_max = _motor.u_bus_max,
			.integrator_limit_min = -_motor.u_bus_max,
			.sample_time = _config.sample_time,
		};

		pid_d.config_set(cfg);
		cfg.Kp = _config.bandWidth * _motor.lq,
			cfg.Ki = _motor.rs / _motor.lq,
			pid_q.config_set(cfg);
	};
	Vector2f CurrentController::update(Vector2f i_dq_ref, Vector2f I_dq_mea, float speed_e)
	{
		Vector2f u_dq;
		u_dq.v1 = pid_d.update(i_dq_ref.v1, i_dq_ref.v1);
		u_dq.v2 = pid_q.update(i_dq_ref.v2, i_dq_ref.v2);

		if (_config.enableFeedforward)
		{
			Vector2f u_dq_ff = feedforward(I_dq_mea, speed_e);
			u_dq += u_dq_ff;
		}
		return u_dq;
	};

	Vector2f CurrentController::feedforward(Vector2f i_dq, float speed_e)
	{
		Vector2f u_dq_ff;

		u_dq_ff.v1 = -i_dq.v2 * _motor.lq * speed_e;
		u_dq_ff.v2 = +(i_dq.v1 * _motor.ld + _motor.flux) * speed_e;

		return u_dq_ff;
	};

} // namespace wwMotor
