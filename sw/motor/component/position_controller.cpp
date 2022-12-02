#include "position_controller.hpp"

namespace wwMotor
{
	float PositionController::update(float position_ref, uint8_t direction, float position_mea)
	{
		return _pid_pos.update(position_ref, position_mea);
	};

	void PositionController::_config_apply()
	{
		float k = (float)_motor.polePair * _motor.flux / _motor.interia * 3.0f / 2.0f;

		auto cfg = PidControllerConfig{ .mode = PidControllerMode::Serial,
			.Kp = _config.bandWidth / _config.delta / k,
			.Ki = _config.bandWidth / _config.delta / _config.delta,
			.Kd = 0,
			.tau = 0,
			.output_limit_enable = true,
			.output_limit_max = _motor.i_phase_limit,
			.output_limit_min = -_motor.i_phase_limit,
			.integrator_limit_enable = true,
			.integrator_limit_max = _motor.i_phase_limit,
			.integrator_limit_min = -_motor.i_phase_limit,
			.sample_time = _config.sample_time };
		_pid_pos.config_set(cfg);
	};
} // namespace wwMotor
