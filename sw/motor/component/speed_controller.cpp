#include "speed_controller.hpp"

namespace wwMotor
{

	void SpeedController::_config_apply()
	{
		float k = (float)_motor.polePair * _motor.flux / _motor.interia * 3.0f / 2.0f;
		auto cfg = PidControllerConfig{
			.mode = PidControllerMode::Serial,
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
			.sample_time = _config.sample_time
		};
		pid_spd.config_set(cfg);
	}
	Vector2f SpeedController::update(float speed_ref, float speed_mea)
	{
		Vector2f i_ref;
		i_ref.v1 = 0.0f;
		i_ref.v2 = pid_spd.update(speed_ref, speed_mea);
		return i_ref;
	};
} // namespace wwMotor
