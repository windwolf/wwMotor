#ifndef __wwMotor_SPEED_CONTROLLER_HPP__
#define __wwMotor_SPEED_CONTROLLER_HPP__

#include "motor/base.hpp"
#include "pid/pid.hpp"

namespace wwMotor
{

	using namespace wwControl;

	struct SpeedControllerConfig
	{
		float bandWidth; // Typically: Fs*2PI/20. the same as current loop
		float delta;     // The distance between system zero and speed loop's pole in log scale.
		float sample_time;
	};

	class SpeedController : public Configurable<SpeedControllerConfig>
	{
	 public:
		SpeedController(MotorParameter& motor)
			: _motor(motor)
		{
		};

		Vector2f update(float speed_ref, float speed_mea);

	 protected:
		void _config_apply() override;

	 private:
		MotorParameter& _motor;
		PidController pid_spd;

		float speed_m_ref;
	};

} // namespace wwMotor

#endif // __wwMotor_SPEED_CONTROLLER_HPP__
