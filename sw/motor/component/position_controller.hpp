#ifndef __wwMotor_POSITION_CONTROLLER_HPP__
#define __wwMotor_POSITION_CONTROLLER_HPP__

#include "motor/base.hpp"
#include "pid/pid.hpp"

namespace wwMotor
{

	using namespace wwControl;
	struct PositionControllerConfig
	{
		float bandWidth;
		float delta;
		float sample_time;
	};
	class PositionController : public Configurable<PositionControllerConfig>
	{

	 public:
		PositionController(MotorParameter& motor)
			: _motor(motor)
		{

		};

		float update(float position_ref, uint8_t direction, float position_mea);
		void _config_apply() override;
	 private:
		MotorParameter& _motor;
		PidController _pid_pos;
	};

} // namespace wwMotor

#endif // __wwMotor_POSITION_CONTROLLER_HPP__
