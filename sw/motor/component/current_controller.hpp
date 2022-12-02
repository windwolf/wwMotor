#ifndef __wwMotor_CURRENT_CONTROLLER_HPP__
#define __wwMotor_CURRENT_CONTROLLER_HPP__

#include "motor/base.hpp"
#include "pid/pid.hpp"

namespace wwMotor
{
	using namespace wwControl;
	struct CurrentControllerConfig
	{
		float bandWidth; // Typically: Fs*2PI/20
		bool enableFeedforward;
		float sample_time;
	};
	class CurrentController : public Configurable<CurrentControllerConfig>
	{
	 public:
		CurrentController(MotorParameter& motor) : _motor(motor)
		{
		};

		Vector2f torque_to_current(float torque);
		Vector2f update(Vector2f i_dq_ref, Vector2f I_dq_mea, float speed_e);

	 protected:
		void _config_apply() override;
	 protected:
		MotorParameter& _motor;
		PidController pid_d;
		PidController pid_q;
		Vector2f feedforward(Vector2f i_dq, float speed_e);
	};

} // namespace wwMotor

#endif // __wwMotor_CURRENT_CONTROLLER_HPP__
