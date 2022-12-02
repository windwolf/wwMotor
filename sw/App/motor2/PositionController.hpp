//
// Created by zhouj on 2022/11/17.
//

#ifndef WWMOTOR_APP_MOTOR2_POSITIONCONTROLLER_HPP_
#define WWMOTOR_APP_MOTOR2_POSITIONCONTROLLER_HPP_

#include "base.hpp"
#include "pid.hpp"
namespace wwMotor2
{
	using namespace wwControl;
	struct PositionControllerConfig
	{
		float bandWidth;
		float delta;
		float sample_time;
		MotorParameter* motor_parameter;
	};
	class PositionController : public Configurable<PositionControllerConfig>
	{
	 public:
		void config_apply(PositionControllerConfig& config) override;
		void speed_get(Motor& motor, float& speed);
	 private:
		PidController _pid_pos;
	};

} // wwMotor2

#endif //WWMOTOR_APP_MOTOR2_POSITIONCONTROLLER_HPP_
