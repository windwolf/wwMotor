//
// Created by zhouj on 2022/11/17.
//

#ifndef WWMOTOR_APP_MOTOR2_POSITIONCONTROLLER_HPP_
#define WWMOTOR_APP_MOTOR2_POSITIONCONTROLLER_HPP_

#include "base.hpp"
#include "pid.hpp"
namespace wibot::motor
{
	using namespace wibot::control;
	struct PositionControllerConfig
	{
		float kp;
		float ki;
		float kd;
		float sample_time;
		MotorParameter* motor_parameter;
	};
	class PositionController : public Configurable<PositionControllerConfig>
	{
	 public:
		void apply_config() override;
		void speed_get(Motor& motor, float& speed);
	 private:
		PidController _pid_pos;
	};

} // wibot::motor

#endif //WWMOTOR_APP_MOTOR2_POSITIONCONTROLLER_HPP_
