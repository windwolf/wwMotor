//
// Created by zhouj on 2022/11/28.
//

#ifndef WWMOTOR_APP_MOTOR2_DUTYCURRENTCONTROLLER_HPP_
#define WWMOTOR_APP_MOTOR2_DUTYCURRENTCONTROLLER_HPP_

#include "base.hpp"
#include "pid.hpp"
namespace wwMotor2
{
	struct DutyCurrentControllerConfig
	{
		float Kp;
		float Ki;
		float Kd;

		/* Derivative low-pass filter time constant */
		float tau;

		/* Sample time (in seconds) */
		float sample_time;
	};
	class DutyCurrentController : public Configurable<DutyCurrentControllerConfig>
	{
	 public:
		void config_apply(DutyCurrentControllerConfig& config);

		void update(Motor& motor, float& duty);

	 private:
		wwControl::PidController _pid;
	};

} // wwMotor2

#endif //WWMOTOR_APP_MOTOR2_DUTYCURRENTCONTROLLER_HPP_
