//
// Created by zhouj on 2022/11/28.
//

#ifndef WWMOTOR_APP_MOTOR2_BUSCURRENTCONTROLLER_HPP_
#define WWMOTOR_APP_MOTOR2_BUSCURRENTCONTROLLER_HPP_

#include "base.hpp"
#include "pid.hpp"
namespace wwMotor2
{
	struct BusCurrentControllerConfig
	{
		float Kp;
		float Ki;
		float Kd;

		/* Derivative low-pass filter time constant */
		float tau;

		/* Sample time (in seconds) */
		float sample_time;
	};
	class BusCurrentController : public Configurable<BusCurrentControllerConfig>
	{
	 public:
		void config_apply(BusCurrentControllerConfig& config);

		void duty_get(Motor& motor, float& duty);

	 private:
		wwControl::PidController _pid;
	};

} // wwMotor2

#endif //WWMOTOR_APP_MOTOR2_BUSCURRENTCONTROLLER_HPP_
