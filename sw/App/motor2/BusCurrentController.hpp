//
// Created by zhouj on 2022/11/28.
//

#ifndef WWMOTOR_APP_MOTOR2_BUSCURRENTCONTROLLER_HPP_
#define WWMOTOR_APP_MOTOR2_BUSCURRENTCONTROLLER_HPP_

#include "base.hpp"
#include "pid.hpp"
namespace wibot::motor
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
		void apply_config() override;

		void duty_get(Motor& motor, float& duty);

	 private:
		wibot::control::PidController _pid;
	};

} // wibot::motor

#endif //WWMOTOR_APP_MOTOR2_BUSCURRENTCONTROLLER_HPP_
