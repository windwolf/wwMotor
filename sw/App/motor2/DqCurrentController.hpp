//
// Created by zhouj on 2022/11/17.
//

#ifndef WWMOTOR_APP_MOTOR2_DQCURRENTCONTROLLER_HPP_
#define WWMOTOR_APP_MOTOR2_DQCURRENTCONTROLLER_HPP_

#include "base.hpp"
#include "pid.hpp"
namespace wwMotor2
{
	using namespace wwControl;
	struct CurrentControllerConfig
	{
		/**
		 * 电流环的控制带宽. 典型值为: Fs*2PI/20.
		 */
		float bandWidth;
		/**
		 * 是否禁用前馈控制. 默认为: false.
		 */
		bool disableFeedforward;
		float sample_time;
		MotorParameter* motor_parameter;
	};
	class DqCurrentController : public Configurable<CurrentControllerConfig>
	{
	 public:
		void config_apply(CurrentControllerConfig& config) override;
		void voltage_get(wwMotor2::Motor& motor, Vector2<float>& u_dq);

	 private:
		PidController pid_d;
		PidController pid_q;

	};

} // wwMotor2

#endif //WWMOTOR_APP_MOTOR2_DQCURRENTCONTROLLER_HPP_
