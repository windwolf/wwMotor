//
// Created by zhouj on 2022/11/17.
//

#ifndef WWMOTOR_APP_MOTOR2_SPEEDCONTROLLER_HPP_
#define WWMOTOR_APP_MOTOR2_SPEEDCONTROLLER_HPP_

#include "base.hpp"
#include "pid.hpp"
namespace wibot::motor
{
	using namespace wibot::control;
	struct SpeedControllerConfig
	{
		/**
		 * 电流环的控制带宽. 没错, 是电流环, 而不是速度环的带宽.
		 */
		float bandWidth;

		/**
		 * 阻尼比. 速度环零极点间隔的对数表征
		 */
		float delta;
		float sample_time;
		MotorParameter* motor_parameter;
	};
	class SpeedController : public Configurable<SpeedControllerConfig>
	{
	 public:
		void apply_config() override;
		void current_get(Motor& motor, Vector2f& i_dq);
	 private:
		PidController _pid_spd;
	};

} // wibot::motor

#endif //WWMOTOR_APP_MOTOR2_SPEEDCONTROLLER_HPP_
