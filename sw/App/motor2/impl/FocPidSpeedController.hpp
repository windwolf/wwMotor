//
// Created by zhouj on 2022/11/17.
//

#ifndef WWMOTOR_APP_MOTOR2_SPEEDCONTROLLER_HPP_
#define WWMOTOR_APP_MOTOR2_SPEEDCONTROLLER_HPP_

#include "motor2/base.hpp"
#include "pid.hpp"
#include "motor2/DqCurrentReferenceUpdater.hpp"
namespace wibot::motor
{
	using namespace wibot::control;
	struct FocPidControllerConfig
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
	class FocPidSpeedController
		: public DqCurrentReferenceUpdater,
		  public Configurable<FocPidControllerConfig>
	{
	 public:
		Result apply_config() override;
		void dq_current_update(Motor& motor, Vector2f& i_dq) override;
	 private:
		PidController _pid_spd;
	};

} // wibot::motor

#endif //WWMOTOR_APP_MOTOR2_SPEEDCONTROLLER_HPP_
