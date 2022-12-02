//
// Created by zhouj on 2022/11/18.
//

#ifndef WWMOTOR_APP_MOTOR2_DRIVEREXECUTOR_HPP_
#define WWMOTOR_APP_MOTOR2_DRIVEREXECUTOR_HPP_

#include "base.hpp"

namespace wwMotor2
{

	class DriverExecutor
	{
	 public:
		virtual void duty_set(Vector3f duty) = 0;
		virtual void breakdown() = 0;
		virtual void resume() = 0;
		virtual void charge_prepare() = 0;
		virtual void channel_ctrl(Vector3b channel) = 0;
	};

} // wwMotor2

#endif //WWMOTOR_APP_MOTOR2_DRIVEREXECUTOR_HPP_
