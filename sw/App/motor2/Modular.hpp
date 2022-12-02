//
// Created by zhouj on 2022/11/17.
//

#ifndef WWMOTOR_APP_MOTOR2_INVERTERDRIVER_HPP_
#define WWMOTOR_APP_MOTOR2_INVERTERDRIVER_HPP_

#include "base.hpp"
namespace wwMotor2
{

	class InverterDriver
	{
	 public:
		virtual void update_by_dq(wwMotor2::Motor& motor) = 0;
		virtual void update_by_ab(wwMotor2::Motor& motor) = 0;
	};

} // wwMotor2

#endif //WWMOTOR_APP_MOTOR2_INVERTERDRIVER_HPP_
