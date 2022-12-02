//
// Created by zhouj on 2022/11/17.
//

#ifndef WWMOTOR_APP_MOTOR2_MODULAR_HPP_
#define WWMOTOR_APP_MOTOR2_MODULAR_HPP_

#include "base.hpp"
namespace wwMotor2
{

	class Modular
	{
	 public:
		virtual void module(wwMotor2::Motor& motor) = 0;
	};

} // wwMotor2

#endif //WWMOTOR_APP_MOTOR2_MODULAR_HPP_
