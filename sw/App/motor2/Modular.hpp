//
// Created by zhouj on 2022/11/17.
//

#ifndef WWMOTOR_APP_MOTOR2_MODULAR_HPP_
#define WWMOTOR_APP_MOTOR2_MODULAR_HPP_

#include "base.hpp"
namespace wibot::motor
{

	class Modular
	{
	 public:
		virtual void module(wibot::motor::Motor& motor) = 0;
	};

} // wibot::motor

#endif //WWMOTOR_APP_MOTOR2_MODULAR_HPP_
