//
// Created by zhouj on 2022/11/28.
//

#ifndef WWMOTOR_APP_MOTOR2_PHASEVOLTAGESENSOR_HPP_
#define WWMOTOR_APP_MOTOR2_PHASEVOLTAGESENSOR_HPP_

#include "base.hpp"
namespace wibot::motor
{

	class PhaseVoltageSensor
	{
	 public:
		/**
		 * @brief Get the mid port voltage of the motor.
		 * @param motor
		 * @param u_abc
		 */
		virtual void u_abc_get(wibot::motor::Motor& motor, Vector3f& u_abc) = 0;
	};

} // wibot::motor

#endif //WWMOTOR_APP_MOTOR2_PHASEVOLTAGESENSOR_HPP_
