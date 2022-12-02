//
// Created by zhouj on 2022/11/28.
//

#ifndef WWMOTOR_APP_MOTOR2_PHASEVOLTAGESENSOR_HPP_
#define WWMOTOR_APP_MOTOR2_PHASEVOLTAGESENSOR_HPP_

#include "base.hpp"
namespace wwMotor2
{

	class PhaseVoltageSensor
	{
	 public:
		/**
		 * @brief Get the mid port voltage of the motor.
		 * @param motor
		 * @param u_abc
		 */
		virtual void u_abc_get(wwMotor2::Motor& motor, Vector3f& u_abc) = 0;
	};

} // wwMotor2

#endif //WWMOTOR_APP_MOTOR2_PHASEVOLTAGESENSOR_HPP_
