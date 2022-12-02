//
// Created by zhouj on 2022/11/17.
//

#ifndef WWMOTOR_APP_MOTOR2_PHASECURRENTSENSOR_HPP_
#define WWMOTOR_APP_MOTOR2_PHASECURRENTSENSOR_HPP_

#include "base.hpp"
namespace wibot::motor
{

	class PhaseCurrentSensor
	{
	 public:

		/**
		 * @brief Get the phase current of the motor.
		 * @param motor
		 * @param i_abc
		 */
		virtual void i_abc_get(wibot::motor::Motor& motor, Vector3f& i_abc) = 0;
		virtual void zero_calibrate(wibot::motor::Motor& motor) = 0;
	};

} // wibot::motor

#endif //WWMOTOR_APP_MOTOR2_PHASECURRENTSENSOR_HPP_
