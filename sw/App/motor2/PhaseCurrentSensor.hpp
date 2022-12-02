//
// Created by zhouj on 2022/11/17.
//

#ifndef WWMOTOR_APP_MOTOR2_PHASECURRENTSENSOR_HPP_
#define WWMOTOR_APP_MOTOR2_PHASECURRENTSENSOR_HPP_

#include "base.hpp"
namespace wwMotor2
{

	class PhaseCurrentSensor
	{
	 public:

		/**
		 * @brief Get the phase current of the motor.
		 * @param motor
		 * @param i_abc
		 */
		virtual void i_abc_get(wwMotor2::Motor& motor, Vector3f& i_abc) = 0;
		virtual void zero_calibrate(wwMotor2::Motor& motor) = 0;
	};

} // wwMotor2

#endif //WWMOTOR_APP_MOTOR2_PHASECURRENTSENSOR_HPP_
