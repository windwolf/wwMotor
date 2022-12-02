//
// Created by zhouj on 2022/11/17.
//

#ifndef WWMOTOR_APP_MOTOR2_SECTIONSENSOR_HPP_
#define WWMOTOR_APP_MOTOR2_SECTIONSENSOR_HPP_

#include "base.hpp"
namespace wwMotor2
{

	class SectionSensor
	{

	 public:
		virtual void section_get(wwMotor2::Motor& motor, uint8_t& section) = 0;

		virtual void section_index_calibrate(wwMotor2::Motor& motor) = 0;
	};

} // wwMotor2

#endif //WWMOTOR_APP_MOTOR2_SECTIONSENSOR_HPP_
