//
// Created by zhouj on 2022/11/17.
//

#ifndef WWMOTOR_APP_MOTOR2_DIRECTSECTIONSENSOR_HPP_
#define WWMOTOR_APP_MOTOR2_DIRECTSECTIONSENSOR_HPP_

#include "SectionSensor.hpp"
#include "base.hpp"
namespace wibot::motor
{

	class DirectSectionSensor : public SectionSensor
	{
	 public:
		void section_get(wibot::motor::Motor& motor, uint8_t& section) override;
		void section_index_calibrate(wibot::motor::Motor& motor) override;
	};

} // wibot::motor

#endif //WWMOTOR_APP_MOTOR2_DIRECTSECTIONSENSOR_HPP_
