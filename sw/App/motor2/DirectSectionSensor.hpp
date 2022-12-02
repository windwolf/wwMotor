//
// Created by zhouj on 2022/11/17.
//

#ifndef WWMOTOR_APP_MOTOR2_DIRECTSECTIONSENSOR_HPP_
#define WWMOTOR_APP_MOTOR2_DIRECTSECTIONSENSOR_HPP_

#include "SectionSensor.hpp"
#include "base.hpp"
namespace wwMotor2
{

	class DirectSectionSensor : public SectionSensor
	{
	 public:
		void section_get(wwMotor2::Motor& motor, uint8_t& section) override;
		void section_index_calibrate(wwMotor2::Motor& motor) override;
	};

} // wwMotor2

#endif //WWMOTOR_APP_MOTOR2_DIRECTSECTIONSENSOR_HPP_
