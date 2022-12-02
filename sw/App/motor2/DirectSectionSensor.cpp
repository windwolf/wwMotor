//
// Created by zhouj on 2022/11/17.
//

#include "DirectSectionSensor.hpp"

namespace wwMotor2
{
	void DirectSectionSensor::section_get(Motor& motor, uint8_t& section)
	{
		section = FocMath::section_get(motor.state.pos_spd_e.v1);
	}
	void DirectSectionSensor::section_index_calibrate(Motor& motor)
	{

	}
} // wwMotor2
