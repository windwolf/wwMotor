//
// Created by zhouj on 2022/11/21.
//

#ifndef WWMOTOR_APP_MOTOR2_HALLSECTIONSENSOR_HPP_
#define WWMOTOR_APP_MOTOR2_HALLSECTIONSENSOR_HPP_

#include "motor2/SectionSensor.hpp"
namespace wibot::motor
{
	struct HallSectionSensorConfig
	{

	};

	class HallSectionSensor : public SectionSensor, Configurable<HallSectionSensorConfig>
	{
	 public:
		void section_get(Motor& motor, uint8_t& section) override;
	};

} // wibot::motor

#endif //WWMOTOR_APP_MOTOR2_HALLSECTIONSENSOR_HPP_
