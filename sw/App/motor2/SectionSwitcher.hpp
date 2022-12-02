//
// Created by zhouj on 2022/11/22.
//

#ifndef WWMOTOR_APP_MOTOR2_SECTIONSWITCHER_HPP_
#define WWMOTOR_APP_MOTOR2_SECTIONSWITCHER_HPP_

#include "base.hpp"
namespace wwMotor2
{

	class SectionSwitcher
	{
	 public:
		virtual void section_switch(Motor& motor, uint8_t& section) = 0;
	};

} // wwMotor2

#endif //WWMOTOR_APP_MOTOR2_SECTIONSWITCHER_HPP_
