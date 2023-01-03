//
// Created by zhouj on 2022/12/13.
//

#ifndef WWMOTOR_APP_MOTOR2_IBUSREFERENCEUPDATER_HPP_
#define WWMOTOR_APP_MOTOR2_IBUSREFERENCEUPDATER_HPP_
#include "base.hpp"

namespace wibot::motor
{
	class BusCurrentReferenceUpdater
	{
	 public:
		virtual void ibus_update(Motor& motor, float& i_bus) = 0;
	};
} // wibot::motor
#endif //WWMOTOR_APP_MOTOR2_IBUSREFERENCEUPDATER_HPP_
