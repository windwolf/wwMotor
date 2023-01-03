//
// Created by zhouj on 2022/12/13.
//

#ifndef WWMOTOR_APP_MOTOR2_SPEEDREFERENCEUPDATER_HPP_
#define WWMOTOR_APP_MOTOR2_SPEEDREFERENCEUPDATER_HPP_
#include "base.hpp"

namespace wibot::motor
{
	class SpeedReferenceUpdater
	{
	 public:
		virtual void speed_update(Motor& motor, float& speed) = 0;
	};

}// wibot::motor
#endif //WWMOTOR_APP_MOTOR2_SPEEDREFERENCEUPDATER_HPP_
