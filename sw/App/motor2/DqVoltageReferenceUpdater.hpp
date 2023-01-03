//
// Created by zhouj on 2022/12/13.
//

#ifndef WWMOTOR_APP_MOTOR2_DQVOLTAGEREFERENCEUPDATER_HPP_
#define WWMOTOR_APP_MOTOR2_DQVOLTAGEREFERENCEUPDATER_HPP_
#include "base.hpp"

namespace wibot::motor
{
	class DqVoltageReferenceUpdater
	{
	 public:
		virtual void dq_voltage_update(Motor& motor, Vector2f& v_dq) = 0;
	};
} // wibot::motor
#endif //WWMOTOR_APP_MOTOR2_DQVOLTAGEREFERENCEUPDATER_HPP_
