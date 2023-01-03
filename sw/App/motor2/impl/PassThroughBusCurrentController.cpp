//
// Created by zhouj on 2022/12/13.
//

#include "PassThroughBusCurrentController.hpp"

namespace wibot
{
	namespace motor
	{
		void PassThroughBusCurrentController::dbus_update(Motor& motor, float& d_bus)
		{
			d_bus = motor.reference.i_bus * config.motor_parameter->rs;
		}
	} // wibot
} // motor
