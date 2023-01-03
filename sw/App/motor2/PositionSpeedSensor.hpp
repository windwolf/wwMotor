//
// Created by zhouj on 2022/11/17.
//

#ifndef WWMOTOR_APP_MOTOR2_POSITIONSPEEDSENSOR_HPP_
#define WWMOTOR_APP_MOTOR2_POSITIONSPEEDSENSOR_HPP_

#include "base.hpp"

namespace wibot::motor
{

	class PositionSpeedSensor
	{
	 public:
		virtual void position_speed_get(Motor& motor, Vector2f& position, Vector2f& speed) = 0;

		virtual void calibrate(Motor& motor) = 0;
	};

} // wibot::motor

#endif //WWMOTOR_APP_MOTOR2_POSITIONSPEEDSENSOR_HPP_
