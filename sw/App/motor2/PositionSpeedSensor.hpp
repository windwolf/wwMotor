//
// Created by zhouj on 2022/11/17.
//

#ifndef WWMOTOR_APP_MOTOR2_POSITIONSPEEDSENSOR_HPP_
#define WWMOTOR_APP_MOTOR2_POSITIONSPEEDSENSOR_HPP_

#include "base.hpp"

namespace wwMotor2
{

	class PositionSpeedSensor
	{
	 public:
		virtual void position_speed_get(Motor& motor, Vector2f& pos_spd_e, Vector2f& pos_spd_m) = 0;

		virtual void zero_search(Motor& motor) = 0;
	};

} // wwMotor2

#endif //WWMOTOR_APP_MOTOR2_POSITIONSPEEDSENSOR_HPP_
