//
// Created by zhouj on 2022/11/17.
//

#include "VirtualPositionSpeedSensor.hpp"
#include "math_shared.hpp"

namespace wibot::motor
{
	void VirtualPositionSpeedSensor::position_set(float position)
	{
		_position = position;

	}
	void VirtualPositionSpeedSensor::speed_set(float speed)
	{
		_speed = speed;
	}
	void VirtualPositionSpeedSensor::position_speed_get(Motor& motor, Vector2f& pos_spd_e, Vector2f& pos_spd_m)
	{
		_position += _speed * _config.sample_time;
		_position = Math::circle_normalize(_position);
		pos_spd_m.v1 = _position;
		pos_spd_m.v2 = _speed;
		pos_spd_e.v1 = _position * _config.polePairs;
		pos_spd_e.v2 = _speed * _config.polePairs;

	}
} // wibot::motor
