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
	void VirtualPositionSpeedSensor::position_speed_get(Motor& motor, Vector2f& position, Vector2f& speed)
	{
		_position += _speed * config.sample_time;
		_position = Math::circle_normalize(_position);
		position.v2 = _position;
		speed.v2 = _speed;
		position.v1 = _position * config.polePairs;
		speed.v1 = _speed * config.polePairs;

	}
	void VirtualPositionSpeedSensor::calibrate(Motor& motor)
	{

	}
} // wibot::motor
