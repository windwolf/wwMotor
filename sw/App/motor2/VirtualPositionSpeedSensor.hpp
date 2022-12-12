//
// Created by zhouj on 2022/11/17.
//

#ifndef WWMOTOR_APP_MOTOR2_VIRTUALPOSITIONSPEEDSENSOR_HPP_
#define WWMOTOR_APP_MOTOR2_VIRTUALPOSITIONSPEEDSENSOR_HPP_

#include "PositionSpeedSensor.hpp"

namespace wibot::motor
{
	struct VirtualPositionSpeedSensorConfig
	{
		uint8_t polePairs;
		float sample_time;
	};

	class VirtualPositionSpeedSensor :
		public PositionSpeedSensor, public Configurable<VirtualPositionSpeedSensorConfig>
	{
	 public:
		void config_apply(VirtualPositionSpeedSensorConfig& config);

		void position_set(float position);

		void speed_set(float speed);

		void position_speed_get(Motor& motor, Vector2f& pos_spd_e, Vector2f& pos_spd_m) override;

		void calibrate(Motor& motor) override;
	 private:
		float _position;
		float _speed;
	};

} // wibot::motor

#endif //WWMOTOR_APP_MOTOR2_VIRTUALPOSITIONSPEEDSENSOR_HPP_
