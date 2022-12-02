//
// Created by zhouj on 2022/11/21.
//

#ifndef WWMOTOR_APP_MOTOR2_FLUXOBSERVERWITHLOWSPEEDOPENPOSITIONSPEEDSENSOR_HPP_
#define WWMOTOR_APP_MOTOR2_FLUXOBSERVERWITHLOWSPEEDOPENPOSITIONSPEEDSENSOR_HPP_

#include "FluxObserverPositionSpeedSensor.hpp"
namespace wwMotor2
{

	struct FluxObserverWithLowSpeedOpenPositionSpeedSensorConfig
	{
		FluxObserverPositionSpeedSensorConfig config;
		float low_speed_threshold;
	};
	class FluxObserverWithLowSpeedOpenPositionSpeedSensor : public PositionSpeedSensor,
															public Configurable<
																FluxObserverWithLowSpeedOpenPositionSpeedSensorConfig>
	{

	 private:
		FluxObserverPositionSpeedSensor _inner_sensor;
	};

} // wwMotor2

#endif //WWMOTOR_APP_MOTOR2_FLUXOBSERVERWITHLOWSPEEDOPENPOSITIONSPEEDSENSOR_HPP_
