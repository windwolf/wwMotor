//
// Created by zhouj on 2022/11/16.
//

#ifndef WWMOTOR_APP_MOTOR2_POWERSENSOR_CPP_SIMPLEPOWERSENSOR_HPP_
#define WWMOTOR_APP_MOTOR2_POWERSENSOR_CPP_SIMPLEPOWERSENSOR_HPP_

#include "motor2/PowerSensor.hpp"

namespace wibot::motor
{

	struct SimplePowerSensorConfig
	{
		float u_bus;
	};
	class SimplePowerSensor : public PowerSensor,
							  public Configurable<SimplePowerSensorConfig>
	{
	 public:
		void u_bus_get(Motor& motor, float& u_bus) override;
		void i_bus_get(Motor& motor, float& i_bus) override;
	};

}
#endif //WWMOTOR_APP_MOTOR2_POWERSENSOR_CPP_SIMPLEPOWERSENSOR_HPP_
