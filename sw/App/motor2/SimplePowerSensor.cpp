//
// Created by zhouj on 2022/11/16.
//

#include "SimplePowerSensor.hpp"
void wwMotor2::SimplePowerSensor::u_bus_get(wwMotor2::Motor& motor, float& u_bus)
{
	u_bus = _config.u_bus;
}
void wwMotor2::SimplePowerSensor::i_bus_get(wwMotor2::Motor& motor, float& i_bus)
{
	i_bus = 0;
}
