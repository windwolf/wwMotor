//
// Created by zhouj on 2022/11/16.
//

#include "SimplePowerSensor.hpp"
void wibot::motor::SimplePowerSensor::u_bus_get(wibot::motor::Motor& motor, float& u_bus) {
    u_bus = config.u_bus;
}
void wibot::motor::SimplePowerSensor::i_bus_get(wibot::motor::Motor& motor, float& i_bus) {
    i_bus = 0;
}
