#include "current_sensor.hpp"

namespace wwMotor
{

void CurrentSensor1Shunt::init()
{
    _currentPerUnit =
        config.refVoltage / (float)config.maxScaleValue / config.shuntResistance / config.ampGain;
}

Scalar CurrentSensor1Shunt::bus_current_get()
{
    Scalar current = (float)config.bus_current_value * _currentPerUnit;

    return current;
};

} // namespace wwMotor
