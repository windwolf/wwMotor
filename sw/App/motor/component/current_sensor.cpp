#include "current_sensor.hpp"

namespace wwMotor
{
Vector3f CurrentSensor3Shunt::phase_current_get(uint8_t section)
{
    Vector3f current;
    current.v1 = (float)config.rawDataBuffer.data[config.a_phase_idx] * _currentPerUnit;
    current.v2 = (float)config.rawDataBuffer.data[config.b_phase_idx] * _currentPerUnit;
    current.v3 = (float)config.rawDataBuffer.data[config.c_phase_idx] * _currentPerUnit;
    switch (section)
    {
    case 0:
        current.v1 = current.v1;
        current.v2 = current.v2;
        current.v3 = -(current.v1 + current.v2);
        break;
    case 1:
        current.v1 = current.v2;
        current.v2 = current.v3;
        current.v3 = -(current.v2 + current.v3);
        break;
    case 2:
        current.v1 = current.v3;
        current.v2 = current.v1;
        current.v3 = -(current.v3 + current.v1);
        break;

    default:
        break;
    }
    return current;
};

} // namespace wwMotor
