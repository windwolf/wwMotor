#ifndef __wwMotor_CURRENT_SENSOR_HPP__
#define __wwMotor_CURRENT_SENSOR_HPP__

#include "motor/base.hpp"
#include "base/base.hpp"
#include "buffer.hpp"

namespace wwMotor
{
using namespace ww;
class CurrentSensor
{
  public:
  protected:
};

class CurrentSensor3Shunt : public CurrentSensor
{
  public:
    struct Config
    {
        uint32_t maxScaleValue;
        float refVoltage;
        float shuntResistance;
        float ampGain;
        Buffer16 rawDataBuffer;
        uint8_t a_phase_idx;
        uint8_t b_phase_idx;
        uint8_t c_phase_idx;
    };

  public:
    CurrentSensor3Shunt(CurrentSensor3Shunt::Config config, Motor &motor)
        : CurrentSensor(), config(config), motor(motor)
    {
        _currentPerUnit = config.refVoltage / (float)config.maxScaleValue / config.shuntResistance /
                          config.ampGain;
    };
    Vector3f phase_current_get(uint8_t section);

  protected:
    CurrentSensor3Shunt::Config config;
    Motor &motor;
    float _currentPerUnit; // refVoltage / maxScaleValue / shuntResistance / ampGain
};

} // namespace wwMotor

#endif // __wwMotor_CURRENT_SENSOR_HPP__