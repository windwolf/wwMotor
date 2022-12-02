#include "six_step_driver.hpp"

namespace wwMotor
{
void SixStepDriver::section_set(uint8_t section, float duty)
{
    switch (section)
    {
    case 1: // AB
        _executor.channel_ctrl(Vector3b(1, 1, 0));
        _executor.duty_set(Vector3f(duty, 0, 0));
        break;
    case 2: // CB
        _executor.channel_ctrl(Vector3b(0, 1, 1));
        _executor.duty_set(Vector3f(0, 0, duty));
        break;
    case 3: // CA
        _executor.channel_ctrl(Vector3b(1, 0, 1));
        _executor.duty_set(Vector3f(0, 0, duty));
        break;
    case 4: // BA
        _executor.channel_ctrl(Vector3b(1, 1, 0));
        _executor.duty_set(Vector3f(0, duty, 0));
        break;
    case 5: // BC
        _executor.channel_ctrl(Vector3b(0, 1, 1));
        _executor.duty_set(Vector3f(0, duty, 0));
        break;
    case 6: // AC
        _executor.channel_ctrl(Vector3b(1, 0, 1));
        _executor.duty_set(Vector3f(duty, 0, 0));
        break;
    default:
        break;
    }
};

} // namespace wwMotor