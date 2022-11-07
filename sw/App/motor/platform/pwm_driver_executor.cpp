#include "pwm_driver_executor.hpp"

namespace wwMotor
{
void PwmDriverExecutor::duty_set(Vector3f duty)
{
    pwm.duty_set(config.channel_a, duty.v1);
    pwm.duty_set(config.channel_b, duty.v2);
    pwm.duty_set(config.channel_c, duty.v3);
};
void PwmDriverExecutor::breakdown()
{
    pwm.stop();
};
void PwmDriverExecutor::resume()
{
    pwm.start();
};
void PwmDriverExecutor::charge_prepare()
{
    pwm.stop();
    pwm.duty_set(config.channel_a, 0.0f);
    pwm.duty_set(config.channel_b, 0.0f);
    pwm.duty_set(config.channel_c, 0.0f);
    pwm.start();
};
} // namespace wwMotor