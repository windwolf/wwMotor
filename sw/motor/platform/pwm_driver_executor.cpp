#include "pwm_driver_executor.hpp"

namespace wwMotor
{
void PwmDriverExecutor::duty_set(Vector3f duty)
{
    if (_breakdown_flag)
    {
        return;
    }
    pwm.duty_set(config.channel_a, duty.v1);
    pwm.duty_set(config.channel_b, duty.v2);
    pwm.duty_set(config.channel_c, duty.v3);
};

void PwmDriverExecutor::channel_ctrl(Vector3b channel)
{
    if (_breakdown_flag)
    {
        return;
    }
    if (channel.v1)
    {
        pwm.channel_enable(config.channel_a);
    }
    else
    {
        pwm.channel_disable(config.channel_a);
    }
    if (channel.v2)
    {
        pwm.channel_enable(config.channel_b);
    }
    else
    {
        pwm.channel_disable(config.channel_b);
    }
    if (channel.v3)
    {
        pwm.channel_enable(config.channel_c);
    }
    else
    {
        pwm.channel_disable(config.channel_c);
    }
};

void PwmDriverExecutor::breakdown()
{
    _breakdown_flag = true;
    pwm.all_disable();
};
void PwmDriverExecutor::resume()
{
    _breakdown_flag = false;
    pwm.all_enable();
};
void PwmDriverExecutor::charge_prepare()
{
    if (_breakdown_flag)
    {
        return;
    }
    pwm.all_disable();
    pwm.duty_set(config.channel_a, 0.0f);
    pwm.duty_set(config.channel_b, 0.0f);
    pwm.duty_set(config.channel_c, 0.0f);
    pwm.all_enable();
};
} // namespace wwMotor