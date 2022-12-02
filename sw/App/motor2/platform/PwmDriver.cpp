#include "PwmDriver.hpp"

namespace wwMotor2
{
	void PwmDriver::duty_set(Motor& motor)
	{
		if (_breakdown_flag)
		{
			return;
		}
		auto res = pwm.config_get().fullScaleDuty;

		pwm.duty_set(_config.channel_a, motor.reference.d_abc.v1 * res);
		pwm.duty_set(_config.channel_b, motor.reference.d_abc.v2 * res);
		pwm.duty_set(_config.channel_c, motor.reference.d_abc.v3 * res);
		pwm.duty_set(_config.channel_s, motor.reference.d_sample * res);
		channel_ctrl(motor.reference.sw_channel);
	};

	void PwmDriver::channel_ctrl(uint8_t channel)
	{
		if (_breakdown_flag)
		{
			return;
		}
		if (channel & 0x01)
		{
			pwm.channel_enable(_config.channel_a);
		}
		else
		{
			pwm.channel_disable(_config.channel_a);
		}
		if (channel & 0x02)
		{
			pwm.channel_enable(_config.channel_b);
		}
		else
		{
			pwm.channel_disable(_config.channel_b);
		}
		if (channel & 0x04)
		{
			pwm.channel_enable(_config.channel_c);
		}
		else
		{
			pwm.channel_disable(_config.channel_c);
		}
		if (channel & 0x08)
		{
			pwm.channel_enable(_config.channel_s);
		}
		else
		{
			pwm.channel_disable(_config.channel_s);
		}
	};

	void PwmDriver::breakdown()
	{
		_breakdown_flag = true;
		pwm.all_disable();
	};
	void PwmDriver::resume()
	{
		_breakdown_flag = false;
		pwm.all_enable();
	};
	void PwmDriver::charge_prepare()
	{
		if (_breakdown_flag)
		{
			return;
		}
		pwm.all_disable();
		pwm.duty_set(_config.channel_a, 0.0f);
		pwm.duty_set(_config.channel_b, 0.0f);
		pwm.duty_set(_config.channel_c, 0.0f);
		pwm.all_enable();
	};
} // namespace wwMotor
