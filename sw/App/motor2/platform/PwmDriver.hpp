#ifndef __WWMOTOR_PLATFORM_PWM_DRIVER_EXECUTOR_HPP__
#define __WWMOTOR_PLATFORM_PWM_DRIVER_EXECUTOR_HPP__

#include "motor2/base.hpp"
#include "motor2/Driver.hpp"
#include "pwm.hpp"

namespace wibot::motor
{
	using namespace wibot::peripheral;

	struct PwmDriverConfig
	{
		uint32_t channel_a;
		uint32_t channel_b;
		uint32_t channel_c;
		uint32_t channel_s;
		uint16_t fullScaleDuty;
	};
	class PwmDriver : public Driver, public Configurable<PwmDriverConfig>
	{
	 public:
	 public:
		PwmDriver(Pwm& pwm) : pwm(pwm)
		{
		}
		void config_apply(PwmDriverConfig& config);
		void duty_set(Motor& motor) override;
		void breakdown() override;
		void resume() override;
		void charge_prepare() override;
		void channel_ctrl(uint8_t channel);

	 private:
		Pwm& pwm;
		bool _breakdown_flag = false;
	};

} // namespace wwMotor

#endif // __WWMOTOR_PLATFORM_PWM_DRIVER_EXECUTOR_HPP__
