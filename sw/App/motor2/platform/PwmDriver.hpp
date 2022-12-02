#ifndef __WWMOTOR_PLATFORM_PWM_DRIVER_EXECUTOR_HPP__
#define __WWMOTOR_PLATFORM_PWM_DRIVER_EXECUTOR_HPP__

#include "motor/base.hpp"
#include "motor2/DriverExecutor.hpp"
#include "pwm.hpp"

namespace wwMotor2
{
	using namespace ww::peripheral;

	struct PwmDriverExecutorConfig
	{
		uint32_t channel_a;
		uint32_t channel_b;
		uint32_t channel_c;
		uint32_t channel_s;
	};
	class PwmDriverExecutor : public DriverExecutor, public Configurable<PwmDriverExecutorConfig>
	{
	 public:
	 public:
		PwmDriverExecutor(Pwm& pwm) : pwm(pwm)
		{
		}
		void duty_set(Vector3f duty, float sample_window);
		void duty_set(Vector3f duty) override;
		void breakdown() override;
		void resume() override;
		void charge_prepare() override;
		void channel_ctrl(Vector3b channel) override;

	 private:
		Pwm& pwm;
		bool _breakdown_flag = false;
	};

} // namespace wwMotor

#endif // __WWMOTOR_PLATFORM_PWM_DRIVER_EXECUTOR_HPP__
