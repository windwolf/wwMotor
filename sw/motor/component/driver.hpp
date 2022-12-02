#ifndef __wwMotor_DRIVER_HPP__
#define __wwMotor_DRIVER_HPP__

#include "motor/base.hpp"
#include "motor/component/power.hpp"

namespace wwMotor
{

	class DriverExecutor
	{
	 public:
		virtual void duty_set(Vector3f duty) = 0;
		virtual void breakdown() = 0;
		virtual void resume() = 0;
		virtual void charge_prepare() = 0;
		virtual void channel_ctrl(Vector3b channel) = 0;
	};

	class Driver
	{
	 public:
	 public:
		Driver(DriverExecutor& executor) : _executor(executor)
		{
		};
		virtual Vector2f circle_limit(Vector2f u_ab, float ubus) = 0;
		virtual void phase_voltage_set(Vector2f u_ab, float ubus) = 0;

	 protected:
		DriverExecutor& _executor;
	};

	struct DriverSVPWMConfig
	{
		float maxModuleRate;
		float maxDModuleRate;
		bool enableDeadTime;
		float deadTime;
	};

/**
 * @brief SVPWM driver
 * primer vector: I = 100, II = 110, III = 010, IV = 011, V = 001, VI = 101
 *
 *
 */
	class DriverSVPWM : public Driver, public Configurable<DriverSVPWMConfig>
	{

	 public:
		DriverSVPWM(DriverExecutor& executor) : Driver(executor)
		{
		};
		void phase_voltage_set(Vector2f u_ab, float ubus) override;
		Vector2f circle_limit(Vector2f u_ab, float ubus) override;

		void _config_apply() override;

	 protected:

		float sqDModuleLimit; // (SQRT3_3 * max_d_module_rate)^2
		float sqModuleLimit;  // (SQRT3_3 * max_module_rate)^2
		float dModuleLimit;   // SQRT3_3 * max_d_module_rate

		uint8_t section;
		Vector3f u_abc;
		Vector3f duty;
	};

} // namespace wwMotor

#endif // __wwMotor_DRIVER_HPP__
