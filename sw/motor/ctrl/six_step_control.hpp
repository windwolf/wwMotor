#ifndef __WWMOTOR_SIX_STEP_CONTROL_HPP__
#define __WWMOTOR_SIX_STEP_CONTROL_HPP__

#include "motor/base.hpp"
#include "motor/component/driver.hpp"
#include "motor/component/emf_sensor.hpp"
#include "motor/component/section_switcher.hpp"
#include "motor/component/six_step_driver.hpp"
#include "motor/component/current_sensor.hpp"
#include "pid.hpp"

namespace wwMotor
{
	using namespace wwControl;
/**
 * @brief
 * AB CB CA BA BC AC
 *
 */
	class SixStepControl
	{
	 public:
		enum class Mode
		{
			Speed,
			Current,
			OpenLoop,
		};
		struct Config
		{
			Mode mode;

			SixStepEmfSensorConfig emf_sensor_config;
			SectionSwitcherConfig section_switcher_config;

			CurrentSensor1ShuntConfig curr_sensor_cfg;

			PidControllerConfig speed_pid_cfg;
			PidControllerConfig current_pid_cfg;
		};

		struct Context
		{
			uint8_t section;
			float speed;

			float speed_ref;
			float current_ref;
		};

	 public:
		SixStepControl(Config& config, DriverExecutor& executor)
			: _config(config), _executor(executor), _driver(executor)
		{
		};

		void init();
		void calibrate();
		void update();

		void speed_set(float speed);
		void current_set(float current);

	 private:
		Config _config;
		DriverExecutor& _executor;
		SixStepEmfSensor _emf_sensor;
		SectionSwitcher _section_switcher;
		CurrentSensor1Shunt _current_sensor;
		SixStepDriver _driver;
		Context _context;
		PidController _spd_pid;
		PidController _curr_pid;
	};

} // namespace wwMotor

#endif // __WWMOTOR_SIX_STEP_CONTROL_HPP__
