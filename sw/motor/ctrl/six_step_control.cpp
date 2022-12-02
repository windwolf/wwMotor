#include "six_step_control.hpp"

namespace wwMotor
{
	void SixStepControl::init()
	{
		_emf_sensor.config_set(_config.emf_sensor_config);
		_section_switcher.config_set(_config.section_switcher_config);
		_current_sensor.config_set(_config.curr_sensor_cfg);
		_spd_pid.config_set(_config.speed_pid_cfg);
		_curr_pid.config_set(_config.current_pid_cfg);
	};
	void SixStepControl::calibrate()
	{
		_driver.section_set(1, 0.1);
	};
	void SixStepControl::update()
	{
		_context.speed = _section_switcher.speed_get();
		float duty = 0.0f;
		if (_config.mode == Mode::Speed)
		{
			float i_ref = _spd_pid.update(_context.speed_ref, _context.speed);
			float i = _current_sensor.bus_current_get();

			duty = _curr_pid.update(i_ref, i);
			if (duty > 1)
			{
				duty = 1.0f;
			}
			else if (duty < -1)
			{
				duty = -1.0f;
			}
		}
		else if (_config.mode == Mode::Current)
		{
			float i = _current_sensor.bus_current_get();
			duty = _curr_pid.update(_context.current_ref, i);
		}

		bool zc = _emf_sensor.zero_cross_detect(_context.section);
		bool switch_cmd = _section_switcher.section_switch_needed(_context.section, zc);
		if (switch_cmd)
		{
			_context.section++;
			if (_context.section > 6)
			{
				_context.section = 1;
			}
			if (_context.section < 1)
			{
				_context.section = 6;
			}
		}
		_driver.section_set(_context.section, duty);
	};

	void SixStepControl::speed_set(float speed)
	{
		_context.speed_ref = speed;
	};

	void SixStepControl::current_set(float current)
	{
		_context.current_ref = current;
	};

} // namespace wwMotor
