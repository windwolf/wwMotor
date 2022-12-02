#include "section_switcher.hpp"
#include "motor/base.hpp"

namespace wwMotor
{
	using namespace wwControl;
	void SectionSwitcher::_config_apply()
	{
		auto filterCfg = FirstOrderLowPassFilterConfig{
			.sample_time = _config.sample_time,
			.cutoff_freq = _config.cutoff_freq,
		};
		_filter.config_set(filterCfg);
	}

	bool SectionSwitcher::section_switch_needed(uint8_t section, bool zero_across)
	{
		_tick++;
		if (_last_section != section)
		{
			_last_section_switch_tick = _tick;
		}

		if (zero_across)
		{
			_zero_cross_flag = true;
			uint32_t period = _tick - _last_zero_cross_tick;
			_speed = _filter.filter(_PI_3 / (float)period);
			_last_zero_cross_tick = _tick;
		}

		if (_zero_cross_flag &&
			(_tick - _last_zero_cross_tick) >= (_last_zero_cross_tick - _last_section_switch_tick))
		{
			_zero_cross_flag = false;
			return true;
		}
		return false;
	};

	float SectionSwitcher::speed_get()
	{
		return _speed;
	};

} // namespace wwMotor
