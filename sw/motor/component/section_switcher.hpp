#ifndef __WWMOTOR_SECTION_SWITCHER_HPP__
#define __WWMOTOR_SECTION_SWITCHER_HPP__

#include "motor/base.hpp"
#include "filter/lp.hpp"

namespace wwMotor
{
	using namespace wwControl;
	struct SectionSwitcherConfig
	{
		float pole_pairs;
		float sample_time;
		float cutoff_freq;
	};
	class SectionSwitcher : public Configurable<SectionSwitcherConfig>
	{
	 public:
		bool section_switch_needed(uint8_t section, bool zero_across);
		float speed_get();

	 protected:
		void _config_apply() override;
	 private:
		uint8_t _last_section;
		uint32_t _last_section_switch_tick;
		uint32_t _last_zero_cross_tick;
		uint32_t _tick;
		FirstOrderLowPassFilter _filter;
		bool _zero_cross_flag;
		float _speed;
	};

} // namespace wwMotor

#endif // __WWMOTOR_SECTION_SWITCHER_HPP__
