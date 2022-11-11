#ifndef __WWMOTOR_SECTION_SWITCHER_HPP__
#define __WWMOTOR_SECTION_SWITCHER_HPP__

#include "motor/base.hpp"
#include "filter/lp.hpp"

namespace wwMotor
{
using namespace wwControl;

class SectionSwitcher
{
  public:
    struct Config
    {
        Scalar pole_pairs;
        Scalar sample_time;
        Scalar cutoff_freq;
    };

  public:
    SectionSwitcher(Config &config)
        : _config(config), _filter({
                               .sample_time = config.sample_time,
                               .cutoff_freq = config.cutoff_freq,
                           })
    {
    }
    bool section_switch_needed(uint8_t section, bool zero_across);
    Scalar speed_get();

  private:
    Config _config;
    uint8_t _last_section;
    uint32_t _last_section_switch_tick;
    uint32_t _last_zero_cross_tick;
    uint32_t _tick;
    FirstOrderLowPassFilter _filter;
    bool _zero_cross_flag;
    Scalar _speed;
};

} // namespace wwMotor

#endif // __WWMOTOR_SECTION_SWITCHER_HPP__