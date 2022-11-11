#include "six_step_control.hpp"

namespace wwMotor
{
void SixStepControl::init(){

};
void SixStepControl::calibrate()
{
    _driver.section_set(1, 0.1);
};
void SixStepControl::update()
{
    _context.speed = _section_switcher.speed_get();
    Scalar duty = 0.0f;
    if (_config.mode == Mode::Speed)
    {
        Scalar i_ref = _spd_pid.update(_context.speed_ref, _context.speed);
        Scalar i = _current_sensor.bus_current_get();

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
        Scalar i = _current_sensor.bus_current_get();
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

void SixStepControl::speed_set(Scalar speed)
{
    _context.speed_ref = speed;
};

void SixStepControl::current_set(Scalar current)
{
    _context.current_ref = current;
};

} // namespace wwMotor