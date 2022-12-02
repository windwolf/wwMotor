#ifndef __WW_MOTOR_SIX_STEP_DRIVER_HPP__
#define __WW_MOTOR_SIX_STEP_DRIVER_HPP__

#include "motor/base.hpp"
#include "motor/component/driver.hpp"

namespace wwMotor
{
class SixStepDriver
{
  public:
    SixStepDriver(DriverExecutor &executor) : _executor(executor){};
    void section_set(uint8_t section, float duty);

  private:
    DriverExecutor &_executor;
};

} // namespace wwMotor

#endif // __WW_MOTOR_SIX_STEP_DRIVER_HPP__