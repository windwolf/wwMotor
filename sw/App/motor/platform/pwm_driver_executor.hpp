#ifndef __WWMOTOR_PLATFORM_PWM_DRIVER_EXECUTOR_HPP__
#define __WWMOTOR_PLATFORM_PWM_DRIVER_EXECUTOR_HPP__

#include "motor/base.hpp"
#include "motor/component/driver.hpp"
#include "peripheral/pwm.hpp"

namespace wwMotor
{
using namespace ww::peripheral;
class PwmDriverExecutor : public DriverExecutor
{
  public:
    struct Config
    {
        uint32_t channel_a;
        uint32_t channel_b;
        uint32_t channel_c;
    };

  public:
    PwmDriverExecutor(Config &config, Pwm &pwm) : config(config), pwm(pwm)
    {
    }
    void duty_set(Vector3f duty) override;
    void breakdown() override;
    void resume() override;
    void charge_prepare() override;
    void channel_ctrl(Vector3b channel) override;

  private:
    Config config;
    Pwm &pwm;
    bool _breakdown_flag = false;
};

} // namespace wwMotor

#endif // __WWMOTOR_PLATFORM_PWM_DRIVER_EXECUTOR_HPP__