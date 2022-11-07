#ifndef __WWMOTOR_POS_SPD_OBSERVER_HPP__
#define __WWMOTOR_POS_SPD_OBSERVER_HPP__

#include "motor/base.hpp"
#include "filter/lp.hpp"
#include "pid.hpp"

namespace wwMotor
{
using namespace wwControl;
class PositionSpeedObserver
{
  public:
    virtual Vector2f pos_spd_get(Vector3f uabc, Vector3f iabc) = 0;
};

class SmoPllPositionSpeedOboserver : public PositionSpeedObserver
{
  public:
    struct Config
    {
        Scalar sample_time;
        Scalar current_gain;
        Scalar disturbance_gain;
        Scalar cutoff_freq;
        Scalar pll_kp;
        Scalar pll_pi;
    };

  public:
    SmoPllPositionSpeedOboserver(Config &config, Motor &motor)
        : _config(config), motor(motor),
          _filter({.sample_time = config.sample_time, .cutoff_freq = config.cutoff_freq}),
          _pid({
              .mode = PidController::Mode::Serial,
              .Kp = config.pll_kp,
              .Ki = config.pll_pi,
              .Kd = 0,
              .tau = 0,
              .output_limit_enable = false,
              .intergrator_limit_enable = false,
              .sample_time = config.sample_time,
          })
    {
        init();
    };
    void init();
    Vector2f pos_spd_get(Vector3f uabc, Vector3f iabc) override;

  private:
    Config _config;
    Motor &motor;
    FirstOrderLowPassFilter _filter;
    PidController _pid;

    Vector2f _i;
    Vector2f _u;

    Vector2f _i_obs;
    Vector2f _zk;
    Vector2f _i_err;

    Vector2f _e_obs;

    Vector2f _pos_spd;

    float a;
    float b;

    void smo();
    void pll();
    void lpf();
};

} // namespace wwMotor

#endif // __WWMOTOR_POS_SPD_OBSERVER_HPP__