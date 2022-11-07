#ifndef __wwMotor_SPEED_CONTROLLER_HPP__
#define __wwMotor_SPEED_CONTROLLER_HPP__

#include "motor/base.hpp"
#include "pid/pid.hpp"

namespace wwMotor
{

using namespace wwControl;

class SpeedController
{
  public:
    struct Config
    {
        Scalar bandWidth; // Typically: Fs*2PI/20. the same as current loop
        Scalar delta;     // The distance between system zero and speed loop's pole in log scale.
        Scalar sample_time;
    };

  public:
    SpeedController(Config &config, Motor &motor)
        : context(motor),
          pid_spd(PidController::Config{.mode = PidController::Mode::Serial,
                                        .Kd = 0,
                                        .tau = 0,
                                        .output_limit_enable = true,
                                        .output_limit_max = motor.i_phase_limit,
                                        .output_limit_min = -motor.i_phase_limit,
                                        .intergrator_limit_enable = true,
                                        .intergrator_limit_max = motor.i_phase_limit,
                                        .intergrator_limit_min = -motor.i_phase_limit,
                                        .sample_time = config.sample_time})
    {
        Scalar k = (Scalar)motor.polePair * motor.flux / motor.interia * 3.0f / 2.0f;
        pid_spd.config.Kp = config.bandWidth / config.delta / k;
        pid_spd.config.Ki = config.bandWidth / config.delta / config.delta;
    };

    Vector2f update(Scalar speed_ref, Scalar speed_mea);

  private:
    Motor &context;
    PidController pid_spd;

    float speed_m_ref;
};

} // namespace wwMotor

#endif // __wwMotor_SPEED_CONTROLLER_HPP__