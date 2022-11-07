#ifndef __wwMotor_POSITION_CONTROLLER_HPP__
#define __wwMotor_POSITION_CONTROLLER_HPP__

#include "motor/base.hpp"
#include "pid/pid.hpp"

namespace wwMotor
{

using namespace wwControl;

class PositionController
{
  public:
    struct Config
    {
        Scalar bandWidth;
        Scalar delta;
        Scalar sample_time;
    };

  public:
    PositionController(Config &config, Motor &motor)
        : context(motor),
          pid_pos(PidController::Config{.mode = PidController::Mode::Serial,
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
        pid_pos.config.Kp = config.bandWidth / config.delta / k;
        pid_pos.config.Ki = config.bandWidth / config.delta / config.delta;
    };

    Scalar update(Scalar position_ref, uint8_t direction, Scalar position_mea);

  private:
    Motor &context;
    PidController pid_pos;
};

} // namespace wwMotor

#endif // __wwMotor_POSITION_CONTROLLER_HPP__