#ifndef __wwMotor_CURRENT_CONTROLLER_HPP__
#define __wwMotor_CURRENT_CONTROLLER_HPP__

#include "motor/base.hpp"
#include "pid/pid.hpp"

namespace wwMotor
{
using namespace wwControl;

class CurrentController
{
  public:
    struct DirectConfig
    {
        Scalar pid_d_kp;
        Scalar pid_d_ki;
        Scalar pid_q_kp;
        Scalar pid_q_ki;
        bool enableFeedforward;
        Scalar sample_time;
    };
    struct SampleConfig
    {
        Scalar bandWidth; // Typically: Fs*2PI/20
        bool enableFeedforward;
        Scalar sample_time;
    };

  private:
    struct Config
    {
        bool enableFeedforward;
    };

  public:
    CurrentController(const SampleConfig &config, Motor &motor)
        : config{.enableFeedforward = config.enableFeedforward}, motor(motor),
          pid_d(PidController::Config{
              .mode = PidController::Mode::Serial,
              .Kp = config.bandWidth * motor.ld,
              .Ki = motor.rs / motor.ld,
              .Kd = 0,
              .tau = 0,
              .output_limit_enable = true,
              .output_limit_max = motor.u_bus_max,
              .output_limit_min = -motor.u_bus_max,
              .intergrator_limit_enable = true,
              .intergrator_limit_max = motor.u_bus_max,
              .intergrator_limit_min = -motor.u_bus_max,
              .sample_time = config.sample_time,
          }),
          pid_q(PidController::Config{
              .mode = PidController::Mode::Serial,
              .Kp = config.bandWidth * motor.lq,
              .Ki = motor.rs / motor.lq,
              .Kd = 0,
              .tau = 0,
              .output_limit_enable = true,
              .output_limit_max = motor.u_bus_max,
              .output_limit_min = -motor.u_bus_max,
              .intergrator_limit_enable = true,
              .intergrator_limit_max = motor.u_bus_max,
              .intergrator_limit_min = -motor.u_bus_max,
              .sample_time = config.sample_time,
          }){

          };

    CurrentController(Motor &motor, const DirectConfig &config)
        : config{.enableFeedforward = config.enableFeedforward}, motor(motor),
          pid_d(PidController::Config{
              .mode = PidController::Mode::Serial,
              .Kp = config.pid_d_kp,
              .Ki = config.pid_d_ki,
              .Kd = 0,
              .tau = 0,
              .output_limit_enable = true,
              .output_limit_max = motor.u_bus_max,
              .output_limit_min = -motor.u_bus_max,
              .intergrator_limit_enable = true,
              .intergrator_limit_max = motor.u_bus_max,
              .intergrator_limit_min = -motor.u_bus_max,
              .sample_time = config.sample_time,
          }),
          pid_q(PidController::Config{
              .mode = PidController::Mode::Serial,
              .Kp = config.pid_q_kp,
              .Ki = config.pid_q_ki,
              .Kd = 0,
              .tau = 0,
              .output_limit_enable = true,
              .output_limit_max = motor.u_bus_max,
              .output_limit_min = -motor.u_bus_max,
              .intergrator_limit_enable = true,
              .intergrator_limit_max = motor.u_bus_max,
              .intergrator_limit_min = -motor.u_bus_max,

              .sample_time = config.sample_time,
          }){

          };

    Vector2f torque_to_current(Scalar torque);
    Vector2f update(Vector2f i_dq_ref, Vector2f I_dq_mea, Scalar speed_e);

  protected:
    Config config;
    Motor &motor;
    PidController pid_d;
    PidController pid_q;
    Vector2f feedforward(Vector2f i_dq, Scalar speed_e);
};

} // namespace wwMotor

#endif // __wwMotor_CURRENT_CONTROLLER_HPP__