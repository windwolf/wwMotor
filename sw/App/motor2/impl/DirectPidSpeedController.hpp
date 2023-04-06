//
// Created by zhouj on 2022/12/13.
//

#ifndef WWMOTOR_APP_MOTOR2_IMPL_DIRECTPIDSPEEDCONTROLLER_HPP_
#define WWMOTOR_APP_MOTOR2_IMPL_DIRECTPIDSPEEDCONTROLLER_HPP_

#include "base.hpp"
#include "motor2/BusCurrentReferenceUpdater.hpp"
#include "motor2/BusDutyReferenceUpdater.hpp"
#include "pid.hpp"
namespace wibot::motor {
using namespace wibot::control;
struct DirectPidSpeedControllerConfig {
    float           kp;
    float           ki;
    float           kd;
    float           sample_time;
    MotorParameter* motor_parameter;
};
class DirectPidSpeedController : public BusCurrentReferenceUpdater,
                                 public Configurable<DirectPidSpeedControllerConfig> {
   public:
    void   ibus_update(Motor& motor, float& i_bus) override;
    Result apply_config() override;

   private:
    control::PidController _pid;
};

}  // namespace wibot::motor

#endif  // WWMOTOR_APP_MOTOR2_IMPL_DIRECTPIDSPEEDCONTROLLER_HPP_
