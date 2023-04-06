//
// Created by zhouj on 2022/11/17.
//

#ifndef WWMOTOR_APP_MOTOR2_POSITIONCONTROLLER_HPP_
#define WWMOTOR_APP_MOTOR2_POSITIONCONTROLLER_HPP_

#include "motor2/base.hpp"
#include "pid.hpp"
#include "motor2/SpeedReferenceUpdater.hpp"
namespace wibot::motor {
using namespace wibot::control;
struct PositionControllerConfig {
    float           kp;
    float           ki;
    float           kd;
    float           sample_time;
    MotorParameter* motor_parameter;
};
class PidPositionController : public SpeedReferenceUpdater,
                              public Configurable<PositionControllerConfig> {
   public:
    Result apply_config() override;
    void   speed_update(Motor& motor, float& speed) override;

   private:
    PidController _pid_pos;
};

}  // namespace wibot::motor

#endif  // WWMOTOR_APP_MOTOR2_POSITIONCONTROLLER_HPP_
