//
// Created by zhouj on 2022/11/28.
//

#ifndef WWMOTOR_APP_MOTOR2_BUSCURRENTCONTROLLER_HPP_
#define WWMOTOR_APP_MOTOR2_BUSCURRENTCONTROLLER_HPP_

#include "motor2/base.hpp"
#include "pid.hpp"
#include "motor2/BusDutyReferenceUpdater.hpp"
namespace wibot::motor {
struct BusCurrentControllerConfig {
    float Kp;
    float Ki;
    float Kd;

    /* Derivative low-pass filter time constant */
    float tau;

    /* Sample time (in seconds) */
    float sample_time;
};
class PidBusCurrentController : public BusDutyReferenceUpdater,
                                public Configurable<BusCurrentControllerConfig> {
   public:
    Result apply_config() override;

    void dbus_update(Motor& motor, float& duty) override;

   private:
    wibot::control::PidController _pid;
};

}  // namespace wibot::motor

#endif  // WWMOTOR_APP_MOTOR2_BUSCURRENTCONTROLLER_HPP_
