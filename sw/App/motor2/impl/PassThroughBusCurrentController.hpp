//
// Created by zhouj on 2022/12/13.
//

#ifndef WWMOTOR_APP_MOTOR2_IMPL_PASSTHROUGHBUSCURRENTCONTROLLER_HPP_
#define WWMOTOR_APP_MOTOR2_IMPL_PASSTHROUGHBUSCURRENTCONTROLLER_HPP_

#include "base.hpp"
#include "motor2/base.hpp"
#include "motor2/BusDutyReferenceUpdater.hpp"
namespace wibot {
namespace motor {
struct PassThroughBusCurrentControllerConfig {
    MotorParameter* motor_parameter;
};

class PassThroughBusCurrentController : public BusDutyReferenceUpdater,
                                        public Configurable<PassThroughBusCurrentControllerConfig> {
   public:
    void dbus_update(Motor& motor, float& d_bus) override;
};

}  // namespace motor
}  // namespace wibot

#endif  // WWMOTOR_APP_MOTOR2_IMPL_PASSTHROUGHBUSCURRENTCONTROLLER_HPP_
