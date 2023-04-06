//
// Created by zhouj on 2022/12/13.
//

#ifndef WWMOTOR_APP_MOTOR2_BUSDUTYREFERENCEUPDATER_HPP_
#define WWMOTOR_APP_MOTOR2_BUSDUTYREFERENCEUPDATER_HPP_
#include "base.hpp"

namespace wibot::motor {
class BusDutyReferenceUpdater {
   public:
    virtual void dbus_update(Motor& motor, float& d_bus) = 0;
};
}  // namespace wibot::motor
#endif  // WWMOTOR_APP_MOTOR2_BUSDUTYREFERENCEUPDATER_HPP_
