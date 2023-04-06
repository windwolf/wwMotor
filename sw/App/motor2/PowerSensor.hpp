//
// Created by zhouj on 2022/11/16.
//

#ifndef WWMOTOR_APP_MOTOR2_POWERSENSOR_HPP_
#define WWMOTOR_APP_MOTOR2_POWERSENSOR_HPP_

#include "base.hpp"
namespace wibot::motor {

class PowerSensor {
   public:
    virtual void u_bus_get(Motor& motor, float& u_bus) = 0;
    virtual void i_bus_get(Motor& motor, float& i_bus) = 0;
};

}  // namespace wibot::motor

#endif  // WWMOTOR_APP_MOTOR2_POWERSENSOR_HPP_
