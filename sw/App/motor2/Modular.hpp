//
// Created by zhouj on 2022/11/17.
//

#ifndef WWMOTOR_APP_MOTOR2_MODULAR_HPP_
#define WWMOTOR_APP_MOTOR2_MODULAR_HPP_

#include "base.hpp"
namespace wibot::motor {

class Modular {
   public:
    virtual void module(wibot::motor::Motor& motor, uint8_t& section, Vector3f& d_abc,
                        Vector3f& u_abc, uint8_t& channels, float& d_sample) = 0;
};

}  // namespace wibot::motor

#endif  // WWMOTOR_APP_MOTOR2_MODULAR_HPP_
