//
// Created by zhouj on 2022/11/17.
//

#ifndef WWMOTOR_APP_MOTOR2_SECTIONSENSOR_HPP_
#define WWMOTOR_APP_MOTOR2_SECTIONSENSOR_HPP_

#include "base.hpp"
namespace wibot::motor {

class SectionSensor {
   public:
    virtual void section_get(wibot::motor::Motor& motor, uint8_t& section) = 0;
};

}  // namespace wibot::motor

#endif  // WWMOTOR_APP_MOTOR2_SECTIONSENSOR_HPP_
