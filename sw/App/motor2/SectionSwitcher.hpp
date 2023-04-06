//
// Created by zhouj on 2022/11/22.
//

#ifndef WWMOTOR_APP_MOTOR2_SECTIONSWITCHER_HPP_
#define WWMOTOR_APP_MOTOR2_SECTIONSWITCHER_HPP_

#include "base.hpp"
namespace wibot::motor {

class SectionSwitcher {
   public:
    virtual void section_switch(Motor& motor, uint8_t& section) = 0;
};

}  // namespace wibot::motor

#endif  // WWMOTOR_APP_MOTOR2_SECTIONSWITCHER_HPP_
