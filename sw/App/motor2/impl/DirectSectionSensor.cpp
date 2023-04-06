//
// Created by zhouj on 2022/11/17.
//

#include "DirectSectionSensor.hpp"

namespace wibot::motor {
void DirectSectionSensor::section_get(Motor& motor, uint8_t& section) {
    section = FocMath::section_get(motor.state.position.v1);
}

}  // namespace wibot::motor
