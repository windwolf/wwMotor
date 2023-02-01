//
// Created by zhouj on 2023/1/3.
//

#include "SimpleCurrentController.hpp"

namespace wibot
{
    namespace motor
    {
        void SimpleCurrentController::dq_voltage_update(Motor& motor, Vector2f& v_dq)
        {
            v_dq = motor.reference.i_dq * config.rs;
        }
    } // wibot
} // motor
