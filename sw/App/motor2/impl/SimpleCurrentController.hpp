//
// Created by zhouj on 2023/1/3.
//

#ifndef WWMOTOR_APP_MOTOR2_IMPL_SIMPLECURRENTCONTROLLER_HPP_
#define WWMOTOR_APP_MOTOR2_IMPL_SIMPLECURRENTCONTROLLER_HPP_

#include "motor2/DqVoltageReferenceUpdater.hpp"
namespace wibot
{
    namespace motor
    {
        struct SimpleCurrentControllerConfig
        {
            float rs;
        };

        class SimpleCurrentController
            : public DqVoltageReferenceUpdater,
              public Configurable<SimpleCurrentControllerConfig>
        {
         public:
            void dq_voltage_update(Motor& motor, Vector2f& v_dq) override;
        };

    } // wibot
} // motor

#endif //WWMOTOR_APP_MOTOR2_IMPL_SIMPLECURRENTCONTROLLER_HPP_
