//
// Created by zhouj on 2022/11/17.
//

#ifndef WWMOTOR_APP_MOTOR2_DQCURRENTCONTROLLER_HPP_
#define WWMOTOR_APP_MOTOR2_DQCURRENTCONTROLLER_HPP_

#include "motor2/DqVoltageReferenceUpdater.hpp"
#include "motor2/base.hpp"
#include "pid.hpp"
namespace wibot::motor {
using namespace wibot::control;
struct DqCurrentControllerConfig {
    union {
        struct {
            float p;
            float i;
            float d;
        } simple;
        struct {
            /**
             * 电流环的控制带宽. 典型值为: Fs*2PI/20.
             */
            float bandWidth;
        } params;
    };
    bool            useParams;
    /**
     * 是否禁用前馈控制. 默认为: false.
     */
    bool            disableFeedforward;
    float           sample_time;
    MotorParameter* motor_parameter;
};
class PidWithFeedforwardCurrentController : public DqVoltageReferenceUpdater,
                                            public Configurable<DqCurrentControllerConfig> {
   public:
    Result apply_config() override;
    void   dq_voltage_update(wibot::motor::Motor& motor, Vector2<float>& v_dq) override;

   private:
    PidController pid_d;
    PidController pid_q;
};

}  // namespace wibot::motor

#endif  // WWMOTOR_APP_MOTOR2_DQCURRENTCONTROLLER_HPP_
