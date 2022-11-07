#include "current_controller.hpp"

namespace wwMotor
{
Vector2f CurrentController::torque_to_current(Scalar torque)
{
    return Vector2f(torque, 0);
};

Vector2f CurrentController::update(Vector2f i_dq_ref, Vector2f I_dq_mea, Scalar speed_e)
{
    Vector2f u_dq;
    u_dq.v1 = pid_d.update(i_dq_ref.v1, i_dq_ref.v1);
    u_dq.v2 = pid_q.update(i_dq_ref.v2, i_dq_ref.v2);

    if (config.enableFeedforward)
    {
        Vector2f u_dq_ff = feedforward(I_dq_mea, speed_e);
        u_dq += u_dq_ff;
    }
    return u_dq;
};

Vector2f CurrentController::feedforward(Vector2f i_dq, Scalar speed_e)
{
    Vector2f u_dq_ff;

    u_dq_ff.v1 = -i_dq.v2 * motor.lq * speed_e;
    u_dq_ff.v2 = +(i_dq.v1 * motor.ld + motor.flux) * speed_e;

    return u_dq_ff;
};

} // namespace wwMotor