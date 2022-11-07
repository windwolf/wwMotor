#include "speed_controller.hpp"

namespace wwMotor
{
Vector2f SpeedController::update(Scalar speed_ref, Scalar speed_mea)
{
    Vector2f i_ref;
    i_ref.v1 = 0.0f;
    i_ref.v2 = pid_spd.update(speed_ref, speed_mea);
    return i_ref;
};
} // namespace wwMotor