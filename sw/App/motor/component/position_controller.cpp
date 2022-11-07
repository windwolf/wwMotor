#include "position_controller.hpp"

namespace wwMotor
{
Scalar PositionController::update(Scalar position_ref, uint8_t direction, Scalar position_mea)
{
    return pid_pos.update(position_ref, position_mea);
};
} // namespace wwMotor
