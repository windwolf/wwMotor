#ifndef __WWCONTROL_FOC_BASE_HPP__
#define __WWCONTROL_FOC_BASE_HPP__

#include "basic/math_shared.hpp"

namespace wwMotor
{
struct MotorState
{
    float u_bus; // bus voltage
    float i_bus; // bus current

    Vector2f pos_spd_e; // position and speed in electrical domain
    Vector2f pos_spd_m; // position and speed in mechanical domain
    uint8_t section; // section of electrical position

    Vector3f u_m_abc; // port voltage

    Vector3f i_abc; // phase current
    Vector2f i_ab;
    Vector2f i_dq;

    float speed;
    float position;
    float current;

    Vector2f i_dq_ref;
    float speed_ref;
    float position_ref;

    Vector2f u_dq_ref;
    Vector2f u_ab_ref;
};

struct MotorParameter
{
  public:
    enum class FluxSetMode
    {
        Flux,
        BackEmfConstant,
    };

  public:
    // Config or parameters
    uint8_t polePair;
    float rs;
    float ld;
    float lq;
    float flux;

    float interia;
    float friction;

    // Limits
    float speed_limit;
    float u_bus_max;
    float i_bus_limit;
    float i_phase_limit;
};

} // namespace wwMotor

#endif // __WWCONTROL_FOC_BASE_HPP__
