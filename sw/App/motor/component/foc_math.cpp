#include "foc_math.hpp"

namespace wwMotor
{
void FocMath::abc2ab(Vector3f abc, Vector2f &ab)
{
    ab.v1 = abc.v1 - (abc.v2 + abc.v3) / 2.0f;
    ab.v2 = (abc.v2 - abc.v3) * _SQRT3_2;
};
void FocMath::ab2dq(Vector2f ab, Scalar theta, Vector2f &dq)
{
    Scalar sin;
    Scalar cos;
    Math::sincos(theta, &sin, &cos);
    dq.v1 = ab.v1 * cos + ab.v2 * sin;
    dq.v2 = ab.v2 * cos - ab.v1 * sin;
};

void FocMath::dq2ab(Vector2f dq, Scalar theta, Vector2f &ab)
{
    Scalar sin;
    Scalar cos;
    Math::sincos(theta, &sin, &cos);
    ab.v1 = dq.v1 * cos - dq.v2 * sin;
    ab.v2 = dq.v2 * cos + dq.v1 * sin;
};

uint8_t FocMath::section_get(Scalar theta)
{
    uint8_t section = 0;
    if (theta >= 0.0f && theta < _PI_3)
    {
        section = 1;
    }
    else if (theta >= _PI_3 && theta < _2PI_3)
    {
        section = 2;
    }
    else if (theta >= _2PI_3 && theta < _PI)
    {
        section = 3;
    }
    else if (theta >= _PI && theta < _4PI_3)
    {
        section = 4;
    }
    else if (theta >= _4PI_3 && theta < _5PI_3)
    {
        section = 5;
    }
    else if (theta >= _5PI_3 && theta < _2PI)
    {
        section = 6;
    }
    return section;
};

} // namespace wwMotor