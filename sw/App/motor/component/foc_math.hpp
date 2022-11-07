#ifndef __WWMOTOR_FOC_MATH_HPP__
#define __WWMOTOR_FOC_MATH_HPP__

#include "motor/base.hpp"

namespace wwMotor
{
class FocMath
{
  public:
    static void abc2ab(Vector3f abc, Vector2f &ab);
    static void ab2dq(Vector2f ab, Scalar theta, Vector2f &dq);
    static void dq2ab(Vector2f dq, Scalar theta, Vector2f &ab);
    static uint8_t section_get(Scalar theta);
};
} // namespace wwMotor

#endif // __WWMOTOR_FOC_MATH_HPP__