#ifndef __WWMOTOR_EMF_SERSOR_HPP__
#define __WWMOTOR_EMF_SERSOR_HPP__
#include "motor/base.hpp"
namespace wwMotor
{
class EmfSensor
{
  public:
    explicit EmfSensor();
    virtual Vector3f abc_get(uint8_t section) = 0;

  protected:
};
} // namespace wwMotor

#endif // __WWMOTOR_EMF_SERSOR_HPP__