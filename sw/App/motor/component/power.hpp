#ifndef __wwMotor_POWERCONTEXT_HPP__
#define __wwMotor_POWERCONTEXT_HPP__

#include "motor/base.hpp"

namespace wwMotor
{

class Power
{
  public:
    /**
     * @brief Get the bus voltage.
     *
     * @return Bus voltage.
     */
    virtual float ubus_get() = 0;
};

/**
 * @brief
 *
 */
class SimplePower : public Power
{
  public:
    struct Config
    {
        float ubus;
    };

  public:
    SimplePower(Config &&config) : config(config){};
    SimplePower(Config &config) : config(config){};
    float ubus_get() override;

  protected:
    Config config;
};

/**
 * @brief
 *
 */
class VbusSensorPower : public Power
{
  public:
    VbusSensorPower(){};
    float ubus_get() override;
};
} // namespace wwMotor

#endif // __wwMotor_POWERCONTEXT_HPP__