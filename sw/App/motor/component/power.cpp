#include "power.hpp"

namespace wwMotor
{

float SimplePower::ubus_get()
{
    return config.ubus;
};

float VbusSensorPower::ubus_get()
{
    return 0;
};

} // namespace wwMotor
