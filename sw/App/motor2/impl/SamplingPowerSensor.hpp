//
// Created by zhouj on 2022/11/16.
//

#ifndef WWMOTOR_APP_MOTOR2_SAMPLINGPOWERSENSOR_HPP_
#define WWMOTOR_APP_MOTOR2_SAMPLINGPOWERSENSOR_HPP_

#include "motor2/PowerSensor.hpp"
#include "accessor/LinearValueMapper.hpp"
#include "DataSource.hpp"

namespace wibot::motor {
using namespace wibot::accessor;
struct SamplingPowerSensorConfig {
    DataSource* u_bus;
    DataSource* i_bus;
    float       u_bus_pu;
    float       i_bus_pu;
};
class SamplingPowerSensor : public PowerSensor, public Configurable<SamplingPowerSensorConfig> {
   public:
    Result apply_config() override;
    void   u_bus_get(Motor& motor, float& u_bus) override;
    void   i_bus_get(Motor& motor, float& i_bus) override;

   private:
    LinearValueMapper u_bus_mapper;
    LinearValueMapper i_bus_mapper;
};

}  // namespace wibot::motor

#endif  // WWMOTOR_APP_MOTOR2_SAMPLINGPOWERSENSOR_HPP_
