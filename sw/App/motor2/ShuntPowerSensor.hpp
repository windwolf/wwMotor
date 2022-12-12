//
// Created by zhouj on 2022/11/16.
//

#ifndef WWMOTOR_APP_MOTOR2_SHUNTPOWERSENSOR_HPP_
#define WWMOTOR_APP_MOTOR2_SHUNTPOWERSENSOR_HPP_

#include "PowerSensor.hpp"
#include "accessor/LinearValueMapper.hpp"

namespace wibot::motor
{
	using namespace wibot::accessor;
	struct ShuntPowerSensorConfig
	{
		uint32_t* u_bus_buffer;
		uint32_t* i_bus_buffer;
		float _u_bus_value_per_unit;
		float _i_bus_value_per_unit;
	};
	class ShuntPowerSensor : public PowerSensor,
							 public Configurable<ShuntPowerSensorConfig>
	{
	 public:
		void apply_config() override;
		void u_bus_get(Motor& motor, float& u_bus) override;
		void i_bus_get(Motor& motor, float& i_bus) override;

	 private:
		LinearValueMapper u_bus_mapper;
		LinearValueMapper i_bus_mapper;
	};

} // wibot::motor

#endif //WWMOTOR_APP_MOTOR2_SHUNTPOWERSENSOR_HPP_
