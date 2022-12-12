//
// Created by zhouj on 2022/11/16.
//

#include "ShuntPowerSensor.hpp"

namespace wibot::motor
{
	void ShuntPowerSensor::apply_config()
	{
		u_bus_mapper.config.zero_offset = 0;
		u_bus_mapper.config.value_per_unit = config._u_bus_value_per_unit;
		u_bus_mapper.apply_config();

		i_bus_mapper.config.zero_offset = 0;
		i_bus_mapper.config.value_per_unit = config._i_bus_value_per_unit;
		i_bus_mapper.apply_config();
	}
	void ShuntPowerSensor::u_bus_get(Motor& motor, float& u_bus)
	{
		u_bus = u_bus_mapper.value_get(*config.u_bus_buffer);
	}
	void ShuntPowerSensor::i_bus_get(Motor& motor, float& i_bus)
	{
		i_bus = i_bus_mapper.value_get(*config.i_bus_buffer);
	}

	void calibrate_begin();
	void calibrate_step(uint16_t raw_value);
	void calibrate_end();
} // wibot::motor
