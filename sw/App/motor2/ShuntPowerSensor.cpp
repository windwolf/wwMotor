//
// Created by zhouj on 2022/11/16.
//

#include "ShuntPowerSensor.hpp"

namespace wwMotor2
{
	void ShuntPowerSensor::config_apply(ShuntPowerSensorConfig& config)
	{
		Configurable::config_apply(config);
		auto cfg = LinearValueMapperConfig{
			.zero_offset = 0,
			.value_per_unit = config._u_bus_value_per_unit,
		};
		u_bus_mapper.config_apply(cfg);

		cfg.zero_offset = 0;
		cfg.value_per_unit = config._i_bus_value_per_unit;
		i_bus_mapper.config_apply(cfg);
	}
	void ShuntPowerSensor::u_bus_get(Motor& motor, float& u_bus)
	{
		u_bus = u_bus_mapper.value_get(*_config.u_bus_buffer);
	}
	void ShuntPowerSensor::i_bus_get(Motor& motor, float& i_bus)
	{
		i_bus = i_bus_mapper.value_get(*_config.i_bus_buffer);
	}

	void calibrate_begin();
	void calibrate_step(uint16_t raw_value);
	void calibrate_end();
} // wwMotor2
