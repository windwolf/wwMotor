//
// Created by zhouj on 2022/11/16.
//

#include "SamplingPowerSensor.hpp"

namespace wibot::motor
{
	Result SamplingPowerSensor::apply_config()
	{
		Result rst;
		if (config.u_bus == nullptr || config.i_bus == nullptr)
		{
			return Result::InvalidParameter;
		}
		u_bus_mapper.config.zero_offset = 0;
		u_bus_mapper.config.value_per_unit = config.u_bus_pu;
		rst = u_bus_mapper.apply_config();
		if (rst != Result::OK)
		{
			return rst;
		}
		i_bus_mapper.config.zero_offset = 0;
		i_bus_mapper.config.value_per_unit = config.i_bus_pu;
		return i_bus_mapper.apply_config();
	}

	void SamplingPowerSensor::u_bus_get(Motor& motor, float& u_bus)
	{
		if (config.u_bus != nullptr)
		{
			u_bus = u_bus_mapper.value_get(config.u_bus->get_data());
		}
	}

	void SamplingPowerSensor::i_bus_get(Motor& motor, float& i_bus)
	{
		if (config.i_bus != nullptr)
		{
			i_bus = i_bus_mapper.value_get(config.i_bus->get_data());
		}
	}

} // wibot::motor
