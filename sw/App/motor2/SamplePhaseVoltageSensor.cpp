//
// Created by zhouj on 2022/11/28.
//

#include "SamplePhaseVoltageSensor.hpp"

namespace wibot::motor
{
	void SamplePhaseVoltageSensor::config_apply(SamplePhaseVoltageSensorConfig& config)
	{
		Configurable::config_apply(config);
		LinearValueMapperConfig cfg{
			.zero_offset = 0,
			.value_per_unit = config.u_value_per_unit,
		};
		_a_mapper.config_apply(cfg);
		_b_mapper.config_apply(cfg);
		_c_mapper.config_apply(cfg);

		FirstOrderLowPassFilterConfig fcfg;
		fcfg.cutoff_freq = config.cutoff_freq;
		fcfg.sample_time = config.sample_time;
		_a_filter.config_apply(fcfg);
		_b_filter.config_apply(fcfg);
		_c_filter.config_apply(fcfg);
	}

	void SamplePhaseVoltageSensor::u_abc_get(Motor& motor, Vector3f& u_abc)
	{
		Vector3f u;
		u.v1 = _a_mapper.value_get(*_config.u_a_buffer);
		u.v2 = _b_mapper.value_get(*_config.u_b_buffer);
		u.v3 = _c_mapper.value_get(*_config.u_c_buffer);
		u_abc.v1 = _a_filter.filter(u_abc.v1);
		u_abc.v2 = _b_filter.filter(u_abc.v2);
		u_abc.v3 = _c_filter.filter(u_abc.v3);
	}
} // wibot::motor
