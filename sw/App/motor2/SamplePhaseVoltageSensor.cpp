//
// Created by zhouj on 2022/11/28.
//

#include "SamplePhaseVoltageSensor.hpp"

namespace wibot::motor
{
	void SamplePhaseVoltageSensor::config_apply(SamplePhaseVoltageSensorConfig& config)
	{
		this->config = config;
		_a_mapper.config.zero_offset = 0;
		_a_mapper.config.value_per_unit = config.u_value_per_unit;

		_b_mapper.config = _a_mapper.config;
		_c_mapper.config = _a_mapper.config;

		_a_filter.config.cutoff_freq = config.cutoff_freq;
		_a_filter.config.sample_time = config.sample_time;
		_a_filter.config_apply(_a_filter.config);
		_b_filter.config = _a_filter.config;
		_b_filter.config_apply(_b_filter.config);
		_c_filter.config = _a_filter.config;
		_c_filter.config_apply(_c_filter.config);
	}

	void SamplePhaseVoltageSensor::u_abc_get(Motor& motor, Vector3f& u_abc)
	{
		Vector3f u;
		u.v1 = _a_mapper.value_get(*config.u_a_buffer);
		u.v2 = _b_mapper.value_get(*config.u_b_buffer);
		u.v3 = _c_mapper.value_get(*config.u_c_buffer);
		u_abc.v1 = _a_filter.filter(u_abc.v1);
		u_abc.v2 = _b_filter.filter(u_abc.v2);
		u_abc.v3 = _c_filter.filter(u_abc.v3);
	}
} // wibot::motor
