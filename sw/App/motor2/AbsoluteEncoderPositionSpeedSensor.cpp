//
// Created by zhouj on 2022/11/17.
//

#include "AbsoluteEncoderPositionSpeedSensor.hpp"
#include "math_shared.hpp"
#include "os.hpp"

namespace wibot::motor
{
	using namespace wibot::os;

	void AbsoluteEncoderPositionSpeedSensor::config_apply(AbsoluteEncoderPositionSpeedSensorConfig& config)
	{
		this->config = config;
		_half_resolution = config.resolution >> 1;
		_2pi_res_ts = _2PI / (float)config.resolution / config.sample_time;
		_2pipp_res_ts = _2PI * config.pole_pairs / (float)config.resolution / config.sample_time;

		_mapper.config.value_per_unit = _2PI / (float)config.resolution;

		_filter_pos_m.config.sample_time = config.sample_time;
		_filter_pos_m.config.cutoff_freq = config.mech_pos_cutoff_freq;
		_filter_pos_m.config_apply(_filter_pos_m.config);

		_filter_pos_e.config.sample_time = config.sample_time;
		_filter_pos_e.config.cutoff_freq = config.mech_pos_cutoff_freq * (float)config.pole_pairs;
		_filter_pos_e.config_apply(_filter_pos_e.config);

		_filter_spd_m.config.sample_time = config.sample_time;
		_filter_spd_m.config.cutoff_freq = config.mech_speed_cutoff_freq;
		_filter_spd_m.config_apply(_filter_spd_m.config);

		_filter_spd_e.config.sample_time = config.sample_time;
		_filter_spd_e.config.cutoff_freq = config.mech_speed_cutoff_freq * (float)config.pole_pairs;
		_filter_spd_e.config_apply(_filter_spd_e.config);
	}

	void AbsoluteEncoderPositionSpeedSensor::position_speed_get(Motor& motor, Vector2f& pos_spd_e, Vector2f& pos_spd_m)
	{
		int32_t pos_raw = (int32_t)(*config.encoder_buffer);
		if (pos_raw > config.zero_index)
		{
			pos_raw -= config.zero_index;
		}
		else
		{
			pos_raw += config.resolution - config.zero_index;
		}
		if (config.direction == -1)
		{
			pos_raw = -pos_raw;
		}
		// calculate no-wrap position
		int32_t pos_raw_diff = pos_raw - (int32_t)_last_pos_raw;
		if (pos_raw_diff > _half_resolution)
		{
			pos_raw_diff -= config.resolution;
		}
		else if (pos_raw_diff < -_half_resolution)
		{
			pos_raw_diff += config.resolution;
		}

		_last_pos_raw = pos_raw;

		float pos_rad = _mapper.value_get(pos_raw);
		pos_spd_m.v1 = Math::circle_normalize(pos_rad);
		pos_spd_e.v1 = Math::circle_normalize(pos_rad * config.pole_pairs);

		pos_spd_m.v2 = _filter_spd_m.filter(pos_raw_diff * _2pi_res_ts);
		pos_spd_e.v2 = _filter_spd_e.filter(pos_raw_diff * _2pipp_res_ts);
	}

	void AbsoluteEncoderPositionSpeedSensor::zero_search(Motor& motor)
	{
		Utils::delay(10);
		config.zero_index = *config.encoder_buffer;
	};
} // wibot::motor
