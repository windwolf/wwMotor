//
// Created by zhouj on 2022/11/17.
//

#include "EncoderPositionSpeedSensor.hpp"
#include "math_shared.hpp"

namespace wwMotor2
{
	void EncoderPositionSpeedSensor::config_apply(EncoderPositionSpeedSensorConfig& config)
	{
		Configurable::config_apply(config);
		_half_resolution = config.resolution >> 1;
		_2pi_res_ts = _2PI / (float)_config.resolution / _config.sample_time;
		_2pipp_res_ts = _2PI * config.pole_pairs / (float)_config.resolution / _config.sample_time;

		LinearValueMapperConfig adjcfg{
			.value_per_unit = _2PI / (float)config.resolution,
		};
		_mapper.config_apply(adjcfg);

		FirstOrderLowPassFilterConfig lpcfg{
			.sample_time = config.sample_time,
			.cutoff_freq = config.mech_speed_cutoff_freq,
		};
		_filter_m.config_apply(lpcfg);

		lpcfg.cutoff_freq = config.mech_speed_cutoff_freq * (float)config.pole_pairs;
		_filter_e.config_apply(lpcfg);
	}

	void EncoderPositionSpeedSensor::position_speed_get(Motor& motor, Vector2f& pos_spd_e, Vector2f& pos_spd_m)
	{
		uint16_t pos_raw = *_config.encoder_buffer;
		if (pos_raw > _zero_index)
		{
			pos_raw -= _zero_index;
		}
		else
		{
			pos_raw += _config.resolution - _zero_index;
		}

		// calculate no-wrap position
		int32_t pos_raw_diff = (int32_t)pos_raw - (int32_t)_last_pos_raw;
		if (pos_raw_diff > _half_resolution)
		{
			pos_raw_diff -= _config.resolution;
		}
		else if (pos_raw_diff < -_half_resolution)
		{
			pos_raw_diff += _config.resolution;
		}

		_last_pos_raw = pos_raw;

		float pos_rad = _mapper.value_get(pos_raw);
		pos_spd_m.v1 = Math::circle_normalize(pos_rad);
		pos_spd_e.v1 = Math::circle_normalize(pos_rad * _config.pole_pairs);

		pos_spd_m.v2 = _filter_m.filter(pos_raw_diff * _2pi_res_ts);
		pos_spd_e.v2 = _filter_e.filter(pos_raw_diff * _2pipp_res_ts);
	}

	void EncoderPositionSpeedSensor::zero_search(Motor& motor)
	{
		_zero_index = *_config.encoder_buffer;
	};
} // wwMotor2
