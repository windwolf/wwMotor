//
// Created by zhouj on 2022/11/17.
//

#include "AbsoluteEncoderPositionSpeedSensor.hpp"
#include "math_shared.hpp"
#include "os.hpp"
#include "misc.hpp"

namespace wibot::motor
{
	using namespace wibot::os;

	void AbsoluteEncoderPositionSpeedSensor::config_apply(AbsoluteEncoderPositionSpeedSensorConfig& config)
	{
		this->config = config;

		this->_2PI_RES = _2PI / (float)config.resolution;
		this->_1_TS = 1.0f / config.sample_time;
		this->_PP_TS = (float)config.pole_pairs * this->_1_TS;

		LinearValueMapperConfig lvmCfg;
		lvmCfg.value_per_unit = this->_2PI_RES;
		lvmCfg.zero_offset = config.resolution / 2;
		mapper_.config_apply(lvmCfg);

		FirstOrderLowPassFilterConfig lpCfg;
		lpCfg.sample_time = config.sample_time;
		lpCfg.cutoff_freq = config.mech_pos_cutoff_freq * _2PI;
		lpCfg.enable_wrap = true;
		lpCfg.enable_wrap = _PI;
		filter_pos_m_.config_apply(lpCfg);
		lpCfg.cutoff_freq = config.mech_pos_cutoff_freq * (float)config.pole_pairs * _2PI;
		filter_pos_e_.config_apply(lpCfg);

		lpCfg.enable_wrap = false;
		lpCfg.wrap_value = 0;
		lpCfg.cutoff_freq = config.mech_speed_cutoff_freq;
		filter_spd_m_.config_apply(lpCfg);

		lpCfg.cutoff_freq = config.mech_speed_cutoff_freq * (float)config.pole_pairs * _2PI;
		filter_spd_e_.config_apply(lpCfg);

	}

	void AbsoluteEncoderPositionSpeedSensor::position_speed_get(Motor& motor, Vector2f& pos_spd_e, Vector2f& pos_spd_m)
	{
		//TODO: 电角度计算是否有误.
		int32_t pos_m_raw_i = (int32_t)(*config.encoder_buffer);
		if (pos_m_raw_i > config.zero_index)
		{
			pos_m_raw_i -= config.zero_index;
		}
		else
		{
			pos_m_raw_i += config.resolution - config.zero_index;
		}
		float pos_m_raw = mapper_.value_get(pos_m_raw_i); // in rad. -pi ~ pi
		if (config.direction == EncoderDirection::Reverse)
		{
			pos_m_raw = -pos_m_raw;
		}

		float pos_m_raw_diff = pos_m_raw - last_pos_m_raw_;
		last_pos_m_raw_ = pos_m_raw;
		if (pos_m_raw_diff > _PI)
		{
			pos_m_raw_diff -= _2PI;
			round_m_--;
		}
		else if (pos_m_raw_diff < -_PI)
		{
			pos_m_raw_diff += _2PI;
			round_m_++;
		}

		pos_m_raw = filter_pos_m_.filter(pos_m_raw);
		pos_spd_m.v1 = Math::circle_normalize(pos_m_raw + _PI);
		pos_spd_m.v2 = filter_spd_m_.filter(pos_m_raw_diff * _1_TS);

		float pos_e_raw = (pos_m_raw + _PI) * (float)config.pole_pairs;
		pos_spd_e.v1 = Math::circle_normalize(pos_e_raw);
		pos_spd_e.v2 = filter_spd_e_.filter(pos_m_raw_diff * _PP_TS);
	}

	void AbsoluteEncoderPositionSpeedSensor::zero_search(Motor& motor)
	{
		const uint16_t calibrationRound = 1000;
		uint32_t pos_sum = 0;
		for (int i = 0; i < calibrationRound; i++)
		{
			pos_sum += *config.encoder_buffer;
			peripheral::Misc::ms_delay(1);
		}
		config.zero_index = pos_sum / calibrationRound;
	}
	float AbsoluteEncoderPositionSpeedSensor::get_position_nowrap() const
	{
		return round_m_ * _2PI + last_pos_m_raw_;

	};
} // wibot::motor
