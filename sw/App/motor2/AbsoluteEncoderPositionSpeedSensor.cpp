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

#define CALIBRATION_SAMPLING_ROUND 50

	void AbsoluteEncoderPositionSpeedSensor::config_apply(AbsoluteEncoderPositionSpeedSensorConfig& config)
	{
		this->config = config;

		this->_2PI_RES = _2PI / (float)config.resolution;
		this->_1_TS = 1.0f / config.sample_time;
		this->_PP_TS = (float)config.pole_pairs * this->_1_TS;

		PiecewiseLinearValueMapperConfig<ENCODER_MAPPER_POINT_COUNT> pvmCfg;
		pvmCfg.value_per_unit = this->_2PI_RES;
		pvmCfg.in_wrap = config.resolution;
		pvmCfg.out_wrap = _2PI;
		_mapper.apply_config(pvmCfg);

		FirstOrderLowPassFilterConfig lpCfg;
		lpCfg.sample_time = config.sample_time;
		lpCfg.cutoff_freq = config.mech_pos_cutoff_freq * _2PI;
		lpCfg.enable_wrap = true;
		lpCfg.wrap_value = _PI;
		filter_pos_m_.config_apply(lpCfg);

		lpCfg.wrap_value = _PI * config.pole_pairs;
		lpCfg.cutoff_freq = config.mech_pos_cutoff_freq * _2PI * (float)config.pole_pairs;
		filter_pos_e_.config_apply(lpCfg);

		lpCfg.enable_wrap = false;
		lpCfg.wrap_value = 0;
		lpCfg.cutoff_freq = config.mech_speed_cutoff_freq;
		filter_spd_m_.config_apply(lpCfg);

		lpCfg.cutoff_freq = config.mech_speed_cutoff_freq * (float)config.pole_pairs;
		filter_spd_e_.config_apply(lpCfg);

	}

	void AbsoluteEncoderPositionSpeedSensor::position_speed_get(Motor& motor, Vector2f& pos_spd_e, Vector2f& pos_spd_m)
	{
		//TODO: 电角度计算是否有误.
		auto pos_m_raw_i = _data_source->get_data();

		// process mech pos
		float pos_raw_m = _mapper.value_get(pos_m_raw_i); // in rad. 0 ~ 2pi
		float pos_raw_shift_m = pos_raw_m - _PI; // in rad. -pi ~ pi
		if (config.direction == EncoderDirection::Reverse)
		{
			pos_raw_shift_m = -pos_raw_shift_m;
		}

		float pos_m_raw_diff = pos_raw_shift_m - last_pos_m_raw_;
		last_pos_m_raw_ = pos_raw_shift_m;
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
		last_pos_m_ = pos_raw_shift_m + _PI;
		pos_raw_shift_m = filter_pos_m_.filter(pos_raw_shift_m);
		pos_spd_m.v1 = Math::circle_normalize(pos_raw_shift_m + _PI);
		pos_spd_m.v2 = filter_spd_m_.filter(pos_m_raw_diff * _1_TS);

		// process elec pos
		float pos_raw_e = pos_raw_m * (float)config.pole_pairs; // in rad. 0 ~ 2pi*PP
		float pos_raw_shift_e = pos_raw_e - _PI * config.pole_pairs; // in rad. -pi*PP ~ pi*PP
		if (config.direction == EncoderDirection::Reverse)
		{
			pos_raw_shift_e = -pos_raw_shift_e;
		}
		pos_raw_shift_e = filter_pos_e_.filter(pos_raw_shift_e);
		pos_spd_e.v1 = Math::circle_normalize(pos_raw_shift_e + _PI * config.pole_pairs);
		pos_spd_e.v2 = filter_spd_e_.filter(pos_m_raw_diff * _PP_TS);
	}

	float AbsoluteEncoderPositionSpeedSensor::get_position_nowrap() const
	{
		return round_m_ * _2PI + last_pos_m_raw_;
	}
	float AbsoluteEncoderPositionSpeedSensor::get_position_without_filter() const
	{
		return last_pos_m_;
	}
	void AbsoluteEncoderPositionSpeedSensor::calibrate(Motor& motor)
	{

		Vector2f o_u_dq = motor.reference.u_dq;
		Vector2f o_pos_spd_m = motor.state.pos_spd_m;
		Vector2f o_pos_spd_e = motor.state.pos_spd_e;

		motor.reference.u_dq.v1 = config.calbration_voltage;
		motor.reference.u_dq.v2 = 0.0f;
		motor.state.pos_spd_e.v1 = 0.0f;
		os::Utils::delay(100);

		// set zero offset
		rotate(motor, 0.0f, _2PI, 200, 1);
		os::Utils::delay(10);
		rotate(motor, 0.0f, -_2PI, 200, 1);
		os::Utils::delay(10);

		uint32_t zero_offset = 0;
		for (int i = 0; i < CALIBRATION_SAMPLING_ROUND; i++)
		{
			zero_offset += _data_source->get_data();
			os::Utils::delay(2);
		}
		zero_offset /= CALIBRATION_SAMPLING_ROUND;
		this->_mapper.set_zero_offset(zero_offset);

		this->_mapper.calibrate_begin();
		_mapper.calibrate(0, zero_offset, 0);

		float output_res = _2PI / ENCODER_MAPPER_POINT_COUNT;
		int i = 1;
		// rotate 360 degree forward
		for (; i < ENCODER_MAPPER_POINT_COUNT; ++i)
		{
			rotate(motor, output_res * (i - 1), output_res, 10, 1);
			sample_data(motor, i);
		}
		// i = PC - 1; rotate to 0 degree
		rotate(motor, output_res * i, output_res, 10, 1);

		// rotate 360 degree backward
		for (; i > 0; --i)
		{
			rotate(motor, output_res * (i + 1), -output_res, 10, 1);
			sample_data(motor, i);
		}
		_mapper.calibrate_end();

		motor.reference.u_dq = o_u_dq;
		motor.state.pos_spd_m = o_pos_spd_m;
		motor.state.pos_spd_e = o_pos_spd_e;

	}

	void AbsoluteEncoderPositionSpeedSensor::sample_data(const Motor& motor, int step)
	{
		for (int r = 0; r < CALIBRATION_SAMPLING_ROUND; r++)
		{
			auto raw = _data_source->get_data();
			_mapper.calibrate(step, raw, motor.state.pos_spd_m.v1);
			Utils::delay(1);
		}
	}

	void AbsoluteEncoderPositionSpeedSensor::rotate(Motor& motor,
		float start_pos,
		float travel,
		uint32_t step,
		uint32_t step_delay) const
	{
		for (int j = 0; j < step + 1; j++)
		{
			motor.state.pos_spd_m.v1 =
				Math::circle_normalize(start_pos + travel / step / j);
			motor.state.pos_spd_e.v1 = Math::circle_normalize(motor.state.pos_spd_m.v1 * config.pole_pairs);
			Utils::delay(step_delay);
		}
	}

} // wibot::motor
