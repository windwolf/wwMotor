#include "position_speed_sensor.hpp"
#include "os.hpp"
namespace wwMotor
{

#define POS_BUF_GET() _config.encoder_buffer.data[_config.position_idx]
	void EncoderPositionSpeedSensor::init()
	{
		uint16_t v = *_config.encoder_data - _zero_position;
		for (size_t i = 0; i < ENCODER_BUFFER_SIZE; i++)
		{
			_buf_pos[i] = v;
		}
		_buf_idx = 0;
	};
	void EncoderPositionSpeedSensor::calibrate_begin()
	{
		for (size_t i = 0; i < ENCODER_BUFFER_SIZE; i++)
		{
			_buf_pos[i] = 0;
		}
		_buf_idx = 0;
	};

	void EncoderPositionSpeedSensor::calibrate()
	{
		_buf_pos[_buf_idx] = *_config.encoder_data;
		_buf_idx = (_buf_idx + 1) % ENCODER_BUFFER_SIZE;
	};

	void EncoderPositionSpeedSensor::calibrate_end()
	{
		uint32_t sum = 0;
		uint32_t var = 0;
		for (size_t i = 0; i < ENCODER_BUFFER_SIZE; i++)
		{
			sum += _buf_pos[i];
		}
		_zero_position = sum / ENCODER_BUFFER_SIZE;
		for (size_t i = 0; i < ENCODER_BUFFER_SIZE; i++)
		{
			var += (_buf_pos[i] - _zero_position) * (_buf_pos[i] - _zero_position);
		}
		_var = var / ENCODER_BUFFER_SIZE;

		uint16_t v = *_config.encoder_data - _zero_position;
		for (size_t i = 0; i < ENCODER_BUFFER_SIZE; i++)
		{
			_buf_pos[i] = v;
		}
		_buf_idx = 0;
	};

	void EncoderPositionSpeedSensor::_config_apply()
	{
		_scale_factor = _2PI / (float)_config.full_scalar;
		_speed_factor =
			_2PI / (float)_config.full_scalar / _config.sample_time / (float)ENCODER_BUFFER_SIZE;
		_half_scale = _config.full_scalar / 2;
	};

	void EncoderPositionSpeedSensor::position_speed_get(Vector2f& pos_spd_e, Vector2f& pos_spd_m)
	{
		uint16_t raw_pos = *_config.encoder_data - _zero_position;
		int32_t raw_pos_diff = (int32_t)raw_pos - (int32_t)_buf_pos[_buf_idx];

		// a->b: 1-2 dir=1; 1-9 dir=-1; 8-7 dir=-1; 8-1 dir=1;
		if (raw_pos_diff > 0)
		{
			if (raw_pos_diff < _half_scale)
			{
			}
			else
			{
				raw_pos_diff = raw_pos_diff - _config.full_scalar;
			}
		}
		else if (raw_pos_diff < 0)
		{
			if (raw_pos_diff > -_half_scale)
			{
			}
			else
			{
				raw_pos_diff = raw_pos_diff + _config.full_scalar;
			}
		}
		else
		{
		}

		_buf_pos[_buf_idx] = raw_pos;
		_buf_idx++;

		pos_spd_m.v1 = Math::circle_normalize((float)raw_pos * _scale_factor);
		pos_spd_m.v2 = Math::circle_normalize(pos_spd_m.v1 * _config.pole_pairs);
		pos_spd_e.v1 = (float)raw_pos_diff * _speed_factor;
		pos_spd_e.v2 = pos_spd_e.v1 * _config.pole_pairs;

	};

} // namespace wwMotor
