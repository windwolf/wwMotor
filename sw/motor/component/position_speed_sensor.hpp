#ifndef __wwMotor_POSITION_SPEED_SENSOR_HPP__
#define __wwMotor_POSITION_SPEED_SENSOR_HPP__

#include "motor/base.hpp"
#include "buffer.hpp"
#include "framework/framework.hpp"

namespace wwMotor
{
	using namespace ww;
	using namespace wwControl;

#define ENCODER_BUFFER_SIZE 16

	class PositionSpeedFetcher
	{
	 public:
		virtual void position_speed_get(Vector2f& pos_spd_e, Vector2f& pos_spd_m) = 0;
	};

	struct EncoderPositionSpeedSensorConfig
	{
		uint16_t* encoder_data;
		float full_scalar;
		uint8_t pole_pairs;
		float sample_time;
	};

	class EncoderPositionSpeedSensor :
		public Configurable<EncoderPositionSpeedSensorConfig>,
		public PositionSpeedFetcher
	{
	 public:

		void init();
		void calibrate_begin();
		void calibrate();
		void calibrate_end();

		void _config_apply();

		/**
		 * @brief
		 *
		 * @return
		 */
		void position_speed_get(Vector2f& pos_spd_e, Vector2f& pos_spd_m) override;

	 private:
		uint16_t _buf_pos[ENCODER_BUFFER_SIZE];
		// float _buf_spd[ENCODER_BUFFER_SIZE];
		uint8_t _buf_idx;

		uint16_t _zero_position;
		uint16_t _var;

		float _speed_factor;  // _2PI / (float)_config.full_scalar / _config.sample_time /
		// (float)ENCODER_BUFFER_SIZE;
		float _scale_factor;  // _2PI / (float)_config.full_scalar
		uint16_t _half_scale; // _config.full_scalar / 2
	};

} // namespace wwMotor

#endif // __wwMotor_POSITION_SPEED_SENSOR_HPP__
