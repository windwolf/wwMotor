//
// Created by zhouj on 2022/11/17.
//

#ifndef WWMOTOR_APP_MOTOR2_ABSOLUTEENCODERPOSITIONSPEEDSENSOR_HPP_
#define WWMOTOR_APP_MOTOR2_ABSOLUTEENCODERPOSITIONSPEEDSENSOR_HPP_

#include "PositionSpeedSensor.hpp"
#include "LinearValueMapper.hpp"
#include "filter/lp.hpp"

namespace wwMotor2
{
	using namespace ww::accessor;
	using namespace wwControl;
	struct AbsoluteEncoderPositionSpeedSensorConfig
	{
		uint32_t* encoder_buffer;
		uint16_t resolution; // 12bit = 4096
		uint16_t zero_index;
		uint8_t pole_pairs;
		float mech_speed_cutoff_freq;
		float sample_time;
	};

	class AbsoluteEncoderPositionSpeedSensor :
		public PositionSpeedSensor, public Configurable<AbsoluteEncoderPositionSpeedSensorConfig>
	{
	 public:
		void config_apply(AbsoluteEncoderPositionSpeedSensorConfig& config) override;

		void position_speed_get(Motor& motor, Vector2f& pos_spd_e, Vector2f& pos_spd_m) override;

		void zero_search(Motor& motor);

	 private:
		uint16_t _last_pos_raw;
		uint16_t _half_resolution;
		LinearValueMapper _mapper;
		FirstOrderLowPassFilter _filter_m;
		FirstOrderLowPassFilter _filter_e;
		float _2pi_res_ts;
		float _2pipp_res_ts;
	};

} // wwMotor2

#endif //WWMOTOR_APP_MOTOR2_ABSOLUTEENCODERPOSITIONSPEEDSENSOR_HPP_