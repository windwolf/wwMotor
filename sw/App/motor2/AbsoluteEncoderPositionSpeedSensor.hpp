//
// Created by zhouj on 2022/11/17.
//

#ifndef WWMOTOR_APP_MOTOR2_ABSOLUTEENCODERPOSITIONSPEEDSENSOR_HPP_
#define WWMOTOR_APP_MOTOR2_ABSOLUTEENCODERPOSITIONSPEEDSENSOR_HPP_

#include "PositionSpeedSensor.hpp"
#include "LinearValueMapper.hpp"
#include "filter/lp.hpp"

namespace wibot::motor
{
	using namespace wibot::accessor;
	using namespace wibot::control;
	struct AbsoluteEncoderPositionSpeedSensorConfig
	{
		uint32_t* encoder_buffer;
		uint16_t resolution; // 12bit = 4096
		uint16_t zero_index; //
		uint8_t pole_pairs;
		int8_t direction = 1;
		float mech_pos_cutoff_freq;
		float mech_speed_cutoff_freq;
		float sample_time;
	};

	class AbsoluteEncoderPositionSpeedSensor :
		public PositionSpeedSensor, public Configurable<AbsoluteEncoderPositionSpeedSensorConfig>
	{
	 public:
		void config_apply(AbsoluteEncoderPositionSpeedSensorConfig& config);

		void position_speed_get(Motor& motor, Vector2f& pos_spd_e, Vector2f& pos_spd_m) override;

		void zero_search(Motor& motor);

	 private:
		int32_t _last_pos_raw;
		uint32_t _half_resolution;
		LinearValueMapper _mapper;
		FirstOrderLowPassFilter _filter_pos_m;
		FirstOrderLowPassFilter _filter_pos_e;
		FirstOrderLowPassFilter _filter_spd_m;
		FirstOrderLowPassFilter _filter_spd_e;
		float _2pi_res_ts;
		float _2pipp_res_ts;
	};

} // wibot::motor

#endif //WWMOTOR_APP_MOTOR2_ABSOLUTEENCODERPOSITIONSPEEDSENSOR_HPP_
