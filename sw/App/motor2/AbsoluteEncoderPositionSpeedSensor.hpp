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

	enum class EncoderDirection
	{
		Forward = 1,
		Reverse = -1
	};

	struct AbsoluteEncoderPositionSpeedSensorConfig
	{
		uint32_t* encoder_buffer;
		uint16_t resolution; // 12bit = 4096
		uint16_t zero_index; //
		uint8_t pole_pairs;
		EncoderDirection direction = EncoderDirection::Forward;
		float mech_pos_cutoff_freq;
		float mech_speed_cutoff_freq;
		float sample_time;
	};

	class AbsoluteEncoderPositionSpeedSensor :
		public PositionSpeedSensor, public Configurable<AbsoluteEncoderPositionSpeedSensorConfig>
	{
	 public:
		void config_apply(AbsoluteEncoderPositionSpeedSensorConfig& config);

		/**
		 * @brief get position and speed in electrical and mechanical domain.
		 * speed is in rad/s, position is in rad.
		 * @param motor
		 * @param pos_spd_e
		 * @param pos_spd_m
		 */
		void position_speed_get(Motor& motor, Vector2f& pos_spd_e, Vector2f& pos_spd_m) override;

		float get_position_nowrap() const;

		void zero_search(Motor& motor);

	 private:
		float last_pos_m_raw_;
		float last_pos_m_raw_without_lp_;
		int32_t round_m_;
		LinearValueMapper mapper_;
		FirstOrderLowPassFilter filter_pos_m_;
		FirstOrderLowPassFilter filter_pos_e_;
		FirstOrderLowPassFilter filter_spd_m_;
		FirstOrderLowPassFilter filter_spd_e_;

		float _2PI_RES;
		float _1_TS;
		float _PP_TS;
	};

} // wibot::motor

#endif //WWMOTOR_APP_MOTOR2_ABSOLUTEENCODERPOSITIONSPEEDSENSOR_HPP_
