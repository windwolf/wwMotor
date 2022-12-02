//
// Created by zhouj on 2022/11/28.
//

#ifndef WWMOTOR_APP_MOTOR2_SAMPLEPHASEVOLTAGESENSOR_HPP_
#define WWMOTOR_APP_MOTOR2_SAMPLEPHASEVOLTAGESENSOR_HPP_

#include "base.hpp"
#include "PhaseVoltageSensor.hpp"
#include "LinearValueMapper.hpp"
#include "lp.hpp"
namespace wwMotor2
{
	using namespace ww::accessor;
	using namespace wwControl;
	struct SamplePhaseVoltageSensorConfig
	{
		uint32_t* u_a_buffer;
		uint32_t* u_b_buffer;
		uint32_t* u_c_buffer;

		float cutoff_freq; // 务必大于最大电频率.
		float sample_time;

		float u_value_per_unit;
	};
	class SamplePhaseVoltageSensor :
		public PhaseVoltageSensor,
		public Configurable<SamplePhaseVoltageSensorConfig>
	{
	 public:
		void config_apply(SamplePhaseVoltageSensorConfig& config) override;

		void u_abc_get(wwMotor2::Motor& motor, Vector3f& u_abc) override;

	 private:
		LinearValueMapper _a_mapper;
		LinearValueMapper _b_mapper;
		LinearValueMapper _c_mapper;
		FirstOrderLowPassFilter _a_filter;
		FirstOrderLowPassFilter _b_filter;
		FirstOrderLowPassFilter _c_filter;
	};

} // wwMotor2

#endif //WWMOTOR_APP_MOTOR2_SAMPLEPHASEVOLTAGESENSOR_HPP_
