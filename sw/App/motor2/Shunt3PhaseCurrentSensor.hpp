//
// Created by zhouj on 2022/11/17.
//

#ifndef WWMOTOR_APP_MOTOR2_SHUNT3PHASECURRENTSENSOR_HPP_
#define WWMOTOR_APP_MOTOR2_SHUNT3PHASECURRENTSENSOR_HPP_

#include "PhaseCurrentSensor.hpp"
#include "LinearValueMapper.hpp"

namespace wwMotor2
{
	using namespace ww::accessor;
	struct Shunt3PhaseCurrentSensorConfig
	{
		uint32_t* i_a_buffer;
		uint32_t* i_b_buffer;
		uint32_t* i_c_buffer;

		uint32_t i_a_offset;
		uint32_t i_b_offset;
		uint32_t i_c_offset;
		/**
		 * 每刻度电流值.
		 * 参考电压 / 最大量程 / 放大器倍数 / 采样电阻
		 * 例如: 3.3 / 4096 / 1 / 0.1 = 0.0081
		 * 放大倍数的选择, 尽量使可用的最大电流范围满量程.
		 * 放大倍数 = 参考电压 / 采样电阻 / 最大电流
		 */
		float i_value_per_unit;
	};
	class Shunt3PhaseCurrentSensor : public PhaseCurrentSensor,
									 Configurable<Shunt3PhaseCurrentSensorConfig>
	{
	 public:

		void config_apply(Shunt3PhaseCurrentSensorConfig& config) override;

		void i_abc_get(wwMotor2::Motor& motor, Vector3f& i_abc) override;

		void zero_calibrate(wwMotor2::Motor& motor) override;

	 private:
		LinearValueMapper _a_mapper;
		LinearValueMapper _b_mapper;
		LinearValueMapper _c_mapper;
	};

} // wwMotor2

#endif //WWMOTOR_APP_MOTOR2_SHUNT3PHASECURRENTSENSOR_HPP_
