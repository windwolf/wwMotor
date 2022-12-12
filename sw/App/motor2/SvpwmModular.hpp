//
// Created by zhouj on 2022/11/18.
//

#ifndef WWMOTOR_APP_MOTOR2_SVPWMMODULAR_HPP_
#define WWMOTOR_APP_MOTOR2_SVPWMMODULAR_HPP_

#include "Modular.hpp"

namespace wibot::motor
{
	struct SvpwmModularConfig
	{
		/**
		 * @brief 最大调试率.
		 * FOC算法下, < 1/sqrt(3)是线性调制区. 超过就算过调制.
		 */
		float max_module_rate;
		float max_d_module_rate;
		MotorParameter* motor_parameter;
	};

	class SvpwmModular : public Modular,
						 public Configurable<SvpwmModularConfig>
	{
	 public:
		void apply_config() override;
		void module(wibot::motor::Motor& motor,
			uint8_t& section,
			Vector3f& d_abc,
			Vector3f& u_abc,
			uint8_t& channels,
			float& d_sample);
	 private:
		void dq_limit(wibot::motor::Motor& motor);
		void ab_limit(wibot::motor::Motor& motor);

		float _max_sq;
		float _max_d_sq;
	};

} // wibot::motor

#endif //WWMOTOR_APP_MOTOR2_SVPWMMODULAR_HPP_
