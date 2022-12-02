//
// Created by zhouj on 2022/11/18.
//

#ifndef WWMOTOR_APP_MOTOR2_SVPWMMODULAR_HPP_
#define WWMOTOR_APP_MOTOR2_SVPWMMODULAR_HPP_

#include "Modular.hpp"

namespace wwMotor2
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
		void config_apply(SvpwmModularConfig& config) override;
		void module(wwMotor2::Motor& motor) override;
	 private:
		void dq_limit(wwMotor2::Motor& motor);
		void ab_limit(wwMotor2::Motor& motor);

		float _max_sq;
		float _max_d_sq;
	};

} // wwMotor2

#endif //WWMOTOR_APP_MOTOR2_SVPWMMODULAR_HPP_
