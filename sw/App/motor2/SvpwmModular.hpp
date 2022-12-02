//
// Created by zhouj on 2022/11/18.
//

#ifndef WWMOTOR_APP_MOTOR2_SVPWMINVERTERDRIVER_HPP_
#define WWMOTOR_APP_MOTOR2_SVPWMINVERTERDRIVER_HPP_

#include "Modular.hpp"

namespace wwMotor2
{
	struct SvpwmInverterDriverConfig
	{
		float maxModuleRate;
		float maxDModuleRate;
		MotorParameter* motorParameter;
	};
	class SvpwmInverterDriver : public Modular,
								public Configurable<SvpwmInverterDriverConfig>
	{
	 public:
		void update_by_dq(wwMotor2::Motor& motor) override;
		void update_by_ab(wwMotor2::Motor& motor) override;

	 private:
		void circle_limit(wwMotor2::Motor& motor);
	};

} // wwMotor2

#endif //WWMOTOR_APP_MOTOR2_SVPWMINVERTERDRIVER_HPP_
