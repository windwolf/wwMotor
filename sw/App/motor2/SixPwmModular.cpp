//
// Created by zhouj on 2022/11/21.
//

#include "SixPwmModular.hpp"

namespace wibot::motor
{
	void SixPwmModular::module(Motor& motor)
	{
		switch (motor.reference.section)
		{
		case 1: // CA
			motor.reference.sw_channel = 0b1101;
			motor.reference.d_abc.v1 = 0;
			motor.reference.d_abc.v2 = 0;
			motor.reference.d_abc.v3 = motor.reference.d_pwm;
			break;
		case 2: // CB
			motor.reference.sw_channel = 0b1110;
			motor.reference.d_abc.v1 = 0;
			motor.reference.d_abc.v2 = 0;
			motor.reference.d_abc.v3 = motor.reference.d_pwm;
			break;
		case 3: // AB
			motor.reference.sw_channel = 0b1011;
			motor.reference.d_abc.v1 = motor.reference.d_pwm;
			motor.reference.d_abc.v2 = 0;
			motor.reference.d_abc.v3 = 0;
			break;
		case 4: // AC
			motor.reference.sw_channel = 0b1101;
			motor.reference.d_abc.v1 = motor.reference.d_pwm;
			motor.reference.d_abc.v2 = 0;
			motor.reference.d_abc.v3 = 0;
			break;
		case 5: // BC
			motor.reference.sw_channel = 0b1110;
			motor.reference.d_abc.v1 = 0;
			motor.reference.d_abc.v2 = motor.reference.d_pwm;
			motor.reference.d_abc.v3 = 0;
			break;
		case 6: // BA
			motor.reference.sw_channel = 0b1011;
			motor.reference.d_abc.v1 = 0;
			motor.reference.d_abc.v2 = motor.reference.d_pwm;
			motor.reference.d_abc.v3 = 0;
			break;
		default:
			break;

		}
		motor.reference.d_sample = motor.reference.d_pwm / 2;
	}

} // wibot::motor
