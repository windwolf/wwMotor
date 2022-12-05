//
// Created by zhouj on 2022/11/21.
//

#include "SixPwmModular.hpp"

namespace wibot::motor
{
	void SixPwmModular::module(Motor& motor,
		uint8_t& section,
		Vector3f& d_abc,
		Vector3f& u_abc,
		uint8_t& channels,
		float& d_sample)
	{
		switch (motor.reference.section)
		{
		case 1: // CA
			channels = 0b1101;
			d_abc.v1 = 0;
			d_abc.v2 = 0;
			d_abc.v3 = motor.reference.d_pwm;
			break;
		case 2: // CB
			channels = 0b1110;
			d_abc.v1 = 0;
			d_abc.v2 = 0;
			d_abc.v3 = motor.reference.d_pwm;
			break;
		case 3: // AB
			channels = 0b1011;
			d_abc.v1 = motor.reference.d_pwm;
			d_abc.v2 = 0;
			d_abc.v3 = 0;
			break;
		case 4: // AC
			channels = 0b1101;
			d_abc.v1 = motor.reference.d_pwm;
			d_abc.v2 = 0;
			d_abc.v3 = 0;
			break;
		case 5: // BC
			channels = 0b1110;
			d_abc.v1 = 0;
			d_abc.v2 = motor.reference.d_pwm;
			d_abc.v3 = 0;
			break;
		case 6: // BA
			channels = 0b1011;
			d_abc.v1 = 0;
			d_abc.v2 = motor.reference.d_pwm;
			d_abc.v3 = 0;
			break;
		default:
			break;
		}
		d_sample = motor.reference.d_pwm / 2;
	}

} // wibot::motor
