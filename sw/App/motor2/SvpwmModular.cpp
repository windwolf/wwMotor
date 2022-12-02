//
// Created by zhouj on 2022/11/18.
//

#include "SvpwmModular.hpp"
#include "math_shared.hpp"

namespace wwMotor2
{
	void SvpwmModular::update_by_dq(Motor& motor)
	{
		circle_limit(motor);
		float uOut;
		float theta;
		Vector2f u_dq = motor.reference.u_dq;
		//Vector2f u_ab;
		//FocMath::dq2ab(motor.reference.u_dq, motor.state.pos_spd_e.v1, u_ab);

		uint8_t section;
		if (motor.reference.u_dq.v1 == 0)
		{
			uOut = u_dq.v2 / motor.state.u_bus;
			theta = motor.state.pos_spd_e.v1 + _PI_2;
		}
		else
		{
			uOut = Math::sqrt(u_dq.v1 * u_dq.v1 + u_dq.v2 * u_dq.v2) / motor.state.u_bus;
			theta = Math::circle_normalize(Math::atan2(u_dq.v2, u_dq.v1) + motor.state.pos_spd_e.v1);
		}
		section = (uint8_t)(int)Math::floor(theta / _PI_3) + 1;

		float t1 = _SQRT3 * Math::sin((float)(section) * _PI_3 - theta) * uOut;
		float t2 = _SQRT3 * Math::sin(theta - ((float)(section) - 1.0f) * _PI_3) * uOut;
		float t0 = 1 - t1 - t2;

		Vector3f tAbc;
		switch (section)
		{
		case 1:
			tAbc.v1 = t1 + t2 + t0 / 2;
			tAbc.v2 = t2 + t0 / 2;
			tAbc.v3 = t0 / 2;
			break;
		case 2:
			tAbc.v1 = t1 + t0 / 2;
			tAbc.v2 = t1 + t2 + t0 / 2;
			tAbc.v3 = t0 / 2;
			break;
		case 3:
			tAbc.v1 = t0 / 2;
			tAbc.v2 = t1 + t2 + t0 / 2;
			tAbc.v3 = t2 + t0 / 2;
			break;
		case 4:
			tAbc.v1 = t0 / 2;
			tAbc.v2 = t1 + t0 / 2;
			tAbc.v3 = t1 + t2 + t0 / 2;
			break;
		case 5:
			tAbc.v1 = t2 + t0 / 2;
			tAbc.v2 = t0 / 2;
			tAbc.v3 = t1 + t2 + t0 / 2;
			break;
		case 6:
			tAbc.v1 = t1 + t2 + t0 / 2;
			tAbc.v2 = t0 / 2;
			tAbc.v3 = t1 + t0 / 2;
			break;
		default:
			tAbc.v1 = 0;
			tAbc.v2 = 0;
			tAbc.v3 = 0;
		}
		motor.reference.section = section;
		motor.reference.d_abc = tAbc;
		motor.reference.u_abc = tAbc * motor.state.u_bus;

		// TODO: feed to currentSensor to decide the time window for sampling.
		// motor.reference.d_sample?

	}
} // wwMotor2
