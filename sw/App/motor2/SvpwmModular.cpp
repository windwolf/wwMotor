//
// Created by zhouj on 2022/11/18.
//

#include "SvpwmModular.hpp"
#include "math_shared.hpp"

namespace wibot::motor
{
	void SvpwmModular::dq_limit(wibot::motor::Motor& motor)
	{
		Vector2f new_dq = motor.reference.u_dq;
		float ubus_sq = motor.state.u_bus * motor.state.u_bus;

		float sqD = motor.reference.u_dq.v1 * motor.reference.u_dq.v1;
		float sqQ = motor.reference.u_dq.v2 * motor.reference.u_dq.v2;
		float sqDQ = sqD + sqQ;
		float sqModule = _max_sq * ubus_sq;
		float sqDModule = _max_d_sq * ubus_sq;
		if (sqDQ > sqModule)
		{
			if (sqD > sqDModule)
			{
				new_dq.v1 = config.max_d_module_rate * motor.state.u_bus;
				if (motor.reference.u_dq.v1 < 0)
				{
					new_dq.v1 = -new_dq.v1;
				}

				new_dq.v2 = Math::sqrt(sqModule - sqDModule);
				if (motor.reference.u_dq.v2 < 0)
				{
					new_dq.v2 = -new_dq.v2;
				}
			}
			else
			{
				// u_dq.v1 = u_dq.v1;

				new_dq.v2 = Math::sqrt(sqModule - sqD);
				if (motor.reference.u_dq.v2 < 0)
				{
					new_dq.v2 = -new_dq.v2;
				}
			}
		}
		else
		{
			// u_dq = u_dq;
		}
		motor.reference.u_dq = new_dq;
	};

	void SvpwmModular::ab_limit(wibot::motor::Motor& motor)
	{
		Vector2f u_ab = motor.reference.u_ab;
		float limit = config.max_module_rate * motor.state.u_bus;
		float ab = Math::sqrt(u_ab.v1 * u_ab.v1 + u_ab.v2 * u_ab.v2);

		if (ab > limit)
		{
			u_ab.v1 = u_ab.v1 * limit / ab;
			u_ab.v2 = u_ab.v2 * limit / ab;
		}
		motor.reference.u_ab = u_ab;
	};

	/**
	 * @brief
	 * sec 1: a > b > c;
	 * sec 2: b > a > c;
	 * sec 3: b > c > a;
	 * sec 4: c > b > a;
	 * sec 5: c > a > b;
	 * sec 6: a > c > b;
	 * @param motor
	 * @param section
	 * @param d_abc
	 * @param u_abc
	 * @param channels
	 * @param d_sample
	 */
	void SvpwmModular::module(Motor& motor,
		uint8_t& section,
		Vector3f& d_abc,
		Vector3f& u_abc,
		uint8_t& channels,
		float& d_sample)
	{
		dq_limit(motor);
		float uOut;
		float theta;
		float ts;
		Vector2f u_dq = motor.reference.u_dq;
		//Vector2f u_ab;
		//FocMath::dq2ab(motor.reference.u_dq, motor.state.pos_spd_e.v1, u_ab);

		uint8_t sec;
		if (motor.reference.u_dq.v1 == 0)
		{
			uOut = u_dq.v2 / motor.state.u_bus;
			theta = Math::circle_normalize(motor.state.pos_spd_e.v1 + _PI_2);
		}
		else
		{
			uOut = Math::sqrt(u_dq.v1 * u_dq.v1 + u_dq.v2 * u_dq.v2) / motor.state.u_bus;
			theta = Math::circle_normalize(Math::atan2(u_dq.v2, u_dq.v1) + motor.state.pos_spd_e.v1);
		}
		sec = (uint8_t)(int)Math::floor(theta / _PI_3) + 1;

		float t1 = _SQRT3 * Math::sin((float)(sec) * _PI_3 - theta) * uOut;
		float t2 = _SQRT3 * Math::sin(theta - ((float)(sec) - 1.0f) * _PI_3) * uOut;
		float t0 = 1 - t1 - t2;
		if (t0 < 0)
		{
			t0 = .0f;
		}
		Vector3f tAbc;
		switch (sec)
		{
		case 1:
			tAbc.v1 = t1 + t2 + t0 / 2;
			tAbc.v2 = t2 + t0 / 2;
			tAbc.v3 = t0 / 2;
			ts = tAbc.v1 + t0 / 4;
			break;
		case 2:
			tAbc.v1 = t1 + t0 / 2;
			tAbc.v2 = t1 + t2 + t0 / 2;
			tAbc.v3 = t0 / 2;
			ts = tAbc.v2 + t0 / 4;
			break;
		case 3:
			tAbc.v1 = t0 / 2;
			tAbc.v2 = t1 + t2 + t0 / 2;
			tAbc.v3 = t2 + t0 / 2;
			ts = tAbc.v2 + t0 / 4;
			break;
		case 4:
			tAbc.v1 = t0 / 2;
			tAbc.v2 = t1 + t0 / 2;
			tAbc.v3 = t1 + t2 + t0 / 2;
			ts = tAbc.v3 + t0 / 4;
			break;
		case 5:
			tAbc.v1 = t2 + t0 / 2;
			tAbc.v2 = t0 / 2;
			tAbc.v3 = t1 + t2 + t0 / 2;
			ts = tAbc.v3 + t0 / 4;
			break;
		case 6:
			tAbc.v1 = t1 + t2 + t0 / 2;
			tAbc.v2 = t0 / 2;
			tAbc.v3 = t1 + t0 / 2;
			ts = tAbc.v1 + t0 / 4;
			break;
		default:
			tAbc.v1 = 0;
			tAbc.v2 = 0;
			tAbc.v3 = 0;
			ts = 1.0f;
		}
		section = sec;
		d_abc = tAbc;
		u_abc = tAbc * motor.state.u_bus;
		channels = 0x01 | 0x02 | 0x04 | 0x08;

		if (ts > 1.0f)
		{
			d_sample = 1.0f;
		}
		else
		{
			d_sample = ts;
		}

	}

	void SvpwmModular::config_apply(SvpwmModularConfig& config)
	{
		this->config = config;
		_max_sq = config.max_module_rate * config.max_module_rate;
		_max_d_sq = config.max_d_module_rate * config.max_d_module_rate;
	}

} // wibot::motor
