#include "driver.hpp"

namespace wwMotor
{
	Vector2f DriverSVPWM::circle_limit(Vector2f u_dq, float ubus)
	{
		Vector2f newdq;
		float ubus_sq = ubus * ubus;

		float sqD = u_dq.v1 * u_dq.v1;
		float sqQ = u_dq.v2 * u_dq.v2;
		float sqDQ = sqD + sqQ;
		float sqModule = sqModuleLimit * ubus_sq;
		float sqDModule = sqDModuleLimit * ubus_sq;
		if (sqDQ > sqModule)
		{
			if (sqD > sqDModule)
			{
				newdq.v1 = dModuleLimit * ubus;
				if (u_dq.v1 < 0)
				{
					newdq.v1 = -newdq.v1;
				}

				newdq.v2 = Math::sqrt(sqDQ - sqDModule);
				if (u_dq.v2 < 0)
				{
					newdq.v2 = -newdq.v2;
				}
			}
			else
			{
				newdq.v1 = u_dq.v1;

				newdq.v2 = Math::sqrt(sqDQ - sqD);
				if (u_dq.v2 < 0)
				{
					newdq.v2 = -newdq.v2;
				}
			}
		}
		else
		{
			newdq = u_dq;
		}
		return newdq;
	};

	void DriverSVPWM::_config_apply()
	{
		sqDModuleLimit = _config.maxDModuleRate * _config.maxDModuleRate / 3;
		sqModuleLimit = _config.maxModuleRate * _config.maxModuleRate / 3;
		// TODO: update only vbus changed.
		dModuleLimit = _1_SQRT3 * _config.maxDModuleRate;
	};

	void DriverSVPWM::phase_voltage_set(Vector2f u_ab, float ubus)
	{
		float uOut;
		float theta;
		uint8_t section;
		if (u_ab.v1 != 0)
		{
			uOut = Math::sqrt(u_ab.v1 * u_ab.v1 + u_ab.v2 * u_ab.v2);
			uOut /= ubus;

			theta = Math::circle_normalize(Math::atan2(u_ab.v2, u_ab.v1));
		}
		else
		{
			uOut = u_ab.v2 / ubus;
			theta = _PI_2;
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
		this->section = section;
		this->u_abc = tAbc * ubus;
		this->duty = tAbc;

		// TODO: feed to currentSensor to decide the time window for sampling.
		//  currentSense->pwmDutyA = tA;
		//  currentSense->pwmDutyB = tB;
		//  currentSense->pwmDutyC = tC;

		_executor.duty_set(this->duty);
	};

} // namespace wwMotor
