//
// Created by zhouj on 2022/12/5.
//

#include "HfInjectionPositionSpeedSensorAndModular.hpp"

namespace wibot::motor
{
	void HfInjectionPositionSpeedSensorAndModular::module(Motor& motor,
		uint8_t& section,
		Vector3f& d_abc,
		Vector3f& u_abc,
		uint8_t& channels,
		float& d_sample)
	{
		_svpwm_modular.module(motor, section, d_abc, u_abc, channels, d_sample);
	}
} // motor
