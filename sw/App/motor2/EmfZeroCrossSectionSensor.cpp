//
// Created by zhouj on 2022/11/17.
//

#include "EmfZeroCrossSectionSensor.hpp"

namespace wibot::motor
{

	void EmfZeroCrossSectionSensor::section_get(Motor& motor, uint8_t& section)
	{
		_tick++;

		if (_blank_count > 0)
		{
			_blank_count--;
		}
		else
		{
			emf_get(motor);
		}

		bool zero_cross = zero_cross_detect(motor);
		if (zero_cross)
		{
			switch_delay_set(motor);
		}

		_last_section = section;

	}

	void EmfZeroCrossSectionSensor::emf_get(Motor& motor)
	{
		if (_last_section != motor.state.section)
		{
			_blank_count = config.blank_count;
		}
		auto& u = motor.state.u_abc;
		float _3n = (u.v1 + u.v2 + u.v3) + _zero_offset;
		_3emf_a = u.v1 * 3 - _3n;
		_3emf_b = u.v2 * 3 - _3n;
		_3emf_c = u.v3 * 3 - _3n;
	}

	void EmfZeroCrossSectionSensor::switch_delay_set(Motor& motor)
	{
		// 此时相位已经延迟超过30°, motor.state.section 和 过零点的区间已经不匹配,
		// 需要查找多附近扇区的过零点, 以便能在下一步纠偏

		_zero_cross_span = _tick - _last_zero_cross_tick;
		_last_zero_cross_tick = _tick;
		auto _30_section_switch_delay_count = _zero_cross_span / 2;
		if (config.switch_delay_count <= _30_section_switch_delay_count)
		{
			// 如果相位延迟在30°内, 延迟30°-delay_count;
			_section_switch_delay_count = _30_section_switch_delay_count - config.switch_delay_count;
			_is_slow = true;
		}
		else
		{
			auto _90_section_switch_delay_count = _30_section_switch_delay_count * 3;
			if (config.switch_delay_count <= _90_section_switch_delay_count)
			{
				// 如果相位延迟在90°内, 延迟90°-delay_count;
				_section_switch_delay_count = _90_section_switch_delay_count - config.switch_delay_count;
				_is_slow = false;
			}
			else
			{
				// 相位延迟超过90°, 直接换向.
				_section_switch_delay_count = 0;
				_is_slow = false;
			}
		}

		_section_switch_event_pending = true;
	}

	void EmfZeroCrossSectionSensor::section_switch(Motor& motor, uint8_t& section)
	{
		uint8_t sec = motor.state.section;
		if (_section_switch_event_pending)
		{
			if (_section_switch_delay_count == 0)
			{
				_section_switch_event_pending = false;
				if (motor.reference.speed > 0)
				{
					sec = sec + 1;
				}
				else if (motor.reference.speed < 0)
				{
					sec = sec - 1;
				}
				else
				{
					sec = 0;
				}
				if (sec > 6)
				{
					sec = 1;
				}
				if (sec < 1)
				{
					sec = 6;
				}
				section = sec;
			}
			else
			{
				_section_switch_delay_count--;
			}
		}
	}
	bool EmfZeroCrossSectionSensor::zero_cross_detect(Motor& motor)
	{
		bool zero_cross = false;
		if (_is_slow)
		{

			switch (motor.state.section)
			{
			case 1: // CA
				zero_cross = (_last_3emf_b >= 0 && _3emf_b <= 0);
				break;
			case 2: // CB
				zero_cross = (_last_3emf_a <= 0 && _3emf_a >= 0);
				break;
			case 3: // AB
				zero_cross = (_last_3emf_c >= 0 && _3emf_c <= 0);
				break;
			case 4: // AC
				zero_cross = (_last_3emf_b <= 0 && _3emf_b >= 0);
				break;
			case 5: // BC
				zero_cross = (_last_3emf_a >= 0 && _3emf_a <= 0);
				break;
			case 6: // BA
				zero_cross = (_last_3emf_c <= 0 && _3emf_c >= 0);
				break;
			default:
				break;
			}
		}
		else
		{
			if (motor.reference.speed > 0)
			{
				switch (motor.state.section)
				{
				case 1: //CA 检查 6
					zero_cross = (_last_3emf_c <= 0 && _3emf_c >= 0);
					break;
				case 2: // CB
					zero_cross = (_last_3emf_b >= 0 && _3emf_b <= 0);
					break;
				case 3: // AB
					zero_cross = (_last_3emf_a <= 0 && _3emf_a >= 0);
					break;
				case 4: // AC
					zero_cross = (_last_3emf_c >= 0 && _3emf_c <= 0);
					break;
				case 5: // BC
					zero_cross = (_last_3emf_b <= 0 && _3emf_b >= 0);
					break;
				case 6: // BA
					zero_cross = (_last_3emf_a >= 0 && _3emf_a <= 0);
					break;
				default:
					break;
				}
			}
			else if (motor.reference.speed < 0)
			{
				switch (motor.state.section)
				{
				case 1: // CA
					zero_cross = (_last_3emf_a <= 0 && _3emf_a >= 0);
					break;
				case 2: // CB
					zero_cross = (_last_3emf_c >= 0 && _3emf_c <= 0);
					break;
				case 3: // AB
					zero_cross = (_last_3emf_b <= 0 && _3emf_b >= 0);
					break;
				case 4: // AC
					zero_cross = (_last_3emf_a >= 0 && _3emf_a <= 0);
					break;
				case 5: // BC
					zero_cross = (_last_3emf_c <= 0 && _3emf_c >= 0);
					break;
				case 6: // BA
					zero_cross = (_last_3emf_b >= 0 && _3emf_b <= 0);
					break;
				default:
					break;
				}
			}
		}
		return zero_cross;
	}

} // wibot::motor
