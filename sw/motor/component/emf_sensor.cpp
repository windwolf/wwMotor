#include "emf_sensor.hpp"

namespace wwMotor
{

	void SixStepEmfSensor::_config_apply()
	{
		//_value_per_unit = _config.refVoltage / _config.maxScaleValue;
	};

/**
 * @brief
 *
 * AB CB CA BA BC AC
 *
 * @param section
 * @return Vector3f
 */
	Vector3f SixStepEmfSensor::abc_get(uint8_t section)
	{
	};

	bool SixStepEmfSensor::zero_cross_detect(uint8_t section)
	{
		bool result = false;
		_tick++;

		uint16_t ph_a = *_config.a_phase_value;
		uint16_t ph_b = *_config.b_phase_value;
		uint16_t ph_c = *_config.c_phase_value;
		uint16_t _3n = (ph_a + ph_b + ph_c);
		int32_t _3emf;
		switch (section)
		{
		case 1: // AB
		case 4: // BA
			_3emf = ph_c * 3 - _3n;
			break;
		case 2: // CB
		case 5: // BC
			_3emf = ph_a * 3 - _3n;
			break;
		case 3: // CA
		case 6: // AC
			_3emf = ph_b * 3 - _3n;
			break;
		default:
			break;
		}

		if (_last_section != section)
		{
			_3emf = 0;
			_last_section_switch_tick = _tick;
		}

		if ((_tick - _last_section_switch_tick) < _config.skip_sample_count)
		{
			result = false;
		}
		else
		{
			if ((_3emf > 0 && _last_3emf < 0) || (_3emf < 0 && _last_3emf > 0))
			{
				_last_zero_cross_tick = _tick;
				result = true;
				_zero_cross_flag = true;
				_last_3emf = _3emf;
			}
		}

		_last_section = section;

		return result;
	};

} // namespace wwMotor
