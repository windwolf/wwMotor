//
// Created by zhouj on 2022/11/17.
//

#include "Shunt3PhaseCurrentSensor.hpp"
#include "os.hpp"

namespace wibot::motor
{
	using namespace wibot::os;
	/**
	 * sec 1: a > b > c;
	 * sec 2: b > a > c;
	 * sec 3: b > c > a;
	 * sec 4: c > b > a;
	 * sec 5: c > a > b;
	 * sec 6: a > c > b;
	 * @param motor
	 * @param i_abc
	 */
	void Shunt3PhaseCurrentSensor::i_abc_get(Motor& motor, Vector3f& i_abc)
	{
		i_abc.v1 = _a_mapper.value_get(*config.i_a_buffer);
		i_abc.v2 = _b_mapper.value_get(*config.i_b_buffer);
		i_abc.v3 = _c_mapper.value_get(*config.i_c_buffer);
		if (motor.reference.d_abc.v1 > config.skip_threshold
			|| motor.reference.d_abc.v2 < -config.skip_threshold
			|| motor.reference.d_abc.v3 < -config.skip_threshold)
		{
			switch (motor.reference.section)
			{
			case 1:
			case 6:
				i_abc.v1 = -i_abc.v2 - i_abc.v3;
				break;
			case 2:
			case 3:
				i_abc.v2 = -i_abc.v1 - i_abc.v3;
				break;
			case 4:
			case 5:
				i_abc.v3 = -i_abc.v1 - i_abc.v2;
				break;
			}
		}
	}
	void Shunt3PhaseCurrentSensor::config_apply(Shunt3PhaseCurrentSensorConfig& config)
	{
		this->config = config;
		_a_mapper.config.zero_offset = config.i_a_offset;
		_a_mapper.config.value_per_unit = config.i_value_per_unit;

		_b_mapper.config.zero_offset = config.i_b_offset;
		_b_mapper.config.value_per_unit = config.i_value_per_unit;

		_c_mapper.config.zero_offset = config.i_c_offset;
		_c_mapper.config.value_per_unit = config.i_value_per_unit;
	}
	void Shunt3PhaseCurrentSensor::zero_calibrate(Motor& motor)
	{
		motor.reference.d_abc.v1 = 0.0f;
		motor.reference.d_abc.v2 = 0.0f;
		motor.reference.d_abc.v3 = 0.0f;
		Utils::delay(50);

		config.i_a_offset = *config.i_a_buffer;
		config.i_b_offset = *config.i_b_buffer;
		config.i_c_offset = *config.i_c_buffer;

		_a_mapper.config.zero_offset = config.i_a_offset;
		_a_mapper.config.value_per_unit = config.i_value_per_unit;

		_b_mapper.config.zero_offset = config.i_b_offset;
		_b_mapper.config.value_per_unit = config.i_value_per_unit;

		_c_mapper.config.zero_offset = config.i_c_offset;
		_c_mapper.config.value_per_unit = config.i_value_per_unit;

	}
	/**
	 * sec 1: a > b > c;
	 * sec 2: b > a > c;
	 * sec 3: b > c > a;
	 * sec 4: c > b > a;
	 * sec 5: c > a > b;
	 * sec 6: a > c > b;
	 * @param motor
	 * @param i_dq
	 */
	void Shunt3PhaseCurrentSensor::i_ab_get(Motor& motor, Vector2f& i_ab)
	{
		Vector3f abc;
		Vector2f ab;
		abc.v1 = _a_mapper.value_get(*config.i_a_buffer);
		abc.v2 = _b_mapper.value_get(*config.i_b_buffer);
		abc.v3 = _c_mapper.value_get(*config.i_c_buffer);
		if (motor.reference.d_abc.v1 > config.skip_threshold
			|| motor.reference.d_abc.v2 < -config.skip_threshold
			|| motor.reference.d_abc.v3 < -config.skip_threshold)
		{
			switch (motor.reference.section)
			{
			case 1:
			case 6:
				abc.v1 = -abc.v3 - abc.v2;
				ab.v1 = abc.v1;
				ab.v2 = _1_SQRT3 * abc.v1 + _2_SQRT3 * abc.v2;
				break;
			case 2:
			case 3:
				// if only two measured currents
				abc.v2 = -abc.v1 - abc.v3;
				ab.v1 = abc.v1;
				ab.v2 = _1_SQRT3 * abc.v1 + _2_SQRT3 * abc.v2;
			case 4:
			case 5:
				ab.v1 = abc.v1;
				ab.v2 = _1_SQRT3 * abc.v1 + _2_SQRT3 * abc.v2;
				break;
			}
		}
		else
		{
			float mid = _1_3 * (abc.v1 + abc.v2 + abc.v3);
			ab.v1 = abc.v1 - mid;
			ab.v2 = abc.v2 - mid;
		}
		i_ab.v1 = ab.v1;
		i_ab.v2 = _1_SQRT3 * ab.v1 + _2_SQRT3 * ab.v2;
	}

} // wibot::motor
