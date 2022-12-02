//
// Created by zhouj on 2022/11/17.
//

#include "Shunt3PhaseCurrentSensor.hpp"
#include "os.hpp"

namespace wwMotor2
{
	using namespace ww::os;

	void Shunt3PhaseCurrentSensor::i_abc_get(Motor& motor, Vector3f& i_abc)
	{
		i_abc.v1 = _a_mapper.value_get(*_config.i_a_buffer);
		i_abc.v2 = _b_mapper.value_get(*_config.i_b_buffer);
		i_abc.v3 = _c_mapper.value_get(*_config.i_c_buffer);
		switch (motor.state.section)
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
	void Shunt3PhaseCurrentSensor::config_apply(Shunt3PhaseCurrentSensorConfig& config)
	{
		Configurable::config_apply(config);
		LinearValueMapperConfig cfg{
			.zero_offset = config.i_a_offset,
			.value_per_unit = config.i_value_per_unit,
		};
		_a_mapper.config_apply(cfg);
		cfg.zero_offset = config.i_b_offset,
			_b_mapper.config_apply(cfg);
		cfg.zero_offset = config.i_c_offset,
			_c_mapper.config_apply(cfg);
	}
	void Shunt3PhaseCurrentSensor::zero_calibrate(Motor& motor)
	{
		motor.reference.d_abc.v1 = 0.0f;
		motor.reference.d_abc.v2 = 0.0f;
		motor.reference.d_abc.v3 = 0.0f;
		Utils::delay(50);

		_config.i_a_offset = *_config.i_a_buffer;
		_config.i_b_offset = *_config.i_b_buffer;
		_config.i_c_offset = *_config.i_c_buffer;

		LinearValueMapperConfig cfg;

		cfg.zero_offset = _config.i_a_offset;
		cfg.value_per_unit = _config.i_value_per_unit;
		_a_mapper.config_apply(cfg);

		cfg.zero_offset = _config.i_b_offset;
		_b_mapper.config_apply(cfg);

		cfg.zero_offset = _config.i_c_offset;
		_c_mapper.config_apply(cfg);

	}

} // wwMotor2
