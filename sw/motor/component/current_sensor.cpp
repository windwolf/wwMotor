#include "current_sensor.hpp"

namespace wwMotor
{

	void CurrentSensor3Shunt::_config_apply()
	{
		_currentPerUnit =
			_config.refVoltage / (float)_config.maxScaleValue / _config.shuntResistance / _config.ampGain;
	};

	Vector3f CurrentSensor3Shunt::phase_current_get(uint8_t section)
	{
		Vector3f current;
		current.v1 = (float)_config.aPhase[section] * _currentPerUnit;
		current.v2 = (float)_config.bPhase[section] * _currentPerUnit;
		current.v3 = (float)_config.cPhase[section] * _currentPerUnit;
		switch (section)
		{
		case 1:
		case 6:
			current.v1 = -current.v2 - current.v3;
			break;
		case 2:
		case 3:
			current.v2 = -current.v1 - current.v3;
			break;
		case 4:
		case 5:
			current.v3 = -current.v1 - current.v2;
			break;
		}
		return current;
	};

	void CurrentSensor1Shunt::_config_apply()
	{
		_currentPerUnit =
			_config.refVoltage / (float)_config.maxScaleValue / _config.shuntResistance / _config.ampGain;
	}

	float CurrentSensor1Shunt::bus_current_get()
	{
		float current = (float)(*_config.bus_current_value) * _currentPerUnit;

		return current;
	};

} // namespace wwMotor
