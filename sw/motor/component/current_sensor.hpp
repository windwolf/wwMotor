#ifndef __wwMotor_CURRENT_SENSOR_HPP__
#define __wwMotor_CURRENT_SENSOR_HPP__

#include "motor/base.hpp"
#include "base/base.hpp"
#include "buffer.hpp"
#include "framework/framework.hpp"

namespace wwMotor
{
	using namespace ww;
	using namespace wwControl;
	class CurrentSensor
	{
	 public:
	 protected:
	};

	struct CurrentSensor3ShuntConfig
	{
		uint32_t maxScaleValue;
		float refVoltage;
		float shuntResistance;
		float ampGain;
		uint16_t* aPhase;
		uint16_t* bPhase;
		uint16_t* cPhase;
	};

	class CurrentSensor3Shunt : public CurrentSensor, public Configurable<CurrentSensor3ShuntConfig>
	{

	 public:

		void _config_apply() override;

		Vector3f phase_current_get(uint8_t section);

	 protected:
		float _currentPerUnit; // refVoltage / maxScaleValue / shuntResistance / ampGain
	};

	struct CurrentSensor1ShuntConfig
	{
		uint32_t maxScaleValue;
		float refVoltage;
		float shuntResistance;
		float ampGain;
		uint16_t* bus_current_value;
	};
	class CurrentSensor1Shunt : public CurrentSensor, public Configurable<CurrentSensor1ShuntConfig>
	{
	 public:

		void _config_apply() override;
		float bus_current_get();

	 protected:
		float _currentPerUnit; // refVoltage / maxScaleValue / shuntResistance / ampGain
	};

} // namespace wwMotor

#endif // __wwMotor_CURRENT_SENSOR_HPP__
