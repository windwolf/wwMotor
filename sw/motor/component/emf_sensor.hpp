#ifndef __WWMOTOR_EMF_SERSOR_HPP__
#define __WWMOTOR_EMF_SERSOR_HPP__
#include "motor/base.hpp"
#include "buffer.hpp"
namespace wwMotor
{
	using namespace ww;
	class EmfSensor
	{
	 public:
	 public:
		virtual Vector3f abc_get(uint8_t section) = 0;
		virtual bool zero_cross_detect(uint8_t section) = 0;
	};

	struct SixStepEmfSensorConfig
	{
		uint16_t* a_phase_value;
		uint16_t* b_phase_value;
		uint16_t* c_phase_value;
		float sample_time;
		uint8_t skip_sample_count; // skip the first few samples after section switch
	};

	class SixStepEmfSensor : public EmfSensor, public Configurable<SixStepEmfSensorConfig>
	{
	 public:
		Vector3f abc_get(uint8_t section) override;
		bool zero_cross_detect(uint8_t section) override;

		void _config_apply() override;

	 private:

		int32_t _last_3emf;
		int8_t _last_section;
		uint32_t _last_section_switch_tick;
		uint32_t _last_zero_cross_tick;
		uint32_t _tick;
		bool _zero_cross_flag;
	};

} // namespace wwMotor

#endif // __WWMOTOR_EMF_SERSOR_HPP__
