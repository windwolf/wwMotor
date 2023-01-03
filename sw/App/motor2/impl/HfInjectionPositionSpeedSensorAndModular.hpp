//
// Created by zhouj on 2022/12/5.
//

#ifndef WWMOTOR_APP_MOTOR2_HFINJECTIONPOSITIONSPEEDSENSORANDMODULAR_HPP_
#define WWMOTOR_APP_MOTOR2_HFINJECTIONPOSITIONSPEEDSENSORANDMODULAR_HPP_

#include "motor2/PositionSpeedSensor.hpp"
#include "SvpwmModular.hpp"
namespace wibot::motor
{
	struct HfInjectionPositionSpeedSensorConfig
	{
		float injected_frequency;
		float injected_amplitude;
		float sample_time;
		MotorParameter* motor_parameter;
	};

	class HfInjectionPositionSpeedSensorAndModular
		: public PositionSpeedSensor, public Modular, public Configurable<HfInjectionPositionSpeedSensorConfig>
	{
	 public:

		void position_speed_get(Motor& motor, Vector2f& pos_spd_e, Vector2f& pos_spd_m) override;
		void calibrate(Motor& motor) override;
		void module(Motor& motor, uint8_t& section,
			Vector3f& d_abc, Vector3f& u_abc,
			uint8_t& channels, float& d_sample) override;

	 private:
		SvpwmModular _svpwm_modular;
		float _theta;

	};

} // motor

#endif //WWMOTOR_APP_MOTOR2_HFINJECTIONPOSITIONSPEEDSENSORANDMODULAR_HPP_
