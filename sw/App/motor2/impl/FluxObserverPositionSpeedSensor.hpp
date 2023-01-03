//
// Created by zhouj on 2022/11/17.
//

#ifndef WWMOTOR_APP_MOTOR2_FLUXOBSERVERPOSITIONSPEEDSENSOR_HPP_
#define WWMOTOR_APP_MOTOR2_FLUXOBSERVERPOSITIONSPEEDSENSOR_HPP_

#include "motor2/PositionSpeedSensor.hpp"
#include "lp.hpp"
#include "pid.hpp"

namespace wibot::motor
{
	using namespace wibot::control;
	struct FluxObserverPositionSpeedSensorConfig
	{
		float sample_time;
		float current_gain;
		float disturbance_gain;
		float cutoff_freq;
		float pll_kp;
		float pll_pi;
		MotorParameter* motor_parameter;
	};

	class FluxObserverPositionSpeedSensor :
		public PositionSpeedSensor, public Configurable<FluxObserverPositionSpeedSensorConfig>
	{
	 public:
		Result apply_config() override;

		void position_speed_get(Motor& motor, Vector2f& position, Vector2f& speed) override;

		void calibrate(Motor& motor);
	 private:
		FirstOrderLowPassFilter _filter;
		PidController _pid;

		Vector2f _i;
		Vector2f _u;

		Vector2f _i_obs;
		Vector2f _zk;
		Vector2f _i_err;

		Vector2f _e_obs;

		Vector2f _pos_spd;
		float _a;
		float _b;

		void smo();
		void pll();
		void lpf();
	};

} // wibot::motor

#endif //WWMOTOR_APP_MOTOR2_FLUXOBSERVERPOSITIONSPEEDSENSOR_HPP_
