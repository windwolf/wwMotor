#ifndef __WWMOTOR_POS_SPD_OBSERVER_HPP__
#define __WWMOTOR_POS_SPD_OBSERVER_HPP__

#include "motor/base.hpp"
#include "filter/lp.hpp"
#include "pid.hpp"

namespace wwMotor
{
	using namespace wwControl;
	class PositionSpeedObserver
	{
	 public:
		virtual Vector2f pos_spd_get(Vector3f uabc, Vector3f iabc) = 0;
	};

	struct SmoPllPositionSpeedOboserverConfig
	{
		float sample_time;
		float current_gain;
		float disturbance_gain;
		float cutoff_freq;
		float pll_kp;
		float pll_pi;
	};

	class SmoPllPositionSpeedOboserver
		: public PositionSpeedObserver,
		  public Configurable<SmoPllPositionSpeedOboserverConfig>
	{
	 public:
		SmoPllPositionSpeedOboserver(MotorParameter& motor)
			: _motor(motor)
		{
		};
		void _config_apply() override;

		Vector2f position_speed_get(Vector3f uabc, Vector3f iabc) override;

	 private:
		MotorParameter& _motor;
		FirstOrderLowPassFilter _filter;
		PidController _pid;

		Vector2f _i;
		Vector2f _u;

		Vector2f _i_obs;
		Vector2f _zk;
		Vector2f _i_err;

		Vector2f _e_obs;

		Vector2f _pos_spd;

		float a;
		float b;

		void smo();
		void pll();
		void lpf();
	};

} // namespace wwMotor

#endif // __WWMOTOR_POS_SPD_OBSERVER_HPP__
