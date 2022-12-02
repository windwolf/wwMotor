#include "pos_spd_observer.hpp"
#include "math.h"
#include "foc_math.hpp"

namespace wwMotor
{

	void SmoPllPositionSpeedOboserver::_config_apply()
	{
		auto pidcfg = PidControllerConfig{
			.mode = PidControllerMode::Serial,
			.Kp = _config.pll_kp,
			.Ki = _config.pll_pi,
			.Kd = 0,
			.tau = 0,
			.output_limit_enable = false,
			.integrator_limit_enable = false,
			.sample_time = _config.sample_time,
		};
		_pid.config_set(pidcfg);
		auto filtercfg = FirstOrderLowPassFilterConfig{
			.sample_time = _config.sample_time,
			.cutoff_freq = _config.cutoff_freq
		};
		_filter.config_set(filtercfg);
		a = exp(-1.0f * _motor.rs / _motor.ld * _config.sample_time);
		b = (1 - a) / _motor.rs;
	};
	Vector2f SmoPllPositionSpeedOboserver::position_speed_get(Vector3f uabc, Vector3f iabc)
	{
		FocMath::abc2ab(iabc, _i);
		FocMath::abc2ab(uabc, _u);
		smo();
		pll();
		return _pos_spd;
	};

	void SmoPllPositionSpeedOboserver::smo()
	{
		Vector2f i_err_now = _i_obs - _i;
		Vector2f zk = Math::sign(i_err_now) * _config.current_gain;
		_e_obs = (i_err_now - _i_err * a + _zk) * b + _e_obs;

		_i_obs = (_u - _e_obs) * b - zk + _i_obs * a;
		_zk = zk;
		_i_err = i_err_now;
	};

	void SmoPllPositionSpeedOboserver::pll()
	{
		float e_alpha = _filter.filter(_e_obs.v1);
		float e_beta = _filter.filter(_e_obs.v2);

		float sin, cos;
		Math::sincos(_pos_spd.v1, &sin, &cos);

		float err = e_alpha * cos + e_beta * sin; // 下一步pid作用再0-sum上, 所以这里不用取负值了
		_pos_spd.v2 = _pid.update(0, err);
		_pos_spd.v1 += _pos_spd.v2 * _config.sample_time;
	};

} // namespace wwMotor
