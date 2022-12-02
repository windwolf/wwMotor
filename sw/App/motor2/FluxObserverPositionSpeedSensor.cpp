//
// Created by zhouj on 2022/11/17.
//

#include "FluxObserverPositionSpeedSensor.hpp"
#include "math.h"
#include "math_shared.hpp"

namespace wibot::motor
{
	void FluxObserverPositionSpeedSensor::config_apply(FluxObserverPositionSpeedSensorConfig& config)
	{
		Configurable::config_apply(config);

		auto pidcfg = PidControllerConfig{
			.mode = PidControllerMode::Serial,
			.Kp = _config.pll_kp,
			.Ki = _config.pll_pi,
			.Kd = 0,
			.tau = 0,
			.output_limit_enable = true,
			.output_limit_max = config.motor_parameter->speed_limit,
			.output_limit_min = -config.motor_parameter->speed_limit,
			.integrator_limit_enable = true,
			.integrator_limit_max = config.motor_parameter->speed_limit,
			.integrator_limit_min = -config.motor_parameter->speed_limit,
			.sample_time = _config.sample_time,
		};
		_pid.config_apply(pidcfg);
		auto filtercfg = FirstOrderLowPassFilterConfig{
			.sample_time = _config.sample_time,
			.cutoff_freq = _config.cutoff_freq
		};
		_filter.config_apply(filtercfg);
		_a = exp(-1.0f * config.motor_parameter->rs / config.motor_parameter->ld * _config.sample_time);
		_b = (1 - _a) / config.motor_parameter->rs;

	}
	void FluxObserverPositionSpeedSensor::position_speed_get(Motor& motor, Vector2f& pos_spd_e, Vector2f& pos_spd_m)
	{
		// SMO
		FocMath::abc2ab(motor.state.i_abc, _i);
		FocMath::abc2ab(motor.state.u_abc, _u);

		Vector2f i_err_now = _i_obs - _i;
		Vector2f zk = Math::sign(i_err_now) * _config.current_gain;
		_e_obs = (i_err_now - _i_err * _a + _zk) * _b + _e_obs;

		_i_obs = (_u - _e_obs) * _b - zk + _i_obs * _a;
		_zk = zk;
		_i_err = i_err_now;


		// PLL
		float e_alpha = _filter.filter(_e_obs.v1);
		float e_beta = _filter.filter(_e_obs.v2);

		float sin, cos;
		Math::sincos(motor.state.pos_spd_e.v1, &sin, &cos);

		float err = e_alpha * cos + e_beta * sin; // 下一步pid作用在0-sum上, 所以这里不用取负值了
		pos_spd_e.v2 = _pid.update(0, err);
		pos_spd_e.v1 += pos_spd_e.v2 * _config.sample_time;
		pos_spd_m.v2 = pos_spd_e.v2 / _config.motor_parameter->pole_pair;
		pos_spd_m.v1 = pos_spd_e.v1 / _config.motor_parameter->pole_pair;
	}
	void FluxObserverPositionSpeedSensor::zero_search(Motor& motor)
	{
	}

} // wibot::motor
