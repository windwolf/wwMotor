#include "foc_control.hpp"
#include "motor/component/foc_math.hpp"
#include "base/base.hpp"
#include "os/os.hpp"
namespace wwMotor
{

	void FocControl::init()
	{
		_power.config_set(_config.power_cfg);
		_position_speed_sensor.config_set(_config.pos_spd_sensor_cfg);
		_current_sensor.config_set(_config.curr_sensor_cfg);
		_position_controller.config_set(_config.pos_ctrl_cfg);
		_speed_controller.config_set(_config.spd_ctrl_cfg);
		_current_controller.config_set(_config.curr_ctrl_cfg);
		_driver.config_set(_config.drv_cfg);
		_state.u_bus = _power.ubus_get();

		_position_speed_sensor.init();
		_driver.phase_voltage_set(Vector2f(), _state.u_bus);
	};

	void FocControl::calibrate()
	{
		Vector2f u_ab;
		_state.u_bus = _power.ubus_get();

		u_ab.v1 = _state.u_bus * 0.1f;
		u_ab.v2 = 0.0f;
		_position_speed_sensor.calibrate_begin();
		_driver.phase_voltage_set(u_ab, _state.u_bus);
		for (size_t i = 0; i < 10; i++)
		{
			_position_speed_sensor.calibrate();
			ww::os::Utils::delay(10);
		}
		_position_speed_sensor.calibrate_end();
		u_ab.v1 = 0.0f;
		_driver.phase_voltage_set(u_ab, _state.u_bus);
	};

	void FocControl::update()
	{
		_state.u_bus = _power.ubus_get();

		if (_mode != FocMode::OpenLoop)
		{
			_position_speed_sensor.position_speed_get(_state.pos_spd_e, _state.pos_spd_m);
			_state.section = FocMath::section_get(_state.pos_spd_e.v1);
			_state.i_abc = _current_sensor.phase_current_get(_state.section);
			FocMath::abc2ab(_state.i_abc, _state.i_ab);
			FocMath::ab2dq(_state.i_ab, _state.pos_spd_e.v1, _state.i_dq);
		}

		switch (_mode)
		{
		case FocMode::Position:
			_state.speed_ref = _position_controller.update(_state.position_ref, 1, _state.pos_spd_m.v1);
		case FocMode::Speed:
			_state.i_dq_ref = _speed_controller.update(_state.speed_ref, _state.pos_spd_e.v1);
		case FocMode::Current:
			_state.u_dq_ref =
				_current_controller.update(_state.i_dq_ref, _state.i_dq, _state.pos_spd_e.v2);
			break;
		case FocMode::OpenLoop:
			_state.position_ref = _state.speed_ref * _config.spd_ctrl_cfg.sample_time;
			_state.u_dq_ref.v1 = 0;
			_state.u_dq_ref.v2 = _state.u_bus;
			_state.pos_spd_m.v2 = _state.speed_ref;
			_state.pos_spd_e.v2 = _state.speed_ref * _motor.polePair;
			_state.pos_spd_m.v1 = Math::circle_normalize(
				_state.pos_spd_e.v1 + _state.speed_ref * _config.spd_ctrl_cfg.sample_time);
			_state.pos_spd_e.v2 = Math::circle_normalize(_state.pos_spd_e.v1 * _motor.polePair);

			break;
		default:
			break;
		}

		FocMath::dq2ab(_state.u_dq_ref, _state.pos_spd_e.v1, _state.u_ab_ref);
		_state.u_ab_ref = _driver.circle_limit(_state.u_ab_ref, _state.u_bus);
		_driver.phase_voltage_set(_state.u_ab_ref, _state.u_bus);
	};

	void FocControl::current_loop_update()
	{
		_state.u_bus = _power.ubus_get();

		if (_mode != FocMode::OpenLoop)
		{
			_position_speed_sensor.position_speed_get(_state.pos_spd_e, _state.pos_spd_m);
			_state.section = FocMath::section_get(_state.pos_spd_e.v1);
			_state.i_abc = _current_sensor.phase_current_get(_state.section);
			FocMath::abc2ab(_state.i_abc, _state.i_ab);
			FocMath::ab2dq(_state.i_ab, _state.pos_spd_e.v1, _state.i_dq);
		}

		switch (_mode)
		{
		case FocMode::Position:
			_state.speed_ref = _position_controller.update(_state.position_ref, 1, _state.pos_spd_m.v1);
		case FocMode::Speed:
			_state.i_dq_ref = _speed_controller.update(_state.speed_ref, _state.pos_spd_e.v1);
		case FocMode::Current:
			_state.u_dq_ref =
				_current_controller.update(_state.i_dq_ref, _state.i_dq, _state.pos_spd_e.v2);
			break;
		case FocMode::OpenLoop:
			_state.position_ref = _state.speed_ref * _config.spd_ctrl_cfg.sample_time;
			_state.u_dq_ref.v1 = 0;
			_state.u_dq_ref.v2 = _state.u_bus;
			_state.pos_spd_m.v2 = _state.speed_ref;
			_state.pos_spd_e.v2 = _state.speed_ref * _motor.polePair;
			_state.pos_spd_m.v1 = Math::circle_normalize(
				_state.pos_spd_e.v1 + _state.speed_ref * _config.spd_ctrl_cfg.sample_time);
			_state.pos_spd_e.v2 = Math::circle_normalize(_state.pos_spd_e.v1 * _motor.polePair);

			break;
		default:
			break;
		}

		FocMath::dq2ab(_state.u_dq_ref, _state.pos_spd_e.v1, _state.u_ab_ref);
		_state.u_ab_ref = _driver.circle_limit(_state.u_ab_ref, _state.u_bus);
		_driver.phase_voltage_set(_state.u_ab_ref, _state.u_bus);
	};

} // namespace wwMotor
