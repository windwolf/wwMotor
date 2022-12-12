//
// Created by zhouj on 2022/11/16.
//

#include "FocControl.hpp"
#include "os.hpp"

namespace wibot::motor
{
	using namespace wibot::os;

	void FocControl::set_command(Motor& motor, FocCommand& cmd)
	{
		_cmd = cmd;
		//TODO: 根据配置, 生成控制轨迹.
	}

	void FocControl::calibrate(Motor& motor)
	{
		auto cmd_mode = _cmd.mode;
		auto u_dq = motor.reference.u_dq;
		auto d_sample = motor.reference.d_sample;
		auto pos_spd_e = motor.state.pos_spd_e;
		auto pos_spd_m = motor.state.pos_spd_m;
		_cmd.mode = FocCommandMode::Calibrate;
		motor.reference.u_dq = Vector2f(0, 0);
		motor.reference.d_sample = 0.0f;
		motor.state.pos_spd_e = Vector2f(0, 0);
		motor.state.pos_spd_m = Vector2f(0, 0);

		_phaseCurrentSensor->calibrate(motor);

		_positionSpeedSensor->calibrate(motor);

		motor.state.pos_spd_e = pos_spd_e;
		motor.state.pos_spd_m = pos_spd_m;
		motor.reference.u_dq = u_dq;
		motor.reference.d_sample = d_sample;
		_cmd.mode = cmd_mode;

	}

	void FocControl::lf_loop(Motor& motor)
	{
		if (_cmd.mode != FocCommandMode::Calibrate && _cmd.mode != FocCommandMode::Stop)
		{
			_positionSpeedSensor->position_speed_get(motor, motor.state.pos_spd_e, motor.state.pos_spd_m);
		}
		_sectionSensor->section_get(motor, motor.state.section);

		// 故意不break, 为了让后面的代码也执行.
		switch (_cmd.mode)
		{
		case FocCommandMode::Position:
			_positionController->speed_get(motor, motor.reference.speed);
		case FocCommandMode::Speed:
			_speedController->current_get(motor, motor.reference.i_dq);
		case FocCommandMode::Current:
		case FocCommandMode::OpenLoop:
		case FocCommandMode::Calibrate:
		case FocCommandMode::Stop:
			break;
		}
	}
	void FocControl::hf_loop(Motor& motor)
	{
		_powerSensor->u_bus_get(motor, motor.state.u_bus);
		_powerSensor->i_bus_get(motor, motor.state.i_bus);

		_phaseCurrentSensor->i_ab_get(motor, motor.state.i_ab);
		//FocMath::abc2ab(motor.state.i_abc, motor.state.i_ab);
		FocMath::ab2dq(motor.state.i_ab, motor.state.pos_spd_e.v1, motor.state.i_dq);

		// 故意不break, 为了让后面的代码也执行.
		switch (_cmd.mode)
		{
		case FocCommandMode::Position:
		case FocCommandMode::Speed:
		case FocCommandMode::Current:
			_currentController->voltage_get(motor, motor.reference.u_dq);
		case FocCommandMode::OpenLoop:
		case FocCommandMode::Calibrate:
			_modular->module(motor,
				motor.reference.section,
				motor.reference.d_abc,
				motor.reference.u_abc,
				motor.reference.sw_channel,
				motor.reference.d_sample);
			break;
		case FocCommandMode::Stop:
			motor.reference.d_abc = Vector3f(0, 0, 0);
			motor.reference.u_abc = Vector3f(0, 0, 0);
			motor.reference.sw_channel = 0x00;
			motor.reference.d_sample = 0.5f;
			break;
		}

		_driver->duty_set(motor);
	}
	void FocControl::command_loop(Motor& motor)
	{
		switch (_cmd.mode)
		{
		case FocCommandMode::Stop:
			break;
		case FocCommandMode::Calibrate:
			break;
		case FocCommandMode::OpenLoop:
			motor.reference.u_dq = _cmd.voltage;
			break;
		case FocCommandMode::Current:
			motor.reference.i_dq = _cmd.current;
			break;
		case FocCommandMode::Speed:
			motor.reference.speed = _cmd.speed;
			break;
		case FocCommandMode::Position:
			motor.reference.position = _cmd.position;
			break;
		}

	}

}
// wibot::motor
