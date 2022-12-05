//
// Created by zhouj on 2022/11/16.
//

#include "FocControl.hpp"
#include "os.hpp"

namespace wibot::motor
{
	using namespace wibot::os;

	void FocControl::command_set(Motor& motor, FocCommand& cmd)
	{
		_cmd = cmd;
	}

	void FocControl::calibrate(Motor& motor)
	{
		auto cmd_mode = _cmd.mode;

		_cmd.mode = FocCommandMode::Calibrate;
		_powerSensor->u_bus_get(motor, motor.state.u_bus);

		/**
		 * 相电流0点校准
		 */
		motor.reference.u_dq.v1 = 0.0f;
		motor.reference.u_dq.v2 = 0.0f;
		os::Utils::delay(100);
		_phaseCurrentSensor->zero_calibrate(motor);

		/**
		 * 校准位置编码器的0点.
		 * 首先将电机转动一个电周期, 最后, D轴对准A相, 然后检测编码器位置.
		 */
		_virtualPositionSpeedSensor->speed_set(0.0f);
		_virtualPositionSpeedSensor->position_set(0.0f);
		motor.reference.u_dq.v1 = motor.state.u_bus * 0.05f;
		motor.reference.u_dq.v2 = 0.0f;
		for (int i = 0; i < 100; ++i)
		{
			_virtualPositionSpeedSensor->position_set(_2PI * (float)i / 100.0f);
			Utils::delay(2);
		}
		Utils::delay(10);
		_positionSpeedSensor->zero_search(motor);
		motor.reference.u_dq.v1 = 0.0f;
		motor.reference.u_dq.v2 = 0.0f;

	}

	void FocControl::outerLoop(Motor& motor)
	{
		_sectionSensor->section_get(motor, motor.state.section);
		if (_cmd.mode == FocCommandMode::OpenLoop)
		{
			// 开环无感模式模式下，才由程序提供虚拟转速及位置信息 TODO: 速度由KV值换算得到
			// _virtualPositionSpeedSensor->speed_set(_cmd.speed);
			// _virtualPositionSpeedSensor->position_speed_get(motor, motor.state.pos_spd_e, motor.state.pos_spd_m);
		}
		else if (_cmd.mode != FocCommandMode::Calibrate)
		{
			// _positionSpeedSensor->position_speed_get(motor, motor.state.pos_spd_e, motor.state.pos_spd_m);
		}

		if (_cmd.mode == FocCommandMode::Position)
		{
			motor.reference.position = _cmd.position;
			_positionController->speed_get(motor, motor.reference.speed);
		}
		else if (_cmd.mode == FocCommandMode::Speed)
		{
			motor.reference.speed = _cmd.speed;
		}

		if (_cmd.mode == FocCommandMode::Speed || _cmd.mode == FocCommandMode::Position)
		{
			_speedController->current_get(motor, motor.reference.i_dq);
		}
		else if (_cmd.mode == FocCommandMode::Current)
		{
			motor.reference.i_dq = _cmd.current;
		}
	}
	void FocControl::innerLoop(Motor& motor)
	{
		_positionSpeedSensor->position_speed_get(motor, motor.state.pos_spd_e, motor.state.pos_spd_m);
		_sectionSensor->section_get(motor, motor.state.section);
		_powerSensor->u_bus_get(motor, motor.state.u_bus);
		_powerSensor->i_bus_get(motor, motor.state.i_bus);
		_phaseCurrentSensor->i_ab_get(motor, motor.state.i_ab);
		//FocMath::abc2ab(motor.state.i_abc, motor.state.i_ab);
		FocMath::ab2dq(motor.state.i_ab, motor.state.pos_spd_e.v1, motor.state.i_dq);

		if (_cmd.mode == FocCommandMode::Speed || _cmd.mode == FocCommandMode::Position
			|| _cmd.mode == FocCommandMode::Current)
		{
			_currentController->voltage_get(motor, motor.reference.u_dq);
		}
		else if (_cmd.mode == FocCommandMode::OpenLoop)
		{
			motor.reference.u_dq = _cmd.voltage;
		}
		else
		{
			// Calibrate
		}

		_modular->module(motor,
			motor.reference.section,
			motor.reference.d_abc,
			motor.reference.u_abc,
			motor.reference.sw_channel,
			motor.reference.d_sample);
		_driver->duty_set(motor);

	}

}
// wibot::motor
