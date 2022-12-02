//
// Created by zhouj on 2022/11/16.
//

#include "foc_control.hpp"

namespace wwMotor2
{
	void FocControl::sensor_get_stage(Motor& motor)
	{
		_powerSensor->u_bus_get(motor, motor.state.u_bus);
		_powerSensor->i_bus_get(motor, motor.state.i_bus);
		_phaseCurrentSensor->i_abc_get(motor, motor.state.i_abc);
		if (_cmd.mode == FocMode::OpenLoop)
		{
			// 开环模式下，由程序提供转速及位置信息
			_virtualPositionSpeedSensor->position_speed_get(motor, motor.state.pos_spd_e, motor.state.pos_spd_m);
		}
		else
		{
			_positionSpeedSensor->position_speed_get(motor, motor.state.pos_spd_e, motor.state.pos_spd_m);
		}

		_sectionSensor->section_get(motor, motor.state.section);
		FocMath::abc2ab(motor.state.i_abc, motor.state.i_ab);
		FocMath::ab2dq(motor.state.i_ab, motor.state.pos_spd_e.v1, motor.state.i_dq);

	}

	void FocControl::driver_execute_stage(Motor& motor)
	{
		FocMath::dq2ab(motor.reference.u_dq, motor.state.pos_spd_e.v1, motor.reference.u_ab);
		_driver->circle_limit(motor, motor.reference.u_ab);
		_driver->phase_voltage_set(motor);

	}

	void FocControl::command_set(Motor& motor, FocCommand& cmd)
	{
		_cmd = cmd;
	}

	void FocControl::current_control_stage(Motor& motor)
	{
		if (_cmd.mode == FocMode::OpenLoop)
		{
			motor.reference.u_dq.v1 = 0;
			motor.reference.u_dq.v2 = motor.state.u_bus;
		}
		else
		{
			if (_cmd.mode == FocMode::Current)
			{
				motor.reference.i_dq = _cmd.current;
			}
			_current_controller->update(motor, motor.reference.u_dq);
		}
	}
	void FocControl::position_speed_control_stage(Motor& motor)
	{
		switch (_cmd.mode)
		{
		case FocMode::Position:
			_position_controller->update(motor, motor.reference.speed);
			// 不要break;
		case FocMode::Speed:
			_speed_controller->update(motor, motor.reference.i_dq);


	}
} // wwMotor2
