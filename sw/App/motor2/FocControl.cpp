//
// Created by zhouj on 2022/11/16.
//

#include "FocControl.hpp"
#include "os.hpp"

namespace wwMotor2
{
	using namespace ww::os;

	void FocControl::state_get_stage(Motor& motor)
	{

		_sectionSensor->section_get(motor, motor.state.section);
		_powerSensor->u_bus_get(motor, motor.state.u_bus);
		_powerSensor->i_bus_get(motor, motor.state.i_bus);
		_phaseCurrentSensor->i_abc_get(motor, motor.state.i_abc);

		if (_cmd.mode == FocMode::OpenLoop)
		{
			// 开环模式下，由程序提供转速及位置信息
			_virutalPositionSpeedSensor->position_speed_get(motor, motor.state.pos_spd_e, motor.state.pos_spd_m);
		}
		else
		{
			_positionSpeedSensor->position_speed_get(motor, motor.state.pos_spd_e, motor.state.pos_spd_m);
		}

		FocMath::abc2ab(motor.state.i_abc, motor.state.i_ab);
		FocMath::ab2dq(motor.state.i_ab, motor.state.pos_spd_e.v1, motor.state.i_dq);

	}

	void FocControl::command_set(Motor& motor, FocCommand& cmd)
	{
		_cmd = cmd;
	}

	void FocControl::position_control_stage(Motor& motor)
	{
		if (_cmd.mode == FocMode::Position)
		{
			_positionController->speed_get(motor, motor.reference.speed);
		}
		else if (_cmd.mode == FocMode::Speed)
		{
			motor.reference.speed = _cmd.speed;
		}
	};

	void FocControl::speed_control_stage(Motor& motor)
	{
		if (_cmd.mode == FocMode::Speed || _cmd.mode == FocMode::Position)
		{
			_speedController->current_get(motor, motor.reference.i_dq);
		}
		else if (_cmd.mode == FocMode::Current)
		{
			motor.reference.i_dq = _cmd.current;
		}
	};

	void FocControl::current_control_stage(Motor& motor)
	{
		if (_cmd.mode == FocMode::Speed || _cmd.mode == FocMode::Position || _cmd.mode == FocMode::Current)
		{
			_currentController->voltage_get(motor, motor.reference.u_dq);
		}
		else
		{
			// if (_cmd.mode == OpenLoop)
			motor.reference.u_dq = _cmd.voltage;
		}
	}

	void FocControl::driver_execute_stage(Motor& motor)
	{
		_modular->module(motor);

		_driver->duty_set(motor);
	}

	void FocControl::calibrate(Motor& motor)
	{
		_powerSensor->u_bus_get(motor, motor.state.u_bus);
		/**
		 * 相电流0点校准
		 */
		_phaseCurrentSensor->zero_calibrate(motor);

		_virutalPositionSpeedSensor->position_set(0);
		_virutalPositionSpeedSensor->speed_set(0.01);
		/**
		 * 校准位置编码器的0点.
		 * 首先将电机转动一个电周期, 最后, D轴对准A相, 然后检测编码器位置.
		 */
		_virutalPositionSpeedSensor->speed_set(0.0f);
		_virutalPositionSpeedSensor->position_set(0.0f);
		motor.reference.u_dq.v1 = motor.state.u_bus * 0.05f;
		motor.reference.u_dq.v2 = 0.0f;
		for (int i = 0; i < 100; ++i)
		{
			_virutalPositionSpeedSensor->position_set(_2PI * (float)i / 100.0f);
			Utils::delay(2);
		}
		Utils::delay(10);
		_positionSpeedSensor->zero_search(motor);
		motor.reference.u_dq.v1 = 0.0f;
		motor.reference.u_dq.v2 = 0.0f;

	}
	
	void FocControl::calibrate_stage(Motor& motor)
	{
		_powerSensor->u_bus_get(motor, motor.state.u_bus);
		_virutalPositionSpeedSensor->position_speed_get(motor, motor.state.pos_spd_e, motor.state.pos_spd_m);
		_modular->module(motor);
		_driver->duty_set(motor);
	}
}
// wwMotor2
