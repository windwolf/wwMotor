//
// Created by zhouj on 2022/11/28.
//

#include "SixStepControl.hpp"

namespace wibot::motor
{
	void SixStepControl::state_get_stage(Motor& motor)
	{
		_power_sensor->u_bus_get(motor, motor.state.u_bus);
		_power_sensor->i_bus_get(motor, motor.state.i_bus);
		_phase_voltage_sensor->u_abc_get(motor, motor.state.u_abc);
		_phase_current_sensor->i_abc_get(motor, motor.state.i_abc);
		_section_sensor->section_get(motor, motor.state.section);

	}
	void SixStepControl::driver_execute_stage(Motor& motor)
	{
		_modular->module(motor);
		_driver->duty_set(motor);
	}
	void SixStepControl::current_control_stage(Motor& motor)
	{
		_current_ctrl->duty_get(motor, motor.reference.d_pwm);
	}
} // wibot::motor
