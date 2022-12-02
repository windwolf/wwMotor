//
// Created by zhouj on 2022/11/28.
//

#ifndef WWMOTOR_APP_MOTOR2_SIXSTEPCONTROLLER_HPP_
#define WWMOTOR_APP_MOTOR2_SIXSTEPCONTROLLER_HPP_

#include "SectionSensor.hpp"
#include "SectionSwitcher.hpp"
#include "PowerSensor.hpp"
#include "SpeedController.hpp"
#include "DqCurrentController.hpp"
#include "Modular.hpp"
#include "Driver.hpp"
#include "PhaseVoltageSensor.hpp"
#include "PhaseCurrentSensor.hpp"
#include "BusCurrentController.hpp"
namespace wwMotor2
{
	enum class SixStepMode
	{
		Speed,
		Current,
		OpenLoop,
		Calibrate,
	};

	struct SixStepCommand
	{
		SixStepCommand()
		{
		};
		SixStepMode mode;
		union
		{
			float speed;
			Vector2f current;
			Vector2f voltage;
		};

	};

	class SixStepControl
	{
	 public:
		void command_set(Motor& motor, SixStepCommand& cmd);
		void state_get_stage(Motor& motor);
		void calibrate_stage(Motor& motor);
		void speed_control_stage(Motor& motor);
		void current_control_stage(Motor& motor);
		void driver_execute_stage(Motor& motor);
	 private:
		PowerSensor* _power_sensor;
		PhaseVoltageSensor* _phase_voltage_sensor;
		PhaseCurrentSensor* _phase_current_sensor;
		SectionSensor* _section_sensor;
		SpeedController* _speed_ctrl;
		BusCurrentController* _current_ctrl;
		SectionSwitcher* _section_switcher;
		Modular* _modular;
		Driver* _driver;
	};

} // wwMotor2

#endif //WWMOTOR_APP_MOTOR2_SIXSTEPCONTROLLER_HPP_
