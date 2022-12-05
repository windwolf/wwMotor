//
// Created by zhouj on 2022/11/16.
//

#ifndef WWMOTOR_APP_MOTOR2_FOCCONTROL_HPP_
#define WWMOTOR_APP_MOTOR2_FOCCONTROL_HPP_

#include "base.hpp"
#include "PowerSensor.hpp"
#include "PhaseCurrentSensor.hpp"
#include "PositionSpeedSensor.hpp"
#include "SectionSensor.hpp"
#include "PositionController.hpp"
#include "SpeedController.hpp"
#include "DqCurrentController.hpp"
#include "Modular.hpp"
#include "Driver.hpp"
#include "VirtualPositionSpeedSensor.hpp"

namespace wibot::motor
{
	enum class FocCommandMode
	{
		OpenLoop = 0, // 仅控电压: 控电压
		Position,
		Speed,
		Current,
		Calibrate,
	};

	struct FocCommand
	{
		FocCommand()
		{
		};
		FocCommandMode mode = FocCommandMode::OpenLoop;
		union
		{
			float position;
			float speed;
			Vector2f current;
			Vector2f voltage;
		};

	};

	class FocControl
	{
	 public:
		FocControl(PowerSensor* power_sensor,
			PhaseCurrentSensor* phase_current_sensor,
			PositionSpeedSensor* position_speed_sensor,
			VirtualPositionSpeedSensor* virtual_position_speed_sensor,
			SectionSensor* section_sensor,
			PositionController* position_controller,
			SpeedController* speed_controller,
			DqCurrentController* current_controller,
			Modular* modular,
			Driver* driver) : _powerSensor(power_sensor),
							  _phaseCurrentSensor(phase_current_sensor),
							  _positionSpeedSensor(position_speed_sensor),
							  _virtualPositionSpeedSensor(virtual_position_speed_sensor),
							  _sectionSensor(section_sensor),
							  _positionController(position_controller),
							  _speedController(speed_controller),
							  _currentController(current_controller),
							  _modular(modular),
							  _driver(driver)
		{
		};
		void command_set(Motor& motor, FocCommand& cmd);

		/**
		 * @brief 校准电机机控制器的各环节
		 *
		 * @param motor
		 */
		void calibrate(Motor& motor);

		void innerLoop(Motor& motor);

		void outerLoop(Motor& motor);

	 private:
		FocCommand _cmd;
		PowerSensor* _powerSensor;
		PhaseCurrentSensor* _phaseCurrentSensor;
		VirtualPositionSpeedSensor* _virtualPositionSpeedSensor;
		PositionSpeedSensor* _positionSpeedSensor;
		SectionSensor* _sectionSensor;
		PositionController* _positionController;
		SpeedController* _speedController;
		DqCurrentController* _currentController;
		Modular* _modular;
		Driver* _driver;
	};

} // wibot::motor

#endif //WWMOTOR_APP_MOTOR2_FOCCONTROL_HPP_
