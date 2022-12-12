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
		Stop = 0,
		/**
		 * 校准模式. 一切皆有参考得到.
		 */
		Calibrate,
		/**
		 * 开环. 电压由参考提供, 位置信息由传感器或者观测器得到.
		 */
		OpenLoop,
		/**
		 * 电流环. 电流由参考提供, 电压由控制器得到, 位置信息由传感器或者观测器得到.
		 */
		Current,
		/**
		 * 速度环. 速度由参考提供, 电流, 电压均由控制器得到, 位置信息由传感器或者观测器得到.
		 */
		Speed,
		/**
		 * 位置环. 位置由参考提供, 速度, 电流, 电压均由控制器得到, 位置信息由传感器或者观测器得到.
		 */
		Position,

	};

	struct FocCommand
	{
		FocCommand()
		{
		};
		FocCommandMode mode = FocCommandMode::Stop;
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
							  _virtualPositionSpeedSensor(virtual_position_speed_sensor),
							  _positionSpeedSensor(position_speed_sensor),
							  _sectionSensor(section_sensor),
							  _modular(modular),
							  _driver(driver),
							  _currentController(current_controller),
							  _speedController(speed_controller),
							  _positionController(position_controller)
		{
		};
		void set_command(Motor& motor, FocCommand& cmd);

		/**
		 * @brief 校准电机机控制器的各环节
		 *
		 * @param motor
		 */
		void calibrate(Motor& motor);

		/**
		 * 命令解析循环. 主要将外部命令通过设定的轨迹生成器, 生成参考.
		 * @param motor
		 */
		void command_loop(Motor& motor);

		/**
		 * @brief 高频控制循环. 主要为电流环和驱动器调制.
		 *
		 * @param motor
		 */
		void hf_loop(Motor& motor);

		/**
		 * 低频控制循环, 主要为维护和速度环
		 * @param motor
		 */
		void lf_loop(Motor& motor);

	 private:
		FocCommand _cmd;
		PowerSensor* _powerSensor;
		PhaseCurrentSensor* _phaseCurrentSensor;
		VirtualPositionSpeedSensor* _virtualPositionSpeedSensor;
		PositionSpeedSensor* _positionSpeedSensor;
		SectionSensor* _sectionSensor;
		Modular* _modular;
		Driver* _driver;
		DqCurrentController* _currentController;
		SpeedController* _speedController;
		PositionController* _positionController;

	};

} // wibot::motor

#endif //WWMOTOR_APP_MOTOR2_FOCCONTROL_HPP_
