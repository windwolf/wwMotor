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
	enum class FocMode
	{
		Position,
		Speed,
		Current,
		OpenLoop,
		Calibrate,
	};

	struct FocCommand
	{
		FocCommand()
		{
		};
		FocMode mode;
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
			SectionSensor* section_sensor,
			PositionController* position_controller,
			SpeedController* speed_controller,
			DqCurrentController* current_controller,
			Modular* modular,
			Driver* driver) : _powerSensor(power_sensor),
							  _phaseCurrentSensor(phase_current_sensor),
							  _positionSpeedSensor(position_speed_sensor),
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
		 * 获取运行状态
		 * @param motor
		 */
		void state_get_stage(Motor& motor);

		/**
		 * @brief 在Position模式, 计算位置环的输出(速度); 在Speed模式下, 直接将速度设置为设定值.
		 * @param motor
		 */
		void position_control_stage(Motor& motor);

		/**
		 * @brief 在Position或者Speed模式下, 计算速度的输出(电流); 在Current模式下, 直接将电流设置为设定值.
		 * @param motor
		 */
		void speed_control_stage(Motor& motor);
		/**
		 * @brief 在Position,Speed,Current模式下, 计算电流环的输出(电压).
		 * OpenLoop模式无需计算.
		 * @param motor
		 */
		void current_control_stage(Motor& motor);
		/**
		 * @brief 根据u_dq生产SVPWM, 驱动逆变器
		 * @param motor
		 */
		void driver_execute_stage(Motor& motor);

		/**
		 * @brief 校准电机机控制器的各环节
		 *
		 * @param motor
		 */
		void calibrate_stage(Motor& motor);

		/**
		 * @brief 校准电机机控制器的各环节
		 *
		 * @param motor
		 */
		void calibrate(Motor& motor);

	 private:
		FocCommand _cmd;
		PowerSensor* _powerSensor;
		PhaseCurrentSensor* _phaseCurrentSensor;
		VirtualPositionSpeedSensor* _virutalPositionSpeedSensor;
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
