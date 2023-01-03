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
#include "motor2/impl/PidPositionController.hpp"
#include "motor2/impl/FocPidSpeedController.hpp"
#include "motor2/impl/PidWithFeedforwardCurrentController.hpp"
#include "Modular.hpp"
#include "Driver.hpp"
#include "motor2/impl/VirtualPositionSpeedSensor.hpp"
#include "motor2/impl/Shunt3PhaseCurrentSensor.hpp"
#include "motor2/impl/AbsoluteEncoderPositionSpeedSensor.hpp"
#include "motor2/impl/SvpwmModular.hpp"
#include "motor2/impl/DirectSectionSensor.hpp"
#include "motor2/impl/SamplingPowerSensor.hpp"
#include "motor2/platform/PwmDriver.hpp"

namespace wibot::motor
{

	class FocControlBase
	{
	 public:
		FocControlBase(PowerSensor* power_sensor,
			PhaseCurrentSensor* phase_current_sensor,
			PositionSpeedSensor* position_speed_sensor,
			SectionSensor* section_sensor,
			SpeedReferenceUpdater* speedReferenceUpdater,
			DqCurrentReferenceUpdater* dqCurrentReferenceUpdater,
			DqVoltageReferenceUpdater* dqVoltageReferenceUpdater,
			Modular* modular,
			Driver* driver) : _powerSensor(power_sensor),
							  _phaseCurrentSensor(phase_current_sensor),
							  _positionSpeedSensor(position_speed_sensor),
							  _sectionSensor(section_sensor),
							  _modular(modular),
							  _driver(driver),
							  _dqVoltageReferenceUpdater(dqVoltageReferenceUpdater),
							  _dqCurrentReferenceUpdater(dqCurrentReferenceUpdater),
							  _speedReferenceUpdater(speedReferenceUpdater)
		{
		};

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
		PowerSensor* _powerSensor;
		PhaseCurrentSensor* _phaseCurrentSensor;
		PositionSpeedSensor* _positionSpeedSensor;
		SectionSensor* _sectionSensor;
		Modular* _modular;
		Driver* _driver;
		DqVoltageReferenceUpdater* _dqVoltageReferenceUpdater;
		DqCurrentReferenceUpdater* _dqCurrentReferenceUpdater;
		SpeedReferenceUpdater* _speedReferenceUpdater;

	};

	struct FocCommand
	{
		FocCommand()
		{
		};
		MotorRunMode mode = MotorRunMode::Stop;
		union
		{
			float position;
			float speed;
			Vector2f current;
			Vector2f voltage;
		};

	};

	struct FocControlConfig
	{
		struct
		{
			DataSource* u_bus;
			float u_bus_pu;
		} power_sensor;

		struct
		{
			DataSource* i_a;
			DataSource* i_b;
			DataSource* i_c;
			float i_pu;
			float low_duty_skip_threshold;
		} current_sensor;

		struct
		{
			bool allow_over_module;
		} modular;

		struct
		{
			DataSource* codex;
			uint32_t resolution;
			EncoderDirection direction;
			float calibration_voltage;
		} encoder;

		struct
		{
			float bw;
			bool disableFeedforward;
		} current_controller;

		struct
		{
			float delta;
		} speed_controller;

		struct
		{
			float kp;
			float ki;
			float kd;
		} position_controller;

		MotorParameter* motor_parameter;
		float high_frequency_samlpe_time;
		float low_frequency_samlpe_time;
	};

	class FocControl : public Configurable<FocControlConfig>
	{
	 public:
		FocControl(PwmDriver* driver)
			: _driver(driver),
			  _focControl(
				  &_powerSensor,
				  &_phaseCurrentSensor,
				  &_positionSpeedSensor,
				  &_sectionSensor,
				  &_positionController,
				  &_speedController,
				  &_currentController,
				  &_modular,
				  _driver)
		{
		};

		Result apply_config() override;

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
		PwmDriver* _driver;
		SamplingPowerSensor _powerSensor;
		Shunt3PhaseCurrentSensor _phaseCurrentSensor;
		AbsoluteEncoderPositionSpeedSensor _positionSpeedSensor;
		DirectSectionSensor _sectionSensor;
		SvpwmModular _modular;

		PidWithFeedforwardCurrentController _currentController;
		FocPidSpeedController _speedController;
		PidPositionController _positionController;

		FocControlBase _focControl;
		FocCommand _cmd;
	};

} // wibot::motor

#endif //WWMOTOR_APP_MOTOR2_FOCCONTROL_HPP_
