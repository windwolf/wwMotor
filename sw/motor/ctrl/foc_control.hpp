#ifndef __WWMOTOR_FOC_FOC_CONTROL_HPP__
#define __WWMOTOR_FOC_FOC_CONTROL_HPP__

#include "motor/base.hpp"
#include "framework/framework.hpp"
#include "motor/component/driver.hpp"
#include "motor/component/power.hpp"
#include "motor/component/driver.hpp"
#include "motor/component/current_controller.hpp"
#include "motor/component/speed_controller.hpp"
#include "motor/component/position_controller.hpp"
#include "motor/component/position_speed_sensor.hpp"
#include "motor/component/current_sensor.hpp"

namespace wwMotor
{
	using namespace wwControl;
	enum class FocMode
	{
		Current,
		Speed,
		Position,
		OpenLoop,
	};

	class FocControl
	{
	 public:
		struct Config
		{

			EncoderPositionSpeedSensorConfig pos_spd_sensor_cfg;
			CurrentSensor3ShuntConfig curr_sensor_cfg;
			CurrentControllerConfig curr_ctrl_cfg;
			SpeedControllerConfig spd_ctrl_cfg;
			PositionControllerConfig pos_ctrl_cfg;
			DriverSVPWMConfig drv_cfg;
			SimplePowerConfig power_cfg;
		};

	 public:
		FocControl(Config& config, MotorParameter& motor, DriverExecutor& driverExecutor)
			: _config(config), _motor(motor),
			  _driverExecutor(driverExecutor),
			  _current_controller(_motor),
			  _position_controller(_motor),
			  _speed_controller(_motor),
			  _driver(driverExecutor)
		{

		};

		void init();
		void calibrate();
		void update();

		void mode_set(FocMode mode);

		void speed_position_loop_update();
		void current_loop_update();

	 protected:
		Config _config;

		MotorParameter& _motor;
		DriverExecutor& _driverExecutor;
		DriverSVPWM _driver;
		SimplePower _power;
		EncoderPositionSpeedSensor _position_speed_sensor;
		CurrentSensor3Shunt _current_sensor;
		PositionController _position_controller;
		SpeedController _speed_controller;
		CurrentController _current_controller;

		MotorState _state;

		FocMode _mode;
	};

} // namespace wwMotor

#endif // __WWMOTOR_FOC_FOC_CONTROL_HPP__
