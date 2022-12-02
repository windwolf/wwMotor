//
// Created by zhouj on 2022/11/16.
//

#ifndef WWMOTOR_APP_MOTOR2_FOC_CONTROL_HPP_
#define WWMOTOR_APP_MOTOR2_FOC_CONTROL_HPP_

#include "base.hpp"
#include "PowerSensor.hpp"
#include "PhaseCurrentSensor.hpp"
namespace wwMotor2
{
	enum class FocMode
	{
		Current,
		Speed,
		Position,
		OpenLoop,
	};

	struct FocCommand
	{
		FocMode mode = FocMode::OpenLoop;
		union
		{
			float position;
			float speed;
			Vector2f current;
		};
	};

	class FocControl
	{
	 public:
		void command_set(Motor& motor, FocCommand& cmd);
		void sensor_get_stage(Motor& motor);
		void current_control_stage(Motor& motor);
		void driver_execute_stage(Motor& motor);
		void position_speed_control_stage(Motor& motor);
	 private:
		FocCommand _cmd;
		PowerSensor* _powerSensor;
		PhaseCurrentSensor* _phaseCurrentSensor;
	};

} // wwMotor2

#endif //WWMOTOR_APP_MOTOR2_FOC_CONTROL_HPP_
