//
// Created by zhouj on 2022/12/13.
//

#ifndef WWMOTOR_APP_MOTOR2_DQCURRENTREFERENCEUPDATE_HPP_
#define WWMOTOR_APP_MOTOR2_DQCURRENTREFERENCEUPDATE_HPP_
#include "base.hpp"

namespace wibot::motor
{
	class DqCurrentReferenceUpdater
	{
	 public:
		virtual void dq_current_update(Motor& motor, Vector2f& i_dq) = 0;
	};
}
#endif //WWMOTOR_APP_MOTOR2_DQCURRENTREFERENCEUPDATE_HPP_
