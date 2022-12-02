//
// Created by zhouj on 2022/11/21.
//

#ifndef WWMOTOR_APP_MOTOR2_SIXPWMMODULAR_HPP_
#define WWMOTOR_APP_MOTOR2_SIXPWMMODULAR_HPP_

#include "Modular.hpp"
namespace wwMotor2
{
	struct SixPwmModularConfig
	{

	};

	/**  b
	 *  3 \ 2 / 1
	 * ____\/____a
	 *  4 / \  6
	 *  /  5 \
	 * c
	 *  1: c->a; 2: c->b; 3: a->b; 4: a->c; 5: b->c; 6: b->a
	 */
	class SixPwmModular : public Modular, public Configurable<SixPwmModularConfig>
	{
	 public:
		void module(wwMotor2::Motor& motor) override;
	};

} // wwMotor2

#endif //WWMOTOR_APP_MOTOR2_SIXPWMMODULAR_HPP_