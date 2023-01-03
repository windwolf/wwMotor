//
// Created by zhouj on 2022/11/16.
//

#ifndef WWMOTOR_APP_MOTOR2_BASE_HPP_
#define WWMOTOR_APP_MOTOR2_BASE_HPP_

#include "base/base.hpp"
#include "base/buffer.hpp"
namespace wibot::motor
{
	struct MotorParameter
	{
	 public:
		enum class FluxSetMode
		{
			Flux,
			BackEmfConstant,
		};

	 public:
		uint8_t pole_pair;
		float rs;
		float ld;
		float lq;
		float flux;

		float interia;
		float friction;

		// Limits
		/**
		 * @brief Speed limit (rpm)
		 */
		float speed_limit;
		float u_bus_max;
		float i_bus_limit;
		float i_phase_limit;
	};

	enum class MotorRunMode
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

	struct Motor
	{
		MotorRunMode mode;
		/**
		 * 各控制组件不直接修改state, 而是通过控制器调用各组件后, 由控制器修改state
		 */
		struct State
		{
			float u_bus; // bus voltage
			float i_bus; // bus current

			Vector2f position; // v1: electrical angle, v2: mechanical angle
			Vector2f speed; // v1: electrical speed, v2: mechanical speed
			uint8_t section; // section of electrical position
			Vector3f u_abc; // port voltage
			Vector3f i_abc; // phase current
			Vector2f i_ab;
			/**
			 * For FOC only.
			 */
			Vector2f i_dq;

		} state;

		/**
		 * 各控制组件可直接修改,
		 */
		struct Reference
		{
			Vector2f i_dq;
			float speed;
			float position;

			Vector2f u_dq;
			Vector2f u_ab;
			uint8_t section;
			Vector3f d_abc;

			float d_sample;

			/**
			 * For FOC control, all channels are enabled.
			 * For 6Step control, channels are switched by Modular.
			 * 0x01: A, 0x02: B, 0x04: C, 0x08: D
			 *
			 */
			uint8_t sw_channel;

			/**
			 * only for 6 Steps
			 */
			float i_bus;
			/**
			 * Only for 6 step control
			 *
			 */
			float d_bus;

			Vector3f u_abc;

		} reference;

	};

	class FocMath
	{
	 public:
		static void abc2ab(Vector3f abc, Vector2f& ab);
		static void ab2dq(Vector2f ab, float theta, Vector2f& dq);
		static void dq2ab(Vector2f dq, float theta, Vector2f& ab);
		static uint8_t section_get(float theta);
	};

} // wibot::motor

#endif //WWMOTOR_APP_MOTOR2_BASE_HPP_
