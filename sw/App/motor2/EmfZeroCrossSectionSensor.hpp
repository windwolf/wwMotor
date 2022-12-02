//
// Created by zhouj on 2022/11/17.
//

#ifndef WWMOTOR_APP_MOTOR2_EMFZEROCROSSSECTIONSENSOR_HPP_
#define WWMOTOR_APP_MOTOR2_EMFZEROCROSSSECTIONSENSOR_HPP_

#include "SectionSensor.hpp"
#include "base.hpp"
#include "SectionSwitcher.hpp"
#include "lp.hpp"

namespace wibot::motor
{
	using namespace wibot::control;

	struct EmfZeroCrossSectionSensorConfig
	{
		uint32_t* u_a_port;
		uint32_t* u_b_port;
		uint32_t* u_c_port;
		float cutoff_freq; // 务必大于最大电频率.
		float sample_time;
		uint32_t blank_count; // 忽略换向初期若干各采样点. 最大值: 12/最大电速度/sample_time.
		/**
		 * 换相角相位补偿系统. 单位: s/s.
		 * 换相延迟: 30°-电角速度*补偿系数*采样间隔.
		 * 硬件滤波加数字滤波的相位延迟:补偿系数*采样间隔
		 * 其代表了相位延迟时间换算成采样点数.
		 * 可根据滤波器的相位延迟时间/采样时间得到.
		 */
		uint32_t switch_delay_count;
	};
	/**
	 * @brief 反电动势过零检测, 区间估计器
	 * 采样需在开关开通时
	 * 滤波
	 * 滤波后的相位补偿
	 * 高速: 采3相, 30度偏移;
	 * 低速: 采1相, 90度偏移;
	 * 换向初期的blank zone;
	 * 过零阈值的自动调整
	 *
	 */
	class EmfZeroCrossSectionSensor :
		public SectionSensor,
		public SectionSwitcher,
		public Configurable<EmfZeroCrossSectionSensorConfig>
	{
	 public:
		void config_apply(EmfZeroCrossSectionSensorConfig& config) override;
		void section_get(Motor& motor, uint8_t& section) override;

		void section_index_calibrate(wibot::motor::Motor& motor) override;
		void section_switch(Motor& motor, uint8_t& section) override;
	 private:

		bool zero_cross_detect(Motor& motor);
		void switch_delay_set(Motor& motor);
		void emf_get(Motor& motor);

		bool _is_slow = true;
		uint8_t _last_section;
		float _zero_offset; // 零点位置的偏移
		float _3emf_a;
		float _3emf_b;
		float _3emf_c;
		float _last_3emf_a;
		float _last_3emf_b;
		float _last_3emf_c;
		uint32_t _last_zero_cross_tick;
		uint32_t _zero_cross_span;

		uint32_t _section_switch_delay_count;
		uint8_t _section_switch_event_pending;

		uint32_t _tick;
		uint32_t _blank_count;
	};

} // wibot::motor

#endif //WWMOTOR_APP_MOTOR2_EMFZEROCROSSSECTIONSENSOR_HPP_
