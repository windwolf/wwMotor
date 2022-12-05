//
// Created by zhouj on 2022/11/17.
//

#include "test.hpp"
#include "main.h"
#include "stm32g4xx_hal.h"
#include "stm32g4xx_hal_tim.h"
#include "stm32g4xx_hal_tim_ex.h"
#include "stm32g4xx_ll_tim.h"
#include "stm32g4xx_hal_adc.h"
#include "stm32g4xx_hal_adc_ex.h"
#include "stm32g4xx_ll_adc.h"
#include "tx_api.h"
#include "peripheral/pwm.hpp"
#include "peripheral/misc.hpp"

#include "motor2/FocControl.hpp"
#include "motor2/base.hpp"
#include "motor2/PowerSensor.hpp"
#include "motor2/ShuntPowerSensor.hpp"
#include "motor2/SimplePowerSensor.hpp"
#include "motor2/PhaseCurrentSensor.hpp"
#include "motor2/Shunt3PhaseCurrentSensor.hpp"
#include "motor2/platform/PwmDriver.hpp"
#include "motor2/SvpwmModular.hpp"
#include "motor2/AbsoluteEncoderPositionSpeedSensor.hpp"
#include "motor2/DirectSectionSensor.hpp"
#include "i2c.hpp"
#include "device/AS5600I2C.hpp"

using namespace wibot;
using namespace wibot::os;
using namespace wibot::peripheral;
using namespace wibot::accessor;
using namespace wibot::device;
using namespace wibot::motor;

extern TIM_HandleTypeDef htim1;
extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2;
extern I2C_HandleTypeDef hi2c1;

static MotorParameter mp = {
	.pole_pair = 7,
	.rs = 5.0f,
	.ld = 0.0018f,
	.lq = 0.0018f,
	.flux = 1.0f,

	.interia = 1.0f,
	.friction = 1.0f,

	// Limits
	.speed_limit = 70 * _2PI, // 4000 / 60 * 2pi
	.u_bus_max = 10.0f,
	.i_bus_limit = 1.0f,
	.i_phase_limit = 1.0f,

};
Motor mtr;

Pwm pwm(htim1);

BUFFER32_DECLARE_STATIC(u_bus_abc, 4)
BUFFER32_DECLARE_STATIC(i_bus_abc, 4)
BUFFER32_DECLARE_STATIC(pos_buf, 1)
#define ADC_READY 0x01
#define ADC_ERROR 0x02
#define I2C1_READY 0x04
#define I2C1_ERROR 0x08
EventGroup eg1("eg1");
WaitHandler wh1(eg1, ADC_READY, ADC_ERROR);
I2cMaster i2c1(hi2c1);
AS5600I2C as5600(i2c1, eg1, I2C1_READY, I2C1_ERROR);

static Shunt3PhaseCurrentSensor phaseCurrentSensor;
static SimplePowerSensor powerSensor1;
static ShuntPowerSensor powerSensor2;
static AbsoluteEncoderPositionSpeedSensor encoderPositionSpeedSensor;
static DirectSectionSensor directSectionSensor;
static DqCurrentController currentController;
static SpeedController speedController;
static PositionController positionController;

static PwmDriver driver(pwm);
static SvpwmModular svpwm;

SimplePowerSensorConfig p1_cfg{
	.u_bus = 10.0f,
};

ShuntPowerSensorConfig p2_cfg{
	.u_bus_buffer = &u_bus_abc.data[0],
	.i_bus_buffer = &i_bus_abc.data[0],
	._u_bus_value_per_unit = 3.3f / 4096.0f * 12.0f / (float)(180 + 12),
	._i_bus_value_per_unit = 0.001f,
};
Shunt3PhaseCurrentSensorConfig c1_cfg{
	.i_a_buffer = &i_bus_abc.data[1],
	.i_b_buffer = &i_bus_abc.data[2],
	.i_c_buffer = &i_bus_abc.data[3],
	.i_value_per_unit = 3.3f / 4096.0f / 0.3f / 1.56f,
};
static SvpwmModularConfig svpwm_cfg{
	.max_module_rate = _1_SQRT3,
	.max_d_module_rate = _1_SQRT3,
	.motor_parameter = &mp,
};
PwmDriverConfig pwm_cfg{
	.channel_a = PwmChannel_1P | PwmChannel_1N,
	.channel_b = PwmChannel_2P | PwmChannel_2N,
	.channel_c = PwmChannel_3P | PwmChannel_3N,
	.channel_s = PwmChannel_4,
};
AbsoluteEncoderPositionSpeedSensorConfig pos_spd_cfg{
	.encoder_buffer = &pos_buf.data[0],
	.resolution = 4096,
	.pole_pairs = 7,
	.mech_speed_cutoff_freq = 4000 / 60,
	.sample_time = 0.001f,
};
static CurrentControllerConfig curctrlcfg{
	.bandWidth = 10000 * _2PI / 20.0f,
	.sample_time = 0.0001f,
	.motor_parameter = &mp,
};
static SpeedControllerConfig spdctrlcfg{
	.bandWidth = 10000 * _2PI / 20,
	.delta = 11,
	.sample_time = 0.01f,
	.motor_parameter = &mp,
};
static PositionControllerConfig posctrlcfg{
	.bandWidth = 10000 * _2PI / 20,
	.delta = 11,
	.sample_time = 0.01f,
	.motor_parameter = &mp,
};
static void config_init()
{
	mtr.reference.sw_channel = 0x0f; // for foc
	mtr.reference.d_sample = 0.999; // for foc
	powerSensor1.config_apply(p1_cfg);
	powerSensor2.config_apply(p2_cfg);
	phaseCurrentSensor.config_apply(c1_cfg);
	svpwm.config_apply(svpwm_cfg);
	driver.config_apply(pwm_cfg);
	encoderPositionSpeedSensor.config_apply(pos_spd_cfg);
	currentController.config_apply(curctrlcfg);
	speedController.config_apply(spdctrlcfg);
	positionController.config_apply(posctrlcfg);
}
static void init_periph()
{
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
	HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
	HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
	LL_TIM_OC_SetCompareCH4(htim1.Instance, 8498);
	//LL_TIM_SetClockDivision(htim1.Instance, 1);
	pwm.config_get().channelsEnable =
		PwmChannel_1P | PwmChannel_2P | PwmChannel_3P | PwmChannel_1N | PwmChannel_2N | PwmChannel_3N;
	pwm.config_get().fullScaleDuty = 8500;
	pwm.all_enable();

	HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);
	HAL_ADCEx_Calibration_Start(&hadc2, ADC_SINGLE_ENDED);

	HAL_ADC_RegisterCallback(&hadc1, HAL_ADC_INJ_CONVERSION_COMPLETE_CB_ID, [](ADC_HandleTypeDef* hadc)
	{
	  HAL_GPIO_TogglePin(SYNC_SIG_GPIO_Port, SYNC_SIG_Pin);

	  i_bus_abc.data[1] = HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_2);
	  i_bus_abc.data[2] = HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_3);
	  i_bus_abc.data[3] = HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_4);

	  u_bus_abc.data[0] = HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_1);

	  u_bus_abc.data[1] = HAL_ADCEx_InjectedGetValue(&hadc2, ADC_INJECTED_RANK_1);
	  u_bus_abc.data[2] = HAL_ADCEx_InjectedGetValue(&hadc2, ADC_INJECTED_RANK_2);
	  u_bus_abc.data[3] = HAL_ADCEx_InjectedGetValue(&hadc2, ADC_INJECTED_RANK_3);
	  wh1.done_set(nullptr);
	});
	//HAL_ADC_Start(&hadc2);
	HAL_ADCEx_InjectedStart_IT(&hadc2);

	//HAL_ADC_Start(&hadc1);
	HAL_ADCEx_InjectedStart_IT(&hadc1);


//	HAL_ADCEx_InjectedStart(&hadc1);
//	Misc::ms_delay(10);
//	uint32_t b1 = HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_1);
//	uint32_t b2 = HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_2);
//	uint32_t b3 = HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_3);
//	uint32_t b4 = HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_4);


}
// --------------------------------
// Power Sensor

static void power_sensor_test()
{
	float i_bus;
	float u_bus;
	powerSensor1.i_bus_get(mtr, mtr.state.i_bus);
	powerSensor1.u_bus_get(mtr, mtr.state.u_bus);
	powerSensor2.i_bus_get(mtr, i_bus);
	powerSensor2.u_bus_get(mtr, u_bus);
};


// ---------------------------------------------
// Current Sensor



static void current_sensor_test()
{
	phaseCurrentSensor.zero_calibrate(mtr);
	driver.duty_set(mtr);
	Utils::delay(50);
	phaseCurrentSensor.zero_calibrate(mtr);

	mtr.reference.d_abc.v1 = 0.1;
	mtr.reference.d_abc.v2 = 0.2;
	mtr.reference.d_abc.v3 = 0.3;
	mtr.reference.d_sample = 0.999;
	driver.duty_set(mtr);

	auto scp = wh1.scope_begin();
	wh1.wait(scp, TIMEOUT_FOREVER);
	phaseCurrentSensor.i_abc_get(mtr, mtr.state.i_abc);
	wh1.scope_end();
};

// ---------------------------------------------
// PWM Driver
static void driver_test()
{

	mtr.reference.d_abc = Vector3f(0.5f, 0.5f, 0.5f);
	mtr.reference.d_sample = 0.999f;
	mtr.reference.sw_channel = 0x0f;
	driver.duty_set(mtr);
}


// ---------------------------------------------
// SVPWM Modular

static void modular_test()
{

	while (1)
	{
//		for (int i = 0; i < 1000; ++i)
//		{
//			mtr.reference.u_dq.v1 = 0.0f;
//			mtr.reference.u_dq.v2 = 0.01f * (float)i;
//			svpwm.module(mtr);
//			driver.duty_set(mtr);
//			os::Thread::sleep(1);
//
//		}
		volatile uint64_t begin;
		volatile uint64_t end;
		[[maybe_unused]] volatile uint64_t duration1 = 0;
		[[maybe_unused]] volatile uint64_t duration2 = 0;
		[[maybe_unused]] volatile uint64_t duration3 = 0;
		for (int i = 0; i < 100000000; ++i)
		{
			begin = Misc::get_tick_ns();
			mtr.reference.u_dq.v1 = 0.0f;
			mtr.reference.u_dq.v2 = 1.0f;
			mtr.state.pos_spd_e.v1 += 0.00001f * (i / 1000);
			end = Misc::get_tick_ns();
			duration1 = end - begin;
			begin = Misc::get_tick_ns();
			svpwm.module(mtr); // 3.5us
			end = Misc::get_tick_ns();
			duration2 = end - begin;
			begin = Misc::get_tick_ns();
			driver.duty_set(mtr); // 1us
			end = Misc::get_tick_ns();
			duration3 = end - begin;
			Misc::us_delay(100);

		}
	}

}

// ---------------------------------------------
// pos_spd_sensor


void pos_spd_sensor_test()
{

	mtr.state.pos_spd_m.v1 = 0.0f;
	mtr.reference.u_dq.v1 = 0.1f;
	mtr.reference.u_dq.v2 = 0.0f;
	svpwm.module(mtr);
	driver.duty_set(mtr);
	Misc::ms_delay(10);
	as5600.zero_set();

	mtr.state.pos_spd_m.v1 = 0.0f;
	mtr.reference.u_dq.v1 = 0.0f;
	mtr.reference.u_dq.v2 = 0.0f;
	svpwm.module(mtr);
	driver.duty_set(mtr);

	while (1)
	{
		as5600.angle_get(*pos_buf.data);
		encoderPositionSpeedSensor.position_speed_get(mtr, mtr.state.pos_spd_e, mtr.state.pos_spd_m);
		Misc::ms_delay(1);
	}

};

// ---------------------------------------------
// section sensor

void section_sensor_test()
{
	directSectionSensor.section_get(mtr, mtr.state.section);
}

// ---------------------------------------------
// Current Control


static void current_control_test()
{

	mtr.reference.i_dq.v1 = 0.0f;
	mtr.reference.i_dq.v2 = 0.5f;

	auto level = wh1.scope_begin();
	while (true)
	{
		powerSensor2.u_bus_get(mtr, mtr.state.u_bus);
		phaseCurrentSensor.i_abc_get(mtr, mtr.state.i_abc);
		encoderPositionSpeedSensor.position_speed_get(mtr, mtr.state.pos_spd_e, mtr.state.pos_spd_m);
		directSectionSensor.section_get(mtr, mtr.state.section);

		currentController.voltage_get(mtr, mtr.reference.u_dq);
		svpwm.module(mtr);
		driver.duty_set(mtr);
		wh1.wait(level, TIMEOUT_FOREVER);
	}
	wh1.scope_end();
}

static void speed_control_test()
{

	mtr.reference.speed = 2.0f * _2PI;

	auto level = wh1.scope_begin();
	while (true)
	{
		powerSensor2.u_bus_get(mtr, mtr.state.u_bus);
		phaseCurrentSensor.i_abc_get(mtr, mtr.state.i_abc);
		encoderPositionSpeedSensor.position_speed_get(mtr, mtr.state.pos_spd_e, mtr.state.pos_spd_m);
		directSectionSensor.section_get(mtr, mtr.state.section);

		speedController.current_get(mtr, mtr.reference.i_dq);
		currentController.voltage_get(mtr, mtr.reference.u_dq);
		svpwm.module(mtr);
		driver.duty_set(mtr);
		wh1.wait(level, TIMEOUT_FOREVER);
	}
	wh1.scope_end();
}

static void position_control_test()
{
	mtr.reference.position = _PI;

	auto level = wh1.scope_begin();
	while (true)
	{
		powerSensor2.u_bus_get(mtr, mtr.state.u_bus);
		phaseCurrentSensor.i_abc_get(mtr, mtr.state.i_abc);
		encoderPositionSpeedSensor.position_speed_get(mtr, mtr.state.pos_spd_e, mtr.state.pos_spd_m);
		directSectionSensor.section_get(mtr, mtr.state.section);

		positionController.speed_get(mtr, mtr.reference.speed);
		speedController.current_get(mtr, mtr.reference.i_dq);
		currentController.voltage_get(mtr, mtr.reference.u_dq);
		svpwm.module(mtr);
		driver.duty_set(mtr);
		wh1.wait(level, TIMEOUT_FOREVER);
	}
	wh1.scope_end();

}
// ---------------------------------------------
// FOC Control

// FocControl foc(&powerSensor, &c1, &encoderPositionSpeedSensor, ), &svpwm);
static void foc_test()
{
	//foc.calibrate();
}

void app_test()
{
	init_periph();
	config_init();
	power_sensor_test();
	current_sensor_test();
	pos_spd_sensor_test();
	section_sensor_test();
	driver_test();
	modular_test();

	current_control_test();
	speed_control_test();
	position_control_test();
	foc_test();
}

