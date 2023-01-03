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
#include "motor2/impl/SamplingPowerSensor.hpp"
#include "motor2/impl/SimplePowerSensor.hpp"
#include "motor2/PhaseCurrentSensor.hpp"
#include "motor2/impl/Shunt3PhaseCurrentSensor.hpp"
#include "motor2/platform/PwmDriver.hpp"
#include "motor2/impl/SvpwmModular.hpp"
#include "motor2/impl/AbsoluteEncoderPositionSpeedSensor.hpp"
#include "motor2/impl/DirectSectionSensor.hpp"
#include "i2c.hpp"
#include "device/AS5600I2C.hpp"
#include "tim.h"
#include "adc.h"
#include "i2c.h"
#include "spi.h"
#include "utils.hpp"
#include "math_shared.hpp"
#include "MT6816SPI.hpp"

using namespace wibot;
using namespace wibot::os;
using namespace wibot::peripheral;
using namespace wibot::accessor;
using namespace wibot::device;
using namespace wibot::motor;

static MotorParameter mp = {

	.pole_pair = 7,
	.rs = 3.465f,
	.ld = 1.568e-3 / 2,
	.lq = 1.64e-3 / 2,
	.flux = 1.4109 / 2.0 * 0.034145 / _2PI / _SQRT3, // Te=34.145ms Vlpp=1.4109.

	.interia = 4.5e-6f,
	.friction = 0.0001f,

	// Limits
	.speed_limit = 5000,
	.u_bus_max = 5.0f,
	.i_bus_limit = 0.2f,
	.i_phase_limit = 0.5f,
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
#define AS5600_READY 0x10
#define AS5600_ERROR 0x20
#define MT6816_READY 0x40
#define MT6816_ERROR 0x80
EventGroup eg1("eg_curr");
EventGroup eg2("eg_pos");
EventGroup eg3("eg_out");
EventGroup eg4("eg_mt6816");
WaitHandler wh_innerloop(eg1, ADC_READY, ADC_ERROR);
WaitHandler wh_outerloop(eg3, AS5600_READY, AS5600_ERROR);
I2cMaster i2c1(hi2c1);
Spi spi(hspi3);
AS5600I2C as5600(i2c1, eg2, I2C1_READY, I2C1_ERROR);
MT6816SPI mt6816(spi, eg4, MT6816_READY, AS5600_ERROR);
MemoryDataSource i_a(&i_bus_abc.data[1]);
MemoryDataSource i_b(&i_bus_abc.data[2]);
MemoryDataSource i_c(&i_bus_abc.data[3]);
MemoryDataSource u_bus(&u_bus_abc.data[0]);
static PwmDriver driver(pwm);


static FocControlConfig cfg = {
	.power_sensor
	{
		.u_bus = &u_bus,
		.u_bus_pu = 3.3f / 4096.0f * 12.0f / (float)(180 + 12),
	},
	.current_sensor
	{
		.i_a = &i_a,
		.i_b = &i_b,
		.i_c = &i_c,
		.i_pu = 3.3f / 4096.0f / 0.33f / 1.53f,
		.low_duty_skip_threshold = 0.75f,
	},
	.modular
	{
		. allow_over_module = false,
	},
	.encoder
	{
        .codex = &mt6816,
		.resolution = 16386,
		.direction = EncoderDirection::Forward,
		.calibration_voltage = 1.0f,
	},
	.current_controller
	{
		.bw = 10000.0f * _2PI / 100.0f,
		.disableFeedforward= false,
	},
	.speed_controller
	{
		.delta = 11,
	},
	.position_controller
	{
		.kp = 0.1f,
		.ki = 0.1f,
		.kd = 0.0f,
	},
	.motor_parameter = &mp,
	.high_frequency_samlpe_time = 0.0001f,
	.low_frequency_samlpe_time = 0.001f,

};

PwmDriverConfig pwm_cfg{
	.channel_a = PwmChannel_1P | PwmChannel_1N,
	.channel_b = PwmChannel_2P | PwmChannel_2N,
	.channel_c = PwmChannel_3P | PwmChannel_3N,
	.channel_s = PwmChannel_4,
	.fullScaleDuty = 8500,
};

static FocControl foc(&driver);

uint8_t innerloopThdStack[5000];
uint8_t outerloopThdStack[5000];
TX_THREAD innerloopThd;
TX_THREAD outerloopThd;

static void config_init()
{
	driver.config = pwm_cfg;
	driver.apply_config();
	foc.config = cfg;
	foc.apply_config();
	mtr.reference.sw_channel = 0x0f; // for foc
	mtr.reference.d_sample = 1.0f; // for foc
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
//	PwmConfig pCfg;
//	pCfg.fullScaleDuty = 8500;
//	pCfg.channelsEnable = PwmChannel_1P | PwmChannel_2P | PwmChannel_3P | PwmChannel_1N | PwmChannel_2N | PwmChannel_3N;
//	pwm.channel_enable(PwmChannel_1P | PwmChannel_2P | PwmChannel_3P | PwmChannel_1N | PwmChannel_2N | PwmChannel_3N);
//	pwm.config.fullScaleDuty = 8500;
	pwm.all_enable();

	HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);
	HAL_ADCEx_Calibration_Start(&hadc2, ADC_SINGLE_ENDED);

	HAL_ADC_RegisterCallback(&hadc1, HAL_ADC_INJ_CONVERSION_COMPLETE_CB_ID, [](ADC_HandleTypeDef* hadc)
	{
	  HAL_GPIO_TogglePin(SYNC_SIG_GPIO_Port, SYNC_SIG_Pin);
	  u_bus_abc.data[0] = LL_ADC_INJ_ReadConversionData32(hadc1.Instance, LL_ADC_INJ_RANK_1);
	  i_bus_abc.data[1] = LL_ADC_INJ_ReadConversionData32(hadc1.Instance, LL_ADC_INJ_RANK_2);
	  i_bus_abc.data[2] = LL_ADC_INJ_ReadConversionData32(hadc1.Instance, LL_ADC_INJ_RANK_3);
	  i_bus_abc.data[3] = LL_ADC_INJ_ReadConversionData32(hadc1.Instance, LL_ADC_INJ_RANK_4);
	  u_bus_abc.data[1] = LL_ADC_INJ_ReadConversionData32(hadc2.Instance, LL_ADC_INJ_RANK_1);
	  u_bus_abc.data[2] = LL_ADC_INJ_ReadConversionData32(hadc2.Instance, LL_ADC_INJ_RANK_2);
	  u_bus_abc.data[3] = LL_ADC_INJ_ReadConversionData32(hadc2.Instance, LL_ADC_INJ_RANK_3);
	  // pos_buf.data[0] = LL_ADC_INJ_ReadConversionData32(hadc2.Instance, LL_ADC_INJ_RANK_4);
//	  u_bus_abc.data[0] = HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_1);
//	  i_bus_abc.data[1] = HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_2);
//	  i_bus_abc.data[2] = HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_3);
//	  i_bus_abc.data[3] = HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_4);
//
//	  u_bus_abc.data[1] = HAL_ADCEx_InjectedGetValue(&hadc2, ADC_INJECTED_RANK_1);
//	  u_bus_abc.data[2] = HAL_ADCEx_InjectedGetValue(&hadc2, ADC_INJECTED_RANK_2);
//	  u_bus_abc.data[3] = HAL_ADCEx_InjectedGetValue(&hadc2, ADC_INJECTED_RANK_3);
//	  pos_buf.data[0] = HAL_ADCEx_InjectedGetValue(&hadc2, ADC_INJECTED_RANK_4);
	  wh_innerloop.done_set(nullptr);
	});
	// HAL_ADC_Start(&hadc2);
	HAL_ADCEx_InjectedStart_IT(&hadc2);

	// HAL_ADC_Start(&hadc1);
	HAL_ADCEx_InjectedStart_IT(&hadc1);

	HAL_TIM_RegisterCallback(&htim16, HAL_TIM_PERIOD_ELAPSED_CB_ID, [](TIM_HandleTypeDef* htim)
	{
	  wh_outerloop.done_set(nullptr);
	});

	HAL_TIM_Base_Start_IT(&htim16);

//	uint8_t data[2] = { 0, 0 };
//	HAL_I2C_Mem_Read_IT(&hi2c1, 0x36 << 1, 0x0e, 1, data, 2);
//	HAL_Delay(1000);
//	HAL_I2C_Mem_Read_IT(&hi2c1, 0x36 << 1, 0x0e, 1, data, 2);
//	HAL_Delay(1000);
	as5600.init();
}

using namespace wibot;

void innerloop_task(uint32_t arg)
{
	auto scope = wh_innerloop.scope_begin();
	wh_innerloop.done_set(nullptr);
	StopWatch sw;
	[[maybe_unused]] volatile auto duration = 0;

	while (true)
	{
		wh_innerloop.wait(scope, TIMEOUT_FOREVER);
		sw.start();
		foc.hf_loop(mtr);
		duration = sw.tick();

	}
	wh_innerloop.scope_end();
}

void outerloop_task(uint32_t arg)
{
	auto scope = wh_outerloop.scope_begin();
	wh_outerloop.done_set(nullptr);
	StopWatch sw;
	[[maybe_unused]] volatile auto duration = 0;
	while (true)
	{
		wh_outerloop.wait(scope, TIMEOUT_FOREVER);
		sw.start();
		foc.command_loop(mtr);
        pos_buf.data[0] = as5600.angle_get();
		//duration = sw.tick();
		foc.lf_loop(mtr);
		duration = sw.tick();

	}
	wh_outerloop.scope_end();
}
float ref_angle;
float encoder_angle;
float encoder_nofilter_angle;
float diff_angle;
static void foc_test()
{
	FocCommand cmd;

	tx_thread_create(&innerloopThd,
		(char*)"innerloop",
		innerloop_task,
		0,
		innerloopThdStack,
		sizeof(innerloopThdStack),
		2,
		1,
		0,
		TX_AUTO_START);

	tx_thread_create(&outerloopThd,
		(char*)"outerloop",
		outerloop_task,
		0,
		outerloopThdStack,
		sizeof(outerloopThdStack),
		1,
		1,
		0,
		TX_AUTO_START);

	os::Utils::delay(100);

	foc.calibrate(mtr);

	[[maybe_unused]] auto cfg = as5600.get_config();
	[[maybe_unused]] volatile static uint8_t sta;
	sta = as5600.get_status();
	[[maybe_unused]] auto zpos = as5600.get_zpos();
	[[maybe_unused]] auto mpos = as5600.get_mpos();

	cmd.mode = wibot::motor::MotorRunMode::OpenLoop;
	cmd.voltage.v1 = 0.0f;
	cmd.voltage.v2 = 0.0f;
	foc.set_command(mtr, cmd);
	os::Utils::delay(1000);

	// test stay in open loop
	while (0)
	{
		cmd.mode = MotorRunMode::OpenLoop;
		cmd.voltage.v1 = mtr.state.u_bus * 0.1;
		cmd.voltage.v2 = 0.0f;
		// cmd.speed = 20.0f;
		foc.set_command(mtr, cmd);
		os::Utils::delay(2000);
	}

	// Test rotation in open loop
	while (0)
	{
		cmd.mode = MotorRunMode::OpenLoop;
		cmd.voltage.v1 = 0.0f;
		cmd.voltage.v2 = mtr.state.u_bus * 0.1;
		// cmd.speed = 20.0f;
		foc.set_command(mtr, cmd);
		os::Utils::delay(2000);
	}

	// Test stay in current close loop
	while (1)
	{
		cmd.mode = MotorRunMode::Current;
		cmd.current.v1 = 0.1f;
		cmd.current.v2 = 0.0f;
		foc.set_command(mtr, cmd);
		os::Utils::delay(2000);
	}

	// Test rotate in current close loo[
	while (1)
	{
		cmd.mode = MotorRunMode::Current;
		cmd.current.v1 = 0.0f;
		cmd.current.v2 = 0.1f;
		foc.set_command(mtr, cmd);
		os::Utils::delay(2000);
	}
	while (0)
	{
		cmd.mode = MotorRunMode::Speed;
		cmd.speed = _2PI * 1;
		foc.set_command(mtr, cmd);
		os::Utils::delay(2000);
	}
	cmd.mode = MotorRunMode::Position;
	cmd.position = _PI;
	foc.set_command(mtr, cmd);
	os::Utils::delay(2000);
}

void app_test()
{
	config_init();
	init_periph();

//	power_sensor_test();
//	current_sensor_test();
//	pos_spd_sensor_test();
	//section_sensor_test();
	//driver_test();
	// modular_test();

	foc_test();
}

