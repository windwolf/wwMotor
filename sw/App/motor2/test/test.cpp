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
#include "tim.h"
#include "adc.h"
#include "i2c.h"
#include "utils.hpp"

using namespace wibot;
using namespace wibot::os;
using namespace wibot::peripheral;
using namespace wibot::accessor;
using namespace wibot::device;
using namespace wibot::motor;

static MotorParameter mp = {

	.pole_pair = 7,
	.rs = 3.465f,
	.ld = 0.823e-3f,
	.lq = 0.85e-3f,
	.flux = 1.4109 / 2.0 * 0.034145 / _2PI / _SQRT3, // Te=34.145ms Vlpp=1.4109.

	.interia = 4.5e-6f,
	.friction = 0.0001f,

	// Limits
	.speed_limit = 70 * _2PI, // 4000 / 60 * 2pi
	.u_bus_max = 5.0f,
	.i_bus_limit = 1.0f,
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
EventGroup eg1("eg_curr");
EventGroup eg2("eg_pos");
EventGroup eg3("eg_out");
WaitHandler wh_innerloop(eg1, ADC_READY, ADC_ERROR);
WaitHandler wh_outerloop(eg3, AS5600_READY, AS5600_ERROR);
I2cMaster i2c1(hi2c1);
AS5600I2C as5600(i2c1, eg2, I2C1_READY, I2C1_ERROR);

static Shunt3PhaseCurrentSensor phaseCurrentSensor;
static SimplePowerSensor powerSensor1;
static ShuntPowerSensor powerSensor2;
static AbsoluteEncoderPositionSpeedSensor encoderPositionSpeedSensor;
static VirtualPositionSpeedSensor virtualPositionSpeedSensor;
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
	.i_value_per_unit = 3.3f / 4096.0f / 0.33f / 1.53f,
	.skip_threshold = 0.75f,
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
	.direction = 1,
	.mech_pos_cutoff_freq = _2PI * 4000 / 60,
	.mech_speed_cutoff_freq = 4000 / 60,
	.sample_time = 0.0001f,
};
static VirtualPositionSpeedSensorConfig vpos_spd_cfg{
	.polePairs = 7,
	.sample_time = 0.0001f,
};

static CurrentControllerConfig curctrlcfg{
	.bandWidth = 10000.0f * _2PI / 100.0f,
	.sample_time = 0.0001f,
	.motor_parameter = &mp,
};

static SpeedControllerConfig spdctrlcfg{
	.bandWidth = 10000.0f * _2PI / 100.0f,
	.delta = 11,
	.sample_time = 0.001f,
	.motor_parameter = &mp,
};

static PositionControllerConfig posctrlcfg{
	.kp = 0.1f,
	.ki = 0.1f,
	.kd = 0.1f,
	.sample_time = 0.001f,
	.motor_parameter = &mp,
};

static FocControl foc(&powerSensor1,
	&phaseCurrentSensor,
	&encoderPositionSpeedSensor,
	&virtualPositionSpeedSensor,
	&directSectionSensor,
	&positionController,
	&speedController,
	&currentController,
	&svpwm,
	&driver);

uint8_t innerloopThdStack[8096];
uint8_t outerloopThdStack[8096];
TX_THREAD innerloopThd;
TX_THREAD outerloopThd;

static void config_init()
{
	mtr.reference.sw_channel = 0x0f; // for foc
	mtr.reference.d_sample = 1.0f; // for foc
	powerSensor1.config = p1_cfg;
	powerSensor2.config_apply(p2_cfg);
	phaseCurrentSensor.config_apply(c1_cfg);
	svpwm.config_apply(svpwm_cfg);
	driver.config = pwm_cfg;
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
	pwm.config.channelsEnable =
		PwmChannel_1P | PwmChannel_2P | PwmChannel_3P | PwmChannel_1N | PwmChannel_2N | PwmChannel_3N;
	pwm.config.fullScaleDuty = 8500;
	pwm.all_enable();

	HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);
	HAL_ADCEx_Calibration_Start(&hadc2, ADC_SINGLE_ENDED);

	HAL_ADC_RegisterCallback(&hadc1, HAL_ADC_INJ_CONVERSION_COMPLETE_CB_ID, [](ADC_HandleTypeDef* hadc)
	{
	  //TODO: 尝试改成TIM.UPDATE 事件触发,
	  HAL_GPIO_TogglePin(SYNC_SIG_GPIO_Port, SYNC_SIG_Pin);

	  u_bus_abc.data[0] = HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_1);
	  i_bus_abc.data[1] = HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_2);
	  i_bus_abc.data[2] = HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_3);
	  i_bus_abc.data[3] = HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_4);

	  u_bus_abc.data[1] = HAL_ADCEx_InjectedGetValue(&hadc2, ADC_INJECTED_RANK_1);
	  u_bus_abc.data[2] = HAL_ADCEx_InjectedGetValue(&hadc2, ADC_INJECTED_RANK_2);
	  u_bus_abc.data[3] = HAL_ADCEx_InjectedGetValue(&hadc2, ADC_INJECTED_RANK_3);
	  pos_buf.data[0] = HAL_ADCEx_InjectedGetValue(&hadc2, ADC_INJECTED_RANK_4);
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

static void power_sensor_test()
{
	float i_bus;
	float u_bus;
	powerSensor1.i_bus_get(mtr, mtr.state.i_bus);
	powerSensor1.u_bus_get(mtr, mtr.state.u_bus);
	powerSensor2.i_bus_get(mtr, i_bus);
	powerSensor2.u_bus_get(mtr, u_bus);
};

static void current_sensor_test()
{
	phaseCurrentSensor.zero_calibrate(mtr);
	driver.duty_set(mtr);
	Utils::delay(50);
	phaseCurrentSensor.zero_calibrate(mtr);

	mtr.reference.d_abc.v1 = 0.1;
	mtr.reference.d_abc.v2 = 0.2;
	mtr.reference.d_abc.v3 = 0.3;
	mtr.reference.d_sample = 1.0f;
	driver.duty_set(mtr);

	auto scp = wh_innerloop.scope_begin();
	while (0)
	{
		wh_innerloop.wait(scp, TIMEOUT_FOREVER);
		phaseCurrentSensor.i_ab_get(mtr, mtr.state.i_ab);
	}

	wh_innerloop.scope_end();
};

static void driver_test()
{

	mtr.reference.d_abc = Vector3f(0.1f, 0.1f, 0.1f);
	mtr.reference.d_sample = 1.0f;
	mtr.reference.sw_channel = 0x0f;
	driver.duty_set(mtr);
	mtr.reference.d_abc = Vector3f(0.0f, 0.0f, 0.0f);
}

static void modular_test()
{

	StopWatch sw;

	[[maybe_unused]] volatile uint64_t duration1 = 0;
	[[maybe_unused]] volatile uint64_t duration2 = 0;
	[[maybe_unused]] volatile uint64_t duration3 = 0;
	// mtr.state.pos_spd_e.v1 = 0.4f;
	while (0)
	{
		mtr.reference.u_dq.v1 = 1.0f;
		mtr.reference.u_dq.v2 = 0.0f;

		directSectionSensor.section_get(mtr, mtr.state.section);
		svpwm.module(mtr, mtr.reference.section,
			mtr.reference.d_abc,
			mtr.reference.u_abc,
			mtr.reference.sw_channel,
			mtr.reference.d_sample); // 3.5us
		driver.duty_set(mtr);
		Misc::ms_delay(1);
		mtr.state.pos_spd_e.v1 += _PI_3;
	}
	mtr.reference.u_dq.v1 = 0.0f;
	mtr.reference.u_dq.v2 = 5.0f;
	while (1)
	{

		encoderPositionSpeedSensor.position_speed_get(mtr, mtr.state.pos_spd_e, mtr.state.pos_spd_m);
		directSectionSensor.section_get(mtr, mtr.state.section);
		svpwm.module(mtr, mtr.reference.section,
			mtr.reference.d_abc,
			mtr.reference.u_abc,
			mtr.reference.sw_channel,
			mtr.reference.d_sample); // 3.5us
		driver.duty_set(mtr);
		Misc::ms_delay(1);
		//mtr.state.pos_spd_e.v1 += _PI_3;
	}
}

void pos_spd_sensor_test()
{
	mtr.reference.u_dq.v1 = 1.0f;
	mtr.reference.u_dq.v2 = 0.0f;
	mtr.state.pos_spd_m.v1 = 0.0f;
	directSectionSensor.section_get(mtr, mtr.state.section);
	svpwm.module(mtr, mtr.reference.section,
		mtr.reference.d_abc,
		mtr.reference.u_abc,
		mtr.reference.sw_channel,
		mtr.reference.d_sample);
	driver.duty_set(mtr);
	Misc::ms_delay(10);
	as5600.zero_set();
	Misc::ms_delay(10);
	// as5600.angle_get(*pos_buf.data);
	mtr.state.pos_spd_m.v1 = 0.0f;
	mtr.reference.u_dq.v1 = 0.0f;
	mtr.reference.u_dq.v2 = 0.0f;
	svpwm.module(mtr, mtr.reference.section,
		mtr.reference.d_abc,
		mtr.reference.u_abc,
		mtr.reference.sw_channel,
		mtr.reference.d_sample);
	driver.duty_set(mtr);

	while (0)
	{
		// as5600.angle_get(*pos_buf.data);
		encoderPositionSpeedSensor.position_speed_get(mtr, mtr.state.pos_spd_e, mtr.state.pos_spd_m);
		Misc::ms_delay(1);
	}

};

void section_sensor_test()
{
	mtr.reference.u_dq.v1 = 1.0f;
	mtr.reference.u_dq.v2 = 0.0f;
	mtr.state.pos_spd_e.v1 = 0.0f;
	directSectionSensor.section_get(mtr, mtr.state.section);
	svpwm.module(mtr, mtr.reference.section,
		mtr.reference.d_abc,
		mtr.reference.u_abc,
		mtr.reference.sw_channel,
		mtr.reference.d_sample);
	driver.duty_set(mtr);
	Misc::ms_delay(10);
	// as5600.angle_get(*pos_buf.data);
	encoderPositionSpeedSensor.position_speed_get(mtr, mtr.state.pos_spd_e, mtr.state.pos_spd_m);
	directSectionSensor.section_get(mtr, mtr.state.section);
	Misc::ms_delay(10);
	mtr.reference.u_dq.v1 = 0.0f;
	mtr.reference.u_dq.v2 = 0.0f;
	svpwm.module(mtr, mtr.reference.section,
		mtr.reference.d_abc,
		mtr.reference.u_abc,
		mtr.reference.sw_channel,
		mtr.reference.d_sample);
	driver.duty_set(mtr);
	Misc::ms_delay(10);
}
using namespace wibot::control;
float pid_test_out = 0.0f;
static void pid_test()
{

	PidController pid;
	pid.config.Kp = 0.517106150780880;
	pid.config.Ki = 4210.20654;
	pid.config.Kd = 0.0f;
	pid.config.sample_time = 0.0001f;
	pid.config.mode = PidControllerMode::Serial;
	pid.config.integrator_limit_enable = pid.config.output_limit_enable = true;
	pid.config.integrator_limit_min = pid.config.output_limit_min = -10.0f;
	pid.config.integrator_limit_max = pid.config.output_limit_max = 10.0f;
	Misc::ms_delay(1000);
	while (1)
	{
		pid_test_out = pid.update(0.3, pid_test_out);
		Misc::ms_delay(1);
	}
};

static void current_control_test()
{

	mtr.reference.i_dq.v1 = 0.0f;
	mtr.reference.i_dq.v2 = 0.1f;

	// as5600.angle_get(*pos_buf.data);
	// encoderPositionSpeedSensor.position_speed_get(mtr, mtr.state.pos_spd_e, mtr.state.pos_spd_m);
	// virtualPositionSpeedSensor.position_set(0.0f);
	// virtualPositionSpeedSensor.position_speed_get(mtr, mtr.state.pos_spd_e, mtr.state.pos_spd_m);

	auto level = wh_innerloop.scope_begin();
	while (true)
	{
		powerSensor2.u_bus_get(mtr, mtr.state.u_bus);

		encoderPositionSpeedSensor.position_speed_get(mtr, mtr.state.pos_spd_e, mtr.state.pos_spd_m);
		directSectionSensor.section_get(mtr, mtr.state.section);
		phaseCurrentSensor.i_ab_get(mtr, mtr.state.i_ab);
		//FocMath::abc2ab(mtr.state.i_abc, mtr.state.i_ab);
		FocMath::ab2dq(mtr.state.i_ab, mtr.state.pos_spd_e.v1, mtr.state.i_dq);

		currentController.voltage_get(mtr, mtr.reference.u_dq);

		svpwm.module(mtr, mtr.reference.section,
			mtr.reference.d_abc,
			mtr.reference.u_abc,
			mtr.reference.sw_channel,
			mtr.reference.d_sample);
		driver.duty_set(mtr);
		wh_innerloop.wait(level, TIMEOUT_FOREVER);
	}
	wh_innerloop.scope_end();
}

static void speed_control_test()
{

	mtr.reference.speed = 2.0f * _2PI;

	auto level = wh_innerloop.scope_begin();
	while (true)
	{
		powerSensor2.u_bus_get(mtr, mtr.state.u_bus);
		encoderPositionSpeedSensor.position_speed_get(mtr, mtr.state.pos_spd_e, mtr.state.pos_spd_m);
		directSectionSensor.section_get(mtr, mtr.state.section);
		phaseCurrentSensor.i_ab_get(mtr, mtr.state.i_ab);
		FocMath::ab2dq(mtr.state.i_ab, mtr.state.pos_spd_e.v1, mtr.state.i_dq);

		speedController.current_get(mtr, mtr.reference.i_dq);
		currentController.voltage_get(mtr, mtr.reference.u_dq);
		svpwm.module(mtr, mtr.reference.section,
			mtr.reference.d_abc,
			mtr.reference.u_abc,
			mtr.reference.sw_channel,
			mtr.reference.d_sample);
		driver.duty_set(mtr);
		wh_innerloop.wait(level, TIMEOUT_FOREVER);
	}
	wh_innerloop.scope_end();
}

static void position_control_test()
{
	mtr.reference.position = _PI;

	auto level = wh_innerloop.scope_begin();
	while (true)
	{
		powerSensor2.u_bus_get(mtr, mtr.state.u_bus);
		encoderPositionSpeedSensor.position_speed_get(mtr, mtr.state.pos_spd_e, mtr.state.pos_spd_m);
		directSectionSensor.section_get(mtr, mtr.state.section);
		phaseCurrentSensor.i_ab_get(mtr, mtr.state.i_ab);
		FocMath::ab2dq(mtr.state.i_ab, mtr.state.pos_spd_e.v1, mtr.state.i_dq);

		positionController.speed_get(mtr, mtr.reference.speed);
		speedController.current_get(mtr, mtr.reference.i_dq);
		currentController.voltage_get(mtr, mtr.reference.u_dq);
		svpwm.module(mtr, mtr.reference.section,
			mtr.reference.d_abc,
			mtr.reference.u_abc,
			mtr.reference.sw_channel,
			mtr.reference.d_sample);
		driver.duty_set(mtr);
		wh_innerloop.wait(level, TIMEOUT_FOREVER);
	}
	wh_innerloop.scope_end();

}

using namespace wibot;

void innerloop_task(uint32_t arg)
{
	auto scope = wh_innerloop.scope_begin();
	wh_innerloop.done_set(nullptr);
	StopWatch sw;
	auto duration = 0;

	while (true)
	{
		wh_innerloop.wait(scope, TIMEOUT_FOREVER);
		sw.start();
		foc.innerLoop(mtr);
		duration = sw.tick();

	}
	wh_innerloop.scope_end();
}

void outerloop_task(uint32_t arg)
{
	auto scope = wh_outerloop.scope_begin();
	wh_outerloop.done_set(nullptr);
	StopWatch sw;
	volatile auto duration = 0;
	while (true)
	{
		wh_outerloop.wait(scope, TIMEOUT_FOREVER);
		sw.start();
		// as5600.angle_get(pos_buf.data[0]);
		//duration = sw.tick();
		foc.outerLoop(mtr);
		duration = sw.tick();

	}
	wh_outerloop.scope_end();
}

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

//	foc.calibrate(mtr);
//	os::Utils::delay(100);

//	cmd.mode = FocCommandMode::Calibrate;
//	mtr.reference.u_dq.v1 = mtr.state.u_bus * 0.2;
//	mtr.reference.u_dq.v1 = 0.0f;

	while (1)
	{
		cmd.mode = FocCommandMode::OpenLoop;
		cmd.voltage.v1 = 0.0f;
		cmd.voltage.v2 = mtr.state.u_bus * 0.1;
		// cmd.speed = 20.0f;
		foc.command_set(mtr, cmd);
		os::Utils::delay(2000);
	}

	while (0)
	{
		cmd.mode = FocCommandMode::OpenLoop;
		cmd.voltage.v1 = mtr.state.u_bus * 0.1;
		cmd.voltage.v2 = 0.0f;
		// cmd.speed = 20.0f;
		foc.command_set(mtr, cmd);
		os::Utils::delay(2000);
	}
	while (0)
	{
		cmd.mode = FocCommandMode::Current;
		cmd.current.v1 = 0.0f;
		cmd.current.v2 = 0.200f;
		foc.command_set(mtr, cmd);
		os::Utils::delay(2000);
	}
	while (0)
	{
		cmd.mode = FocCommandMode::Speed;
		cmd.speed = -_2PI * 1;
		foc.command_set(mtr, cmd);
		os::Utils::delay(2000);
	}
	cmd.mode = FocCommandMode::Position;
	cmd.position = _PI;
	foc.command_set(mtr, cmd);
	os::Utils::delay(2000);
}

void app_test()
{
	// pid_test();
	init_periph();
	config_init();
	power_sensor_test();
	current_sensor_test();
	pos_spd_sensor_test();
	//section_sensor_test();
	//driver_test();
	//modular_test();

	//current_control_test();
	//speed_control_test();
	//position_control_test();
	foc_test();
}

