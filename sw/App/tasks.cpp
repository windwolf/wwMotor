#include "tasks.hpp"

#include "main.h"
#include "stm32g4xx_hal.h"
#include "stm32g4xx_hal_tim.h"
#include "stm32g4xx_hal_tim_ex.h"
#include "stm32g4xx_ll_tim.h"
#include "stm32g4xx_hal_adc.h"
#include "stm32g4xx_hal_adc_ex.h"
#include "stm32g4xx_ll_adc.h"
#include "pwm.hpp"
#include "motor/base.hpp"
#include "motor/component/driver.hpp"
#include "motor/platform/pwm_driver_executor.hpp"
#include "motor/component/foc_math.hpp"
#include "motor/ctrl/foc_control.hpp"

using namespace ww;
using namespace ww::peripheral;
using namespace wwMotor;

extern TIM_HandleTypeDef htim1;
extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2;

Pwm pwm(htim1);

BUFFER16_DECLARE_STATIC(adc1_buf, 4)
// BUFFER16_DECLARE_STATIC(adc2_buf, 4)
BUFFER16_DECLARE_STATIC(pos_buf, 4)

static void pwm_test()
{
    pwm.config_get().channelsEnable =
        TimerChannel_1 | TimerChannel_2 | TimerChannel_3 | TimerChannel_4;
    pwm.init();
    pwm.start();
    pwm.duty_set(TimerChannel_1, 1000);
};

static void driver_test()
{

    SimplePower power({
        .ubus = 10.0f,
    });

    float theta = 0.0f;

    PwmDriverExecutor::Config eCfg{
        .channel_a = TimerChannel_1,
        .channel_b = TimerChannel_2,
        .channel_c = TimerChannel_3,
    };
    PwmDriverExecutor executor(eCfg, pwm);

    DriverSVPWM::Config cfg{
        .maxModuleRate = 0.8,
        .maxDModuleRate = 0.8,
        .enableDeadTime = false,
        .deadTime = 0,
    };

    DriverSVPWM driver(cfg, executor);
    Vector2f dq = {.v1 = 10.f, .v2 = 0.0f};

    while (true)
    {
        Vector2f ab;
        FocMath::dq2ab(dq, theta, ab);
        driver.phase_voltage_set(ab, power.ubus_get());
        theta += 0.01f;
        ww::os::Utils::delay(1);
    }
};
static void encoder_test(){

};

static void curr_sensor_test(){

};

static Motor motor = {
    .polePair = 7,
    .rs = 5.0f,
    .ld = 0.0018f,
    .lq = 0.0018f,
    .flux = 1.0f,

    .interia = 1.0f,
    .friction = 1.0f,

    // Limits
    .speed_limit = 50 * _2PI,
    .u_bus_max = 12.0f,
    .i_bus_limit = 1.0f,
    .i_phase_limit = 1.0f,
};

static FocControl::Config cfg{
    .pos_spd_sensor_cfg =
        {
            .encoder_buffer = pos_buf,
            .position_idx = 0,
            .sample_time = 0.0001,
            .full_scalar = 1000,
            .pole_pairs = 7,
        },
    .curr_sensor_cfg =
        {
            .maxScaleValue = 4096,
            .refVoltage = 3.3f,
            .shuntResistance = 0.05f,
            .ampGain = 100.0f,
            .rawDataBuffer = adc1_buf,
            .a_phase_idx = 0,
            .b_phase_idx = 1,
            .c_phase_idx = 2,
        },
    .curr_ctrl_cfg =
        {
            .bandWidth = 10000.0f * _2PI / 20, // Typically: Fs*2PI/20
            .enableFeedforward = true,
            .sample_time = 0.0001,
        },
    .spd_ctrl_cfg =
        {
            .bandWidth = 10000.0f * _2PI / 20.0f, // Typically: Fs*2PI/20. the same as current loop
            .delta = 8, // The distance between system zero and speed loop's pole in log scale.
            .sample_time = 0.0001,
        },
    .pos_ctrl_cfg =
        {
            .bandWidth = 1,
            .delta = 1,
            .sample_time = 0.0001,
        },
    .drv_cfg =
        {
            .maxModuleRate = 0.9,
            .maxDModuleRate = 0.85,
            .enableDeadTime = false,
            .deadTime = 0.0f,
        },
    .power_cfg =
        {
            .ubus = 12.0f,
        },
};

static void foc_test()
{
    PwmDriverExecutor::Config eCfg{
        .channel_a = TimerChannel_1,
        .channel_b = TimerChannel_2,
        .channel_c = TimerChannel_3,
    };
    PwmDriverExecutor executor(eCfg, pwm);
    FocControl foc(cfg, motor, executor);
};

void app_test(void *argument)
{
    pwm_test();
    driver_test();
    encoder_test();
    curr_sensor_test();

    foc_test();
};