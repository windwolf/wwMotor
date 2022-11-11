#include "foc_control.hpp"
#include "../component/foc_math.hpp"
#include "base/base.hpp"
#include "os/os.hpp"
namespace wwMotor
{

void FocControl::init()
{
    context.u_bus = power.ubus_get();
    position_speed_sensor.init();
    driver.phase_voltage_set(Vector2f(), context.u_bus);
};
void FocControl::calibrate()
{
    Vector2f u_ab;
    context.u_bus = power.ubus_get();

    u_ab.v1 = context.u_bus * 0.1f;
    u_ab.v2 = 0.0f;
    position_speed_sensor.calibrate_begin();
    driver.phase_voltage_set(u_ab, context.u_bus);
    for (size_t i = 0; i < 10; i++)
    {
        position_speed_sensor.calibrate();
        ww::os::Utils::delay(10);
    }
    position_speed_sensor.calibrate_end();
    u_ab.v1 = 0.0f;
    driver.phase_voltage_set(u_ab, context.u_bus);
};

void FocControl::update()
{
    context.u_bus = power.ubus_get();

    if (context.mode != Mode::OpenLoop)
    {
        context.pos_spd = position_speed_sensor.speed_position_get();
        context.section = FocMath::section_get(context.pos_spd.v2);
        context.i_abc = current_sensor.phase_current_get(context.section);
        FocMath::abc2ab(context.i_abc, context.i_ab);
        FocMath::ab2dq(context.i_ab, context.pos_spd.v2, context.i_dq);
    }

    switch (context.mode)
    {
    case Mode::Position:
        context.speed_ref = position_controller.update(context.position_ref, 1, context.pos_spd.v1);
    case Mode::Speed:
        context.i_dq_ref = speed_controller.update(context.speed_ref, context.pos_spd.v3);
    case Mode::Current:
        context.u_dq_ref =
            current_controller.update(context.i_dq_ref, context.i_dq, context.pos_spd.v4);
        break;
    case Mode::OpenLoop:
        context.position_ref = context.speed_ref * config.spd_ctrl_cfg.sample_time;
        context.u_dq_ref.v1 = 0;
        context.u_dq_ref.v2 = context.u_bus;
        context.pos_spd.v3 = context.speed_ref;
        context.pos_spd.v4 = context.speed_ref * motor.polePair;
        context.pos_spd.v1 = Math::circle_normalize(
            context.pos_spd.v1 + context.speed_ref * config.spd_ctrl_cfg.sample_time);
        context.pos_spd.v2 = Math::circle_normalize(context.pos_spd.v1 * motor.polePair);

        break;
    default:
        break;
    }

    FocMath::dq2ab(context.u_dq_ref, context.pos_spd.v2, context.u_ab_ref);
    context.u_ab_ref = driver.circle_limit(context.u_ab_ref, context.u_bus);
    driver.phase_voltage_set(context.u_ab_ref, context.u_bus);
};

} // namespace wwMotor
