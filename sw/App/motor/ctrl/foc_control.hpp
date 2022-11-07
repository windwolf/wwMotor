#ifndef __WWMOTOR_FOC_FOC_CONTROL_HPP__
#define __WWMOTOR_FOC_FOC_CONTROL_HPP__

#include "../base.hpp"
#include "../component/driver.hpp"
#include "../component/power.hpp"
#include "../component/driver.hpp"
#include "../component/current_controller.hpp"
#include "../component/speed_controller.hpp"
#include "../component/position_controller.hpp"
#include "../component/position_speed_sensor.hpp"
#include "../component/current_sensor.hpp"

namespace wwMotor
{
class FocControl
{
  public:
    enum class Mode
    {
        Current,
        Speed,
        Position,
        OpenLoop,
    };

    struct Config
    {

        EncoderPositionSpeedSensor::Config pos_spd_sensor_cfg;
        CurrentSensor3Shunt::Config curr_sensor_cfg;
        CurrentController::SampleConfig curr_ctrl_cfg;
        SpeedController::Config spd_ctrl_cfg;
        PositionController::Config pos_ctrl_cfg;
        DriverSVPWM::Config drv_cfg;
        SimplePower::Config power_cfg;
    };

    struct Context
    {
        float u_bus;

        Vector4f pos_spd;

        uint8_t section;
        Vector3f i_abc;
        Vector2f i_ab;
        Vector2f i_dq;

        Mode mode;
        Scalar speed;
        Scalar position;
        Scalar current;

        Vector2f i_dq_ref;
        Scalar speed_ref;
        Scalar position_ref;

        Vector2f u_dq_ref;
        Vector2f u_ab_ref;
    };

  public:
    FocControl(Config &config, Motor &motor, DriverExecutor &driverExecutor)
        : config(config), motor(motor), power(config.power_cfg),
          position_speed_sensor(config.pos_spd_sensor_cfg),
          current_sensor(config.curr_sensor_cfg, motor),
          position_controller(config.pos_ctrl_cfg, motor),
          speed_controller(config.spd_ctrl_cfg, motor),
          current_controller(config.curr_ctrl_cfg, motor), driverExecutor(driverExecutor),
          driver(config.drv_cfg, driverExecutor){};

    void init();
    void calibrate();
    void update();

  protected:
    Config config;
    Motor &motor;
    SimplePower power;
    EncoderPositionSpeedSensor position_speed_sensor;
    CurrentSensor3Shunt current_sensor;
    PositionController position_controller;
    SpeedController speed_controller;
    CurrentController current_controller;

    DriverExecutor &driverExecutor;
    DriverSVPWM driver;
    Context context;
};
} // namespace wwMotor

#endif // __WWMOTOR_FOC_FOC_CONTROL_HPP__