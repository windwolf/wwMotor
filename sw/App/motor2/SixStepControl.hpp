//
// Created by zhouj on 2022/11/28.
//

#ifndef WWMOTOR_APP_MOTOR2_SIXSTEPCONTROLLER_HPP_
#define WWMOTOR_APP_MOTOR2_SIXSTEPCONTROLLER_HPP_

#include "SectionSensor.hpp"
#include "SectionSwitcher.hpp"
#include "PowerSensor.hpp"
#include "motor2/impl/FocPidSpeedController.hpp"
#include "motor2/impl/PidWithFeedforwardCurrentController.hpp"
#include "Modular.hpp"
#include "Driver.hpp"
#include "PhaseVoltageSensor.hpp"
#include "PhaseCurrentSensor.hpp"
#include "motor2/impl/PidBusCurrentController.hpp"
#include "BusCurrentReferenceUpdater.hpp"
#include "PositionSpeedSensor.hpp"
#include "motor2/impl/SamplingPowerSensor.hpp"
#include "motor2/impl/SamplingPhaseVoltageSensor.hpp"
#include "motor2/impl/Shunt3PhaseCurrentSensor.hpp"
#include "motor2/impl/EmfZeroCrossSectionSensor.hpp"
#include "motor2/impl/DirectPidSpeedController.hpp"
#include "motor2/impl/PassThroughBusCurrentController.hpp"
#include "motor2/impl/SixPwmModular.hpp"
namespace wibot::motor {

struct SixStepCommand {
    MotorRunMode mode;
    union {
        float speed;
        float current;
        float duty;
    };
};

class SixStepControlBase {
   public:
    SixStepControlBase(PowerSensor* power_sensor, PhaseVoltageSensor* phase_voltage_sensor,
                       SectionSensor* section_sensor, SectionSwitcher* section_switcher,
                       PositionSpeedSensor*        position_speed_sensor,
                       BusCurrentReferenceUpdater* bus_current_reference_updater,
                       BusDutyReferenceUpdater* bus_duty_reference_updater, Modular* modular,
                       Driver* driver)
        : _power_sensor(power_sensor), _phase_voltage_sensor(phase_voltage_sensor),
          _section_sensor(section_sensor), _section_switcher(section_switcher),
          _position_speed_sensor(position_speed_sensor),
          _bus_current_reference_updater(bus_current_reference_updater),
          _bus_duty_reference_updater(bus_duty_reference_updater), _modular(modular),
          _driver(driver){};
    void hf_loop(Motor& motor);

   private:
    PowerSensor*                _power_sensor;
    PhaseVoltageSensor*         _phase_voltage_sensor;
    SectionSensor*              _section_sensor;
    SectionSwitcher*            _section_switcher;
    PositionSpeedSensor*        _position_speed_sensor;
    BusCurrentReferenceUpdater* _bus_current_reference_updater;
    BusDutyReferenceUpdater*    _bus_duty_reference_updater;
    Modular*                    _modular;
    Driver*                     _driver;

    SixStepCommand _cmd;
};

struct ClassicSixStepControlConfig {
    struct {
        DataSource* u_bus;
        DataSource* i_bus;
        float       u_bus_pu;
        float       i_bus_pu;
    } power_sensor;

    struct {
        DataSource* u_a;
        DataSource* u_b;
        DataSource* u_c;
        float       u_pu;
    } phase_voltage_sensor;

    struct {
        uint32_t blank_count;  // 忽略换向初期若干各采样点. 最大值: 12/最大电角速度/sample_time.
        /**
         * 换相角相位补偿系统. 单位: s/s.
         * 换相延迟: 30°-电角速度*补偿系数*采样间隔.
         * 硬件滤波加数字滤波的相位延迟:补偿系数*采样间隔
         * 其代表了相位延迟时间换算成采样点数.
         * 可根据滤波器的相位延迟时间/采样时间得到.
         */
        uint32_t switch_delay_count;
    } zero_cross_detector;

    struct {
        float kp;
        float ki;
        float kd;
    } speed_controller;

    MotorParameter* motor_parameter;
    float           sample_time;
};
class ClassicSixStepControl : public Configurable<ClassicSixStepControlConfig> {
   public:
    ClassicSixStepControl(Driver* driver)
        : _driver(driver),
          _six_step_control(&_power_sensor, &_phase_voltage_sensor, &_zero_cross_detector,
                            &_zero_cross_detector, &_zero_cross_detector, &_speed_controller,
                            &_current_controller, &_modular, _driver) {
    }
    Result apply_config() override;

    void set_command(Motor& motor, SixStepCommand& cmd);
    void command_loop(Motor& motor);
    void hf_loop(Motor& motor);
    void calibrate(Motor& motor);

   private:
    SamplingPowerSensor             _power_sensor;
    SamplingPhaseVoltageSensor      _phase_voltage_sensor;
    EmfZeroCrossSectionSensor       _zero_cross_detector;
    DirectPidSpeedController        _speed_controller;
    PassThroughBusCurrentController _current_controller;
    SixPwmModular                   _modular;
    Driver*                         _driver;

    SixStepControlBase _six_step_control;

    SixStepCommand _cmd;
};

}  // namespace wibot::motor

#endif  // WWMOTOR_APP_MOTOR2_SIXSTEPCONTROLLER_HPP_
