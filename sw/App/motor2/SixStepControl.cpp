//
// Created by zhouj on 2022/11/28.
//

#include "SixStepControl.hpp"
#include "os.hpp"

namespace wibot::motor {

void SixStepControlBase::hf_loop(Motor& motor) {
    _power_sensor->u_bus_get(motor, motor.state.u_bus);
    _power_sensor->i_bus_get(motor, motor.state.i_bus);
    _phase_voltage_sensor->u_abc_get(motor, motor.state.u_abc);
    _section_sensor->section_get(motor, motor.state.section);
    _section_switcher->section_switch(motor, motor.reference.section);
    _position_speed_sensor->position_speed_get(motor, motor.state.position, motor.state.speed);
    switch (motor.mode) {
        case MotorRunMode::Speed:
            _bus_current_reference_updater->ibus_update(motor, motor.reference.i_bus);
        case MotorRunMode::Current:
            _bus_duty_reference_updater->dbus_update(motor, motor.reference.d_bus);
        case MotorRunMode::OpenLoop:
        case MotorRunMode::Calibrate:
            _modular->module(motor, motor.reference.section, motor.reference.d_abc,
                             motor.reference.u_abc, motor.reference.sw_channel,
                             motor.reference.d_sample);
        case MotorRunMode::Stop:
            motor.reference.d_bus = 0;
            break;
        case MotorRunMode::Position:
            break;
    }
    _driver->setDuty(motor);
}
void ClassicSixStepControl::set_command(Motor& motor, SixStepCommand& cmd) {
    _cmd = cmd;
    // TODO: 根据配置, 生成控制轨迹.
}
void ClassicSixStepControl::command_loop(Motor& motor) {
    switch (_cmd.mode) {
        case MotorRunMode::Stop:
            motor.mode = MotorRunMode::Stop;
            break;
        case MotorRunMode::Calibrate:
            motor.mode = MotorRunMode::Calibrate;
            break;
        case MotorRunMode::OpenLoop:
            motor.mode            = MotorRunMode::OpenLoop;
            motor.reference.d_bus = _cmd.duty;
            break;
        case MotorRunMode::Current:
            motor.mode            = MotorRunMode::Current;
            motor.reference.i_bus = _cmd.current;
            break;
        case MotorRunMode::Speed:
            motor.mode            = MotorRunMode::Speed;
            motor.reference.speed = _cmd.speed;
            break;
        case MotorRunMode::Position:
            motor.mode = MotorRunMode::Stop;
            break;
    }
}

void ClassicSixStepControl::hf_loop(Motor& motor) {
    this->_six_step_control.hf_loop(motor);
}
void ClassicSixStepControl::calibrate(Motor& motor) {
}
Result ClassicSixStepControl::apply_config() {
    Result rst;
    this->_power_sensor.config.u_bus    = config.power_sensor.u_bus;
    this->_power_sensor.config.u_bus_pu = config.power_sensor.u_bus_pu;
    this->_power_sensor.config.i_bus    = config.power_sensor.i_bus;
    this->_power_sensor.config.i_bus_pu = config.power_sensor.i_bus_pu;
    rst                                 = _power_sensor.apply_config();
    if (rst != Result::OK) {
        return rst;
    }

    this->_phase_voltage_sensor.config.u_a  = config.phase_voltage_sensor.u_a;
    this->_phase_voltage_sensor.config.u_b  = config.phase_voltage_sensor.u_b;
    this->_phase_voltage_sensor.config.u_c  = config.phase_voltage_sensor.u_c;
    this->_phase_voltage_sensor.config.u_pu = config.phase_voltage_sensor.u_pu;
    this->_phase_voltage_sensor.config.cutoff_freq =
        config.motor_parameter->speed_limit / 60 * config.motor_parameter->pole_pair;
    this->_phase_voltage_sensor.config.sample_time = config.sample_time;
    rst                                            = _phase_voltage_sensor.apply_config();
    if (rst != Result::OK) {
        return rst;
    }

    this->_zero_cross_detector.config.sample_time     = config.sample_time;
    this->_zero_cross_detector.config.motor_parameter = config.motor_parameter;
    this->_zero_cross_detector.config.cutoff_freq =
        config.motor_parameter->speed_limit / 60 * config.motor_parameter->pole_pair;
    this->_zero_cross_detector.config.blank_count = config.zero_cross_detector.blank_count;
    this->_zero_cross_detector.config.switch_delay_count =
        config.zero_cross_detector.switch_delay_count;
    rst = _zero_cross_detector.apply_config();
    if (rst != Result::OK) {
        return rst;
    }

    this->_speed_controller.config.motor_parameter = config.motor_parameter;
    this->_speed_controller.config.sample_time     = config.sample_time;
    this->_speed_controller.config.kp              = config.speed_controller.kp;
    this->_speed_controller.config.ki              = config.speed_controller.ki;
    this->_speed_controller.config.kd              = config.speed_controller.kd;
    rst                                            = _speed_controller.apply_config();
    if (rst != Result::OK) {
        return rst;
    }

    this->_current_controller.config.motor_parameter = config.motor_parameter;
    return _current_controller.apply_config();
}

}  // namespace wibot::motor
