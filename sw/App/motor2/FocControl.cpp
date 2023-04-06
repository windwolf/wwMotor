//
// Created by zhouj on 2022/11/16.
//

#include "FocControl.hpp"

#include "os.hpp"

namespace wibot::motor {
using namespace wibot::os;

void FocControlBase::lf_loop(Motor& motor) {
    if (motor.mode != MotorRunMode::Calibrate && motor.mode != MotorRunMode::Stop) {
        _positionSpeedSensor->position_speed_get(motor, motor.state.position, motor.state.speed);
    }
    _sectionSensor->section_get(motor, motor.state.section);

    // 故意不break, 为了让后面的代码也执行.
    switch (motor.mode) {
        case MotorRunMode::Position:
            _speedReferenceUpdater->speed_update(motor, motor.reference.speed);
        case MotorRunMode::Speed:
            _dqCurrentReferenceUpdater->dq_current_update(motor, motor.reference.i_dq);
        case MotorRunMode::Current:
        case MotorRunMode::OpenLoop:
        case MotorRunMode::Calibrate:
        case MotorRunMode::Stop:
            break;
    }
}
void FocControlBase::hf_loop(Motor& motor) {
    _powerSensor->u_bus_get(motor, motor.state.u_bus);
    _powerSensor->i_bus_get(motor, motor.state.i_bus);

    _phaseCurrentSensor->i_ab_get(motor, motor.state.i_ab);
    // FocMath::abc2ab(motor.state.i_abc, motor.state.i_ab);
    FocMath::ab2dq(motor.state.i_ab, motor.state.position.v1, motor.state.i_dq);
    //  故意不break, 为了让后面的代码也执行.
    switch (motor.mode) {
        case MotorRunMode::Position:
        case MotorRunMode::Speed:
        case MotorRunMode::Current:
            _dqVoltageReferenceUpdater->dq_voltage_update(motor, motor.reference.u_dq);
        case MotorRunMode::OpenLoop:
        case MotorRunMode::Calibrate:
            _modular->module(motor, motor.reference.section, motor.reference.d_abc,
                             motor.reference.u_abc, motor.reference.sw_channel,
                             motor.reference.d_sample);
            break;
        case MotorRunMode::Stop:
            motor.reference.d_abc      = Vector3f(0, 0, 0);
            motor.reference.u_abc      = Vector3f(0, 0, 0);
            motor.reference.sw_channel = 0x00;
            motor.reference.d_sample   = 0.5f;
            break;
    }

    _driver->duty_set(motor);
}

void FocControl::lf_loop(Motor& motor) {
    _focControl.lf_loop(motor);
}
void FocControl::hf_loop(Motor& motor) {
    _focControl.hf_loop(motor);
}
void FocControl::set_command(Motor& motor, FocCommand& cmd) {
    _cmd = cmd;

    // TODO: 根据配置, 生成控制轨迹.
}
void FocControl::command_loop(Motor& motor) {
    switch (_cmd.mode) {
        case MotorRunMode::Stop:
            motor.mode = MotorRunMode::Stop;
            break;
        case MotorRunMode::Calibrate:
            motor.mode = MotorRunMode::Calibrate;
            break;
        case MotorRunMode::OpenLoop:
            motor.mode           = MotorRunMode::OpenLoop;
            motor.reference.u_dq = _cmd.voltage;
            break;
        case MotorRunMode::Current:
            motor.mode           = MotorRunMode::Current;
            motor.reference.i_dq = _cmd.current;
            break;
        case MotorRunMode::Speed:
            motor.mode            = MotorRunMode::Speed;
            motor.reference.speed = _cmd.speed;
            break;
        case MotorRunMode::Position:
            motor.mode               = MotorRunMode::Position;
            motor.reference.position = _cmd.position;
            break;
    }
}

void FocControl::calibrate(Motor& motor) {
    auto original_mode     = motor.mode;
    auto original_u_dq     = motor.reference.u_dq;
    auto original_d_sample = motor.reference.d_sample;
    auto original_pos      = motor.state.position;
    auto original_spd      = motor.state.speed;

    motor.mode = MotorRunMode::Calibrate;
    os::Utils::delay(100);

    motor.reference.u_dq     = Vector2f(0, 0);
    motor.reference.d_sample = 0.0f;
    motor.state.position     = Vector2f(0, 0);
    motor.state.speed        = Vector2f(0, 0);

    _phaseCurrentSensor.calibrate(motor);

    _positionSpeedSensor.calibrate(motor);

    motor.state.position     = original_pos;
    motor.state.speed        = original_spd;
    motor.reference.u_dq     = original_u_dq;
    motor.reference.d_sample = original_d_sample;
    motor.mode               = original_mode;
}
Result FocControl::apply_config() {
    Result rst                         = Result::OK;
    this->_powerSensor.config.u_bus    = config.power_sensor.u_bus;
    this->_powerSensor.config.u_bus_pu = config.power_sensor.u_bus_pu;
    rst                                = _powerSensor.apply_config();
    if (rst != Result::OK) {
        return rst;
    }
    _phaseCurrentSensor.config.i_a  = config.current_sensor.i_a;
    _phaseCurrentSensor.config.i_b  = config.current_sensor.i_b;
    _phaseCurrentSensor.config.i_c  = config.current_sensor.i_c;
    _phaseCurrentSensor.config.i_pu = config.current_sensor.i_pu;
    _phaseCurrentSensor.config.low_duty_skip_threshold =
        config.current_sensor.low_duty_skip_threshold;
    rst = _phaseCurrentSensor.apply_config();
    if (rst != Result::OK) {
        return rst;
    }
    _modular.config.motor_parameter = config.motor_parameter;
    if (config.modular.allow_over_module) {
        _modular.config.max_module_rate   = 1.0f;
        _modular.config.max_d_module_rate = 1.0f;
    } else {
        _modular.config.max_module_rate   = _1_SQRT3;
        _modular.config.max_d_module_rate = _1_SQRT3;
    }
    rst = _modular.apply_config();
    if (rst != Result::OK) {
        return rst;
    }
    _positionSpeedSensor.config.codex      = config.encoder.codex;
    _positionSpeedSensor.config.resolution = config.encoder.resolution;
    _positionSpeedSensor.config.pole_pairs = config.motor_parameter->pole_pair;
    _positionSpeedSensor.config.direction  = config.encoder.direction;
    _positionSpeedSensor.config.mech_pos_cutoff_freq =
        config.motor_parameter->speed_limit / 60 * 1.2f;
    _positionSpeedSensor.config.mech_speed_cutoff_freq =
        config.motor_parameter->speed_limit / 60 * 1.2f;
    _positionSpeedSensor.config.calibration_voltage = config.encoder.calibration_voltage;
    _positionSpeedSensor.config.sample_time         = config.low_frequency_samlpe_time;
    rst                                             = _positionSpeedSensor.apply_config();
    if (rst != Result::OK) {
        return rst;
    }
    _currentController.config.motor_parameter = config.motor_parameter;
    _currentController.config.sample_time     = config.high_frequency_samlpe_time;
    _currentController.config.useParams       = config.current_controller.useParams;
    if (_currentController.config.useParams) {
        _currentController.config.params.bandWidth = config.current_controller.bw;
    } else {
        _currentController.config.simple.p = config.current_controller.kd;
        _currentController.config.simple.i = config.current_controller.ki;
        _currentController.config.simple.d = config.current_controller.kd;
    }
    _currentController.config.disableFeedforward = config.current_controller.disableFeedforward;
    rst                                          = _currentController.apply_config();
    if (rst != Result::OK) {
        return rst;
    }
    _speedController.config.motor_parameter = config.motor_parameter;
    _speedController.config.sample_time     = config.low_frequency_samlpe_time;
    _speedController.config.bandWidth       = config.current_controller.bw;
    _speedController.config.delta           = config.speed_controller.delta;
    rst                                     = _speedController.apply_config();
    if (rst != Result::OK) {
        return rst;
    }
    _positionController.config.motor_parameter = config.motor_parameter;
    _positionController.config.sample_time     = config.low_frequency_samlpe_time;
    _positionController.config.kp              = config.position_controller.kp;
    _positionController.config.ki              = config.position_controller.ki;
    _positionController.config.kd              = config.position_controller.kd;
    return _positionController.apply_config();
}
}  // namespace wibot::motor
// wibot::motor
