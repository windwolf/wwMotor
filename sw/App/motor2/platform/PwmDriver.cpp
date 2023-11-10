#include "PwmDriver.hpp"

namespace wibot::motor {
void PwmDriver::setDuty(Motor& motor) {
    if (_breakdown_flag) {
        return;
    }

    _pwm->setDuty(_config.channelA, motor.reference.d_abc.v1);
    _pwm->setDuty(_config.channelB, motor.reference.d_abc.v2);
    _pwm->setDuty(_config.channelC, motor.reference.d_abc.v3);
    _pwm->setDuty(_config.channelS, motor.reference.d_sample);  //TODO: must not be 1
    controlChannel(motor.reference.sw_channel);
};

void PwmDriver::controlChannel(PwmChannel channel) {
    if (_breakdown_flag) {
        return;
    }
    if (channel & 0x01) {
        _pwm->enableChannel(_config.channelA);
    } else {
        _pwm->disableChannel(_config.channelA);
    }
    if (channel & 0x02) {
        _pwm->enableChannel(_config.channelB);
    } else {
        _pwm->disableChannel(_config.channelB);
    }
    if (channel & 0x04) {
        _pwm->enableChannel(_config.channelC);
    } else {
        _pwm->disableChannel(_config.channelC);
    }
    if (channel & 0x08) {
        _pwm->enableChannel(_config.channelS);
    } else {
        _pwm->disableChannel(_config.channelS);
    }
};

void PwmDriver::breakdown() {
    _breakdown_flag = true;
    _pwm->disableChannel(_config.channelA | _config.channelB | _config.channelC);
};
void PwmDriver::resume() {
    _breakdown_flag = false;
    _pwm->enableChannel(_config.channelA | _config.channelB | _config.channelC);
};
void PwmDriver::prepareCharge() {
    if (_breakdown_flag) {
        return;
    }
    _pwm->enableChannel(_config.channelA | _config.channelB | _config.channelC);
    _pwm->setDuty(_config.channelA, 0.0f);
    _pwm->setDuty(_config.channelB, 0.0f);
    _pwm->setDuty(_config.channelC, 0.0f);
    _pwm->disableChannel(_config.channelA | _config.channelB | _config.channelC);
}
void PwmDriver::setConfig(PwmDriverConfig& _config) {
    _config = _config;
}
}  // namespace wibot::motor
