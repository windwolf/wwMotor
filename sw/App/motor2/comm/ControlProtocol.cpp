//
// Created by zhouj on 2023/4/1.
//

#include "ControlProtocol.hpp"

namespace wibot {
namespace motor {

void ControlProtocol::doRxWork() {
    MessageFrame frame(Buffer8{.data = _frameBuffer, .size = 16});
    auto         suc = _parser.parse(&frame);
    if (suc == Result::OK) {
        auto cmd = static_cast<CommandType>(frame.getCommand().data[0]);
        switch (cmd) {
            case CommandType::PID_CURRENT_GET_REQ: {
                this->_reqState.pidCurrent = true;
                break;
            }
            case CommandType::PID_SPEED_GET_REQ: {
                this->_reqState.pidSpeed = true;
                break;
            }
            case CommandType::PID_POSITION_GET_REQ: {
                this->_reqState.pidPosition = true;
                break;
            }
            case CommandType::PID_CURRENT_SET_REQ:
            case CommandType::PID_SPEED_SET_REQ:
            case CommandType::PID_POSITION_SET_REQ: {
                auto ctn = frame.getContent();
                _doSetPid(cmd, ctn.getFloat(0), ctn.getFloat(4), ctn.getFloat(8));
                break;
            }
            case CommandType::MODE_STOP_REQ: {
                _doStop();
                break;
            }
            case CommandType::MODE_CALIBRATE_REQ: {
                _doCalibrate();
                break;
            }
            case CommandType::MODE_OPEN_LOOP_REQ: {
                auto ctn = frame.getContent();
                _doOpenLoop(ctn.getFloat(0), ctn.getFloat(4));
                break;
            }
            case CommandType::MODE_CURRENT_CLOSE_LOOP_REQ: {
                auto ctn = frame.getContent();
                _doCurrentCloseLoop(ctn.getFloat(0), ctn.getFloat(4));
                break;
            }
            case CommandType::MODE_SPEED_CLOSE_LOOP_REQ: {
                auto ctn = frame.getContent();
                _doSpeedLoop(ctn.getFloat(0));
                break;
            }
            case CommandType::MODE_POSITION_CLOSE_LOOP_REQ: {
                auto ctn = frame.getContent();
                _doPositionLoop(ctn.getFloat(0));
                break;
            }

            case CommandType::MONITOR_UBUS_START_REQ: {
                _monitorState.ubus = true;
                break;
            }
            case CommandType::MONITOR_UBUS_STOP_REQ: {
                _monitorState.ubus = false;
                break;
            }

            case CommandType::MONITOR_IBUS_START_REQ: {
                _monitorState.ibus = true;
                break;
            }
            case CommandType::MONITOR_IBUS_STOP_REQ: {
                _monitorState.ibus = false;
                break;
            }

            case CommandType::MONITOR_SPDM_START_REQ: {
                _monitorState.spdm = true;
                break;
            }
            case CommandType::MONITOR_SPDM_STOP_REQ: {
                _monitorState.spdm = false;
                break;
            }

            case CommandType::MONITOR_SPDMREF_START_REQ: {
                _monitorState.spdm_ref = true;
                break;
            }
            case CommandType::MONITOR_SPDMREF_STOP_REQ: {
                _monitorState.spdm_ref = false;
                break;
            }

            case CommandType::MONITOR_POSM_START_REQ: {
                _monitorState.posm = true;
                break;
            }
            case CommandType::MONITOR_POSM_STOP_REQ: {
                _monitorState.posm = false;
                break;
            }

            case CommandType::MONITOR_POSMREF_START_REQ: {
                _monitorState.posm_ref = true;
                break;
            }
            case CommandType::MONITOR_POSMREF_STOP_REQ: {
                _monitorState.posm_ref = false;
                break;
            }

            case CommandType::MONITOR_UABC_START_REQ: {
                _monitorState.uabc = true;
                break;
            }
            case CommandType::MONITOR_UABC_STOP_REQ: {
                _monitorState.uabc = false;
                break;
            }

            case CommandType::MONITOR_DABCREF_START_REQ: {
                _monitorState.dabc_ref = true;
                break;
            }
            case CommandType::MONITOR_DABCREF_STOP_REQ: {
                _monitorState.dabc_ref = false;
                break;
            }

            case CommandType::MONITOR_IABC_START_REQ: {
                _monitorState.iabc = true;
                break;
            }
            case CommandType::MONITOR_IABC_STOP_REQ: {
                _monitorState.iabc = false;
                break;
            }

                //            case CommandType::MONITOR_IABCREF_START_REQ: {
                //                _monitorState.iabc_ref = true;
                //                break;
                //            }
                //            case CommandType::MONITOR_IABCREF_STOP_REQ: {
                //                _monitorState.iabc_ref = false;
                //                break;
                //            }
                //
                //            case CommandType::MONITOR_UDQ_START_REQ: {
                //                _monitorState.udq = true;
                //                break;
                //            }
                //            case CommandType::MONITOR_UDQ_STOP_REQ: {
                //                _monitorState.udq = false;
                //                break;
                //            }

            case CommandType::MONITOR_UDQREF_START_REQ: {
                _monitorState.udq_ref = true;
                break;
            }
            case CommandType::MONITOR_UDQREF_STOP_REQ: {
                _monitorState.udq_ref = false;
                break;
            }

            case CommandType::MONITOR_IDQ_START_REQ: {
                _monitorState.idq = true;
                break;
            }
            case CommandType::MONITOR_IDQ_STOP_REQ: {
                _monitorState.idq = false;
                break;
            }

            case CommandType::MONITOR_IDQREF_START_REQ: {
                _monitorState.idq_ref = true;
                break;
            }
            case CommandType::MONITOR_IDQREF_STOP_REQ: {
                _monitorState.idq_ref = false;
                break;
            }

            case CommandType::MONITOR_SEC_START_REQ: {
                _monitorState.sec = true;
                break;
            }
            case CommandType::MONITOR_SEC_STOP_REQ: {
                _monitorState.sec = false;
                break;
            }

            case CommandType::MONITOR_SECREF_START_REQ: {
                _monitorState.sec_ref = true;
                break;
            }
            case CommandType::MONITOR_SECREF_STOP_REQ: {
                _monitorState.sec_ref = false;
                break;
            }

            case CommandType::MONITOR_SPDE_START_REQ: {
                _monitorState.spde = true;
                break;
            }
            case CommandType::MONITOR_SPDE_STOP_REQ: {
                _monitorState.spde = false;
                break;
            }

                //            case CommandType::MONITOR_SPDEREF_START_REQ: {
                //                _monitorState.spde_ref = true;
                //                break;
                //            }
                //            case CommandType::MONITOR_SPDEREF_STOP_REQ: {
                //                _monitorState.spde_ref = false;
                //                break;
                //            }

            case CommandType::MONITOR_POSE_START_REQ: {
                _monitorState.pose = true;
                break;
            }
            case CommandType::MONITOR_POSE_STOP_REQ: {
                _monitorState.pose = false;
                break;
            }

                //            case CommandType::MONITOR_POSEREF_START_REQ: {
                //                _monitorState.pose_ref = true;
                //                break;
                //            }
                //            case CommandType::MONITOR_POSEREF_STOP_REQ: {
                //                _monitorState.pose_ref = false;
                //                break;
                //            }

            case CommandType::MONITOR_IBUSREF_START_REQ: {
                _monitorState.ibus_ref = true;
                break;
            }
            case CommandType::MONITOR_IBUSREF_STOP_REQ: {
                _monitorState.ibus_ref = false;
                break;
            }

            case CommandType::MONITOR_DBUSREF_START_REQ: {
                _monitorState.dbus_ref = true;
                break;
            }
            case CommandType::MONITOR_DBUSREF_STOP_REQ: {
                _monitorState.dbus_ref = false;
                break;
            }

            case CommandType::MONITOR_SWREF_START_REQ: {
                _monitorState.sw_ref = true;
                break;
            }
            case CommandType::MONITOR_SWREF_STOP_REQ: {
                _monitorState.sw_ref = false;
                break;
            }

            case CommandType::MONITOR_DSPLREF_START_REQ: {
                _monitorState.dsample_ref = true;
                break;
            }
            case CommandType::MONITOR_DSPLREF_STOP_REQ: {
                _monitorState.dsample_ref = false;
                break;
            }

            default:
                break;
        }
    }
}
Buffer8 ControlProtocol::_doGetPid(PidType type) {
    uint8_t      bufferData[16];
    Buffer8      buffer{.data = bufferData, .size = 16};
    MessageFrame frame(buffer);

    Vector3f pid;
    switch (type) {
        case PidType::current:
            pid = _focCtrl.getCurrentPid();
            frame.getCommand().setUint8(0, static_cast<uint8_t>(CommandType::PID_CURRENT_GET_RESP));
            break;
        case PidType::speed:
            //            pid = _focCtrl.getCurrentPid();
            //            frame.getCommand().setUint8(0,
            //            static_cast<uint8_t>(CommandType::PID_SPEED_GET_RESP));
            break;
        case PidType::position:
            //            pid = _focCtrl.getCurrentPid();
            //            frame.getCommand().setUint8(0,
            //                                        static_cast<uint8_t>(CommandType::PID_POSITION_GET_RESP));
            break;
    }

    auto ctn = frame.getContent();
    ctn.setFloat(0, pid.v1, true);
    ctn.setFloat(4, pid.v2, true);
    ctn.setFloat(8, pid.v3, true);
    return frame.getFrameData();
}

void ControlProtocol::_doSetPid(CommandType type, float p, float i, float d) {
    switch (type) {
        case CommandType::PID_CURRENT_GET_REQ: {
            _focCtrl.setCurrentPid(p, i, d);
            break;
        }
        case CommandType::PID_SPEED_SET_REQ: {
            //_focCtrl.set_speed_pid(_motor, p, i, d);
            break;
        }
        case CommandType::PID_POSITION_SET_REQ: {
            //_focCtrl.set_position_pid(_motor, p, i, d);
            break;
        }
        default:
            break;
    }
}
void ControlProtocol::_doStop() {
    FocCommand cmd(MotorRunMode::Stop);
    _focCtrl.set_command(_motor, cmd);
}
void ControlProtocol::_doCalibrate() {
    FocCommand cmd(MotorRunMode::Calibrate);
    _focCtrl.set_command(_motor, cmd);
}
void ControlProtocol::_doOpenLoop(float ud, float uq) {
    FocCommand cmd(MotorRunMode::OpenLoop);
    cmd.voltage = Vector2f(ud, uq);
    _focCtrl.set_command(_motor, cmd);
}
void ControlProtocol::_doCurrentCloseLoop(float id, float iq) {
    FocCommand cmd(MotorRunMode::Current);
    cmd.current = Vector2f(id, iq);
    _focCtrl.set_command(_motor, cmd);
}
void ControlProtocol::_doSpeedLoop(float ref) {
    FocCommand cmd(MotorRunMode::Speed);
    cmd.speed = ref;
    _focCtrl.set_command(_motor, cmd);
}
void ControlProtocol::_doPositionLoop(float ref) {
    FocCommand cmd(MotorRunMode::Position);
    cmd.position = ref;
    _focCtrl.set_command(_motor, cmd);
}
Buffer8 ControlProtocol::_doMonitor() {
    // prefix:2, cmd:1, length:1, (state:4, tick:4, data...), crc:1
    uint32_t ctnLen = 8;
    auto     state  = _monitorState;
    if (state.ubus) {
        ctnLen += 4;
    }
    if (state.ibus) {
        ctnLen += 4;
    }
    if (state.spdm) {
        ctnLen += 4;
    }
    if (state.spdm_ref) {
        ctnLen += 4;
    }
    if (state.posm) {
        ctnLen += 4;
    }
    if (state.posm_ref) {
        ctnLen += 4;
    }
    if (state.uabc) {
        ctnLen += 12;
    }
    if (state.dabc_ref) {
        ctnLen += 12;
    }
    if (state.iabc) {
        ctnLen += 12;
    }
    //    if (state.iabc_ref) {
    //        ctnLen += 12;
    //    }
    //    if (state.udq) {
    //        ctnLen += 8;
    //    }
    if (state.udq_ref) {
        ctnLen += 8;
    }
    if (state.idq) {
        ctnLen += 8;
    }
    if (state.idq_ref) {
        ctnLen += 8;
    }
    if (state.sec) {
        ctnLen += 1;
    }
    if (state.sec_ref) {
        ctnLen += 1;
    }
    if (state.spde) {
        ctnLen += 4;
    }
    //    if (state.spde_ref) {
    //        ctnLen += 4;
    //    }
    if (state.pose) {
        ctnLen += 4;
    }
    //    if (state.pose_ref) {
    //        ctnLen += 4;
    //    }
    if (state.ibus_ref) {
        ctnLen += 4;
    }
    if (state.sw_ref) {
        ctnLen += 1;
    }
    if (state.dsample_ref) {
        ctnLen += 4;
    }

    MessageFrame frame({_tx_buffer, sizeof(_tx_buffer)}, schema, schema.lengthSchemas[7].length,
                       ctnLen);
    frame.getCommand().setUint8(0, static_cast<uint8_t>(CommandType::MONITOR_RESP));
    auto          ctn = frame.getContent();
    Buffer8Setter bs(ctn);
    bs.setUint32(Utils::tick_get());
    if (state.ubus) {
        bs.setFloat(_motor.state.u_bus);
    }
    if (state.ibus) {
        bs.setFloat(_motor.state.i_bus);
    }
    if (state.spdm) {
        bs.setFloat(_motor.state.speed.v2);
    }
    if (state.spdm_ref) {
        bs.setFloat(_motor.reference.speed);
    }
    if (state.posm) {
        bs.setFloat(_motor.state.position.v2);
    }
    if (state.posm_ref) {
        bs.setFloat(_motor.reference.position);
    }
    if (state.uabc) {
        bs.setFloat(_motor.state.u_abc.v1);
        bs.setFloat(_motor.state.u_abc.v2);
        bs.setFloat(_motor.state.u_abc.v3);
    }
    if (state.dabc_ref) {
        bs.setFloat(_motor.reference.d_abc.v1);
        bs.setFloat(_motor.reference.d_abc.v2);
        bs.setFloat(_motor.reference.d_abc.v3);
    }
    if (state.iabc) {
        bs.setFloat(_motor.state.i_abc.v1);
        bs.setFloat(_motor.state.i_abc.v2);
        bs.setFloat(_motor.state.i_abc.v3);
    }
    //    if (state.iabc_ref) {
    //        bs.setFloat(.0f);
    //        bs.setFloat(.0f);
    //        bs.setFloat(.0f);
    //    }
    //    if (state.udq) {
    //        bs.setFloat(_motor.state..v1);
    //        bs.setFloat(_motor.state.i_abc.v2);
    //        bs.setFloat(_motor.state.i_abc.v3);
    //    }
    if (state.udq_ref) {
        bs.setFloat(_motor.reference.u_dq.v1);
        bs.setFloat(_motor.reference.u_dq.v2);
    }
    if (state.idq) {
        bs.setFloat(_motor.state.i_dq.v1);
        bs.setFloat(_motor.state.i_dq.v2);
    }
    if (state.idq_ref) {
        bs.setFloat(_motor.reference.i_dq.v1);
        bs.setFloat(_motor.reference.i_dq.v2);
    }
    if (state.sec) {
        bs.setUint8(_motor.state.section);
    }
    if (state.sec_ref) {
        bs.setUint8(_motor.reference.section);
    }
    if (state.spde) {
        bs.setFloat(_motor.state.speed.v1);
    }
    //    if (state.spde_ref) {
    //        ctnLen += 4;
    //    }
    if (state.pose) {
        bs.setFloat(_motor.state.position.v1);
    }
    //    if (state.pose_ref) {
    //        ctnLen += 4;
    //    }
    if (state.ibus_ref) {
        bs.setFloat(_motor.reference.i_bus);
    }
    if (state.sw_ref) {
        bs.setUint8(_motor.reference.sw_channel);
    }
    if (state.dsample_ref) {
        bs.setFloat(_motor.reference.d_sample);
    }
    return frame.getFrameData();
}
Buffer8 ControlProtocol::doTxWork() {
    if (_reqState.pidCurrent) {
        _reqState.pidCurrent = false;
        return _doGetPid(PidType::current);
    }
    if (_reqState.pidSpeed) {
        _reqState.pidSpeed = false;
        return _doGetPid(PidType::speed);
    }
    if (_reqState.pidPosition) {
        _reqState.pidPosition = false;
        return _doGetPid(PidType::position);
    }
    return _doMonitor();
}

void ControlProtocol::startRx() {
    _resetUart();
    while (true) {
        auto rxRst = _rxWaitHandle.wait(TIMEOUT_FOREVER);
        if (rxRst == Result::GeneralError) {
            _resetUart();
        } else if (rxRst == Result::OK) {
            doRxWork();
        }
    }
}

void ControlProtocol::_resetUart() {
    _uart.stop();
    _rx_cir_buffer.clear();
    _parser.reset();
    _uart.start(_rx_cir_buffer, _rxWaitHandle);
};

void ControlProtocol::startTx() {
    do {
        Buffer8 data  = doTxWork();
        Result  txRst = _uart.write(data.data, data.size, _txWaitHandle);
        if (txRst == Result::OK) {
            txRst = _rxWaitHandle.wait(TIMEOUT_FOREVER);
            if (txRst == Result::GeneralError) {
                _resetUart();
            } else if (txRst == Result::OK) {
                doRxWork();
            }
        }
        os::Utils::delay(1);
    } while (true);
};

}  // namespace motor
}  // namespace wibot
