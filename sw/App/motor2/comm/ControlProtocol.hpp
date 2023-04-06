//
// Created by zhouj on 2023/4/1.
//

#ifndef WWMOTOR_APP_MOTOR2_COMM_CONTROLPROTOCOL_HPP_
#define WWMOTOR_APP_MOTOR2_COMM_CONTROLPROTOCOL_HPP_

#include "CircularBuffer.hpp"
#include "message_parser.hpp"
#include "motor2/FocControl.hpp"
#include "os.hpp"
#include "uart.hpp"
#include "wait_handler.hpp"

namespace wibot {
namespace motor {
using namespace wibot::peripheral;
using namespace wibot::comm;

enum class CommandType : uint8_t {
    MODE_STOP_REQ                = 0x00,
    MODE_CALIBRATE_REQ           = 0x01,
    MODE_OPEN_LOOP_REQ           = 0x02,
    MODE_CURRENT_CLOSE_LOOP_REQ  = 0x03,
    MODE_SPEED_CLOSE_LOOP_REQ    = 0x04,
    MODE_POSITION_CLOSE_LOOP_REQ = 0x05,

    PID_CURRENT_SET_REQ   = 0x11,
    PID_CURRENT_GET_REQ   = 0x13,
    PID_CURRENT_GET_RESP  = 0x14,
    PID_SPEED_SET_REQ     = 0x15,
    PID_SPEED_GET_REQ     = 0x17,
    PID_SPEED_GET_RESP    = 0x18,
    PID_POSITION_SET_REQ  = 0x19,
    PID_POSITION_GET_REQ  = 0x1B,
    PID_POSITION_GET_RESP = 0x1C,

    MONITOR_RESP            = 0x30,
    MONITOR_STATE_START_REQ = 0x31,  // ubus, ibus, spdm, posm,
    MONITOR_STATE_END_REQ   = 0x32,

    MONITOR_ABC_START_REQ = 0x33,  // iabc, uabc(dabc_ref)
    MONITOR_ABC_END_REQ   = 0x34,

    MONITOR_DQ_START_REQ = 0x35,  // idq, idq_ref
    MONITOR_DQ_END_REQ   = 0x36,

    MONITOR_SPD_START_REQ = 0x37,  // spde, spdm, spde_ref, spdm_ref
    MONITOR_SPD_END_REQ   = 0x38,

    MONITOR_POS_START_REQ = 0x39,  // pose, spdm, pose_ref, spdm_ref
    MONITOR_POS_END_REQ   = 0x3A,
};

class ControlProtocol : public Initializable {
   private:
    static constexpr uint32_t RX_BUFFER_SIZE = 64;
    static constexpr uint32_t TX_BUFFER_SIZE = 256;
    UART&                     _uart;
    uint8_t                   _rx_buffer[RX_BUFFER_SIZE];
    uint8_t                   _tx_buffer[TX_BUFFER_SIZE];
    CircularBuffer<uint8_t>   _rx_cir_buffer;
    CircularBuffer<uint8_t>   _tx_cir_buffer;
    MessageParser             _parser;
    os::EventGroup            _eventGroup;
    WaitHandler               _rxWaitHandle;
    WaitHandler               _txWaitHandle;
    WaitHandler               _waitHandle;
    uint8_t                   _frameBuffer[16];
    FocControl&               _focCtrl;
    Motor&                    _motor;
    bool                      _monitorState = false;
    bool                      _monitorAbc   = false;
    bool                      _monitorDq    = false;
    bool                      _monitorSpd   = false;
    bool                      _monitorPos   = false;
    bool                      _pidCurrReq   = false;
    bool                      _pidSpdReq    = false;
    bool                      _pidPosReq    = false;

   public:
    ControlProtocol(UART& uart, FocControl& focCtrl, Motor& motor)
        : _uart(uart), _rx_cir_buffer(_rx_buffer, RX_BUFFER_SIZE),
          _tx_cir_buffer(_tx_buffer, TX_BUFFER_SIZE), _parser(_rx_cir_buffer), _eventGroup("comm"),
          _rxWaitHandle(_eventGroup), _txWaitHandle(_eventGroup),
          _waitHandle(_rxWaitHandle.merge(_txWaitHandle)), _focCtrl(focCtrl), _motor(motor) {
        _parser.init(schema);
    }

   protected:
    Result _init() override {
        _eventGroup.init();
        return Result::OK;
    }
    void _deinit() override {
        _eventGroup.deinit();
    }

   public:
    void start() {
        _uart.start(_rx_cir_buffer, _rxWaitHandle);
        while (true) {
            auto rst = _waitHandle.wait(TIMEOUT_FOREVER);
            if (rst == Result::OK) {
                if (_waitHandle.triggeredFor(_rxWaitHandle) == Result::OK) {
                    doRxWork();
                }
                if (_waitHandle.triggeredFor(_txWaitHandle) == Result::OK) {
                    doTxWork();
                }
            }
        }
    }

    void doTxWork() {
        _doMonitor();
        if (_pidCurrReq) {
            _doGetCurrentPid();
        }
        if (_pidSpdReq) {
            //_doGetSpeedPid();
        }
        if (_pidPosReq) {
            //_doGetPositionPid();
        }

        // _uart.write(_tx_cir_buffer.readVirtual(), _txWaitHandle);
    }

    void doRxWork() {
        MessageFrame frame(Buffer8{.data = _frameBuffer, .size = 16});
        auto         suc = _parser.parse(&frame);
        if (suc == Result::OK) {
            auto cmd = static_cast<CommandType>(frame.getCommand().data[0]);
            switch (cmd) {
                case CommandType::PID_CURRENT_GET_REQ: {
                    _pidCurrReq = true;
                    break;
                }
                case CommandType::PID_SPEED_GET_REQ: {
                    _pidSpdReq = true;
                    break;
                }
                case CommandType::PID_POSITION_GET_REQ: {
                    _pidPosReq = true;
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
                case CommandType::MONITOR_DQ_START_REQ: {
                    _monitorDq = true;
                    break;
                }
                case CommandType::MONITOR_DQ_END_REQ: {
                    _monitorDq = false;
                    break;
                }
                default:
                    break;
            }
        }
    }

   private:
    void _doMonitor() {
        uint8_t      bufferData[16];
        Buffer8      buffer{.data = bufferData, .size = 16};
        auto         pid = _focCtrl.getCurrentPid();
        MessageFrame frame(buffer);
        frame.getCommand().setUint8(0, static_cast<uint8_t>(CommandType::PID_CURRENT_GET_RESP));
        auto ctn = frame.getContent();
        ctn.setFloat(0, pid.v1, true);
        ctn.setFloat(4, pid.v2, true);
        ctn.setFloat(8, pid.v3, true);
        auto fd = frame.getFrameData();
        _tx_cir_buffer.write(fd.data, fd.size);
        if (_monitorState) {
        }
        if (_monitorAbc) {
            //_doMonitorAbc();
        }
        if (_monitorDq) {
            //_doMonitorDq();
        }
        if (_monitorSpd) {
            // _doMonitorSpd();
        }
        if (_monitorPos) {
            // _doMonitorPos();
        }
    }

    void _doGetCurrentPid() {
        uint8_t      bufferData[16];
        Buffer8      buffer{.data = bufferData, .size = 16};
        auto         pid = _focCtrl.getCurrentPid();
        MessageFrame frame(buffer);
        frame.getCommand().setUint8(0, static_cast<uint8_t>(CommandType::PID_CURRENT_GET_RESP));
        auto ctn = frame.getContent();
        ctn.setFloat(0, pid.v1, true);
        ctn.setFloat(4, pid.v2, true);
        ctn.setFloat(8, pid.v3, true);
        auto fd = frame.getFrameData();
        _tx_cir_buffer.write(fd.data, fd.size);
    }

    void _doSetPid(CommandType type, float p, float i, float d) {
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
    void _doStop() {
        FocCommand cmd(MotorRunMode::Stop);
        _focCtrl.set_command(_motor, cmd);
    };
    void _doCalibrate() {
        FocCommand cmd(MotorRunMode::Calibrate);
        _focCtrl.set_command(_motor, cmd);
    };
    void _doOpenLoop(float ud, float uq) {
        FocCommand cmd(MotorRunMode::OpenLoop);
        cmd.voltage = Vector2f(ud, uq), _focCtrl.set_command(_motor, cmd);
    };
    void _doCurrentCloseLoop(float id, float iq) {
        FocCommand cmd(MotorRunMode::Current);
        cmd.current = Vector2f(id, iq);
        _focCtrl.set_command(_motor, cmd);
    };
    void _doSpeedLoop(float ref) {
        FocCommand cmd(MotorRunMode::Speed);
        cmd.speed = ref;
        _focCtrl.set_command(_motor, cmd);
    };
    void _doPositionLoop(float ref) {
        FocCommand cmd(MotorRunMode::Position);
        cmd.position = ref;
        _focCtrl.set_command(_motor, cmd);
    };

   private:
    constexpr static MessageLengthSchemaDefinition definitions[7] = {
        {
            .command = {static_cast<uint8_t>(CommandType::MODE_OPEN_LOOP_REQ)},
            .length{
                .mode = MESSAGE_LENGTH_SCHEMA_MODE::FIXED_LENGTH,
                .fixed{.length = 8},
            },
        },
        {
            .command = {static_cast<uint8_t>(CommandType::MODE_CURRENT_CLOSE_LOOP_REQ)},
            .length{
                .mode = MESSAGE_LENGTH_SCHEMA_MODE::FIXED_LENGTH,
                .fixed{.length = 8},
            },
        },
        {
            .command = {static_cast<uint8_t>(CommandType::MODE_SPEED_CLOSE_LOOP_REQ)},
            .length{
                .mode = MESSAGE_LENGTH_SCHEMA_MODE::FIXED_LENGTH,
                .fixed{.length = 4},
            },
        },
        {
            .command = {static_cast<uint8_t>(CommandType::MODE_POSITION_CLOSE_LOOP_REQ)},
            .length{
                .mode = MESSAGE_LENGTH_SCHEMA_MODE::FIXED_LENGTH,
                .fixed{.length = 4},
            },
        },
        {
            .command = {static_cast<uint8_t>(CommandType::PID_CURRENT_SET_REQ)},
            .length{
                .mode = MESSAGE_LENGTH_SCHEMA_MODE::FIXED_LENGTH,
                .fixed{.length = 12},
            },
        },
        {
            .command = {static_cast<uint8_t>(CommandType::PID_SPEED_SET_REQ)},
            .length{
                .mode = MESSAGE_LENGTH_SCHEMA_MODE::FIXED_LENGTH,
                .fixed{.length = 12},
            },
        },
        {
            .command = {static_cast<uint8_t>(CommandType::PID_POSITION_SET_REQ)},
            .length{
                .mode = MESSAGE_LENGTH_SCHEMA_MODE::FIXED_LENGTH,
                .fixed{.length = 12},
            },
        }};
    constexpr static MessageSchema schema = {

        .prefix        = {0x55, 0x50},
        .prefixSize    = 2,
        .commandSize   = MESSAGE_SCHEMA_SIZE::BIT8,
        .lengthSchemas = const_cast<MessageLengthSchemaDefinition*>(ControlProtocol::definitions),
        .lengthSchemaCount = 7,
        .defaultLength{
            .mode = MESSAGE_LENGTH_SCHEMA_MODE::FIXED_LENGTH,
            .fixed{.length = 0},
        },

        .crcSize = MESSAGE_SCHEMA_SIZE::BIT8,
    };
};

}  // namespace motor
}  // namespace wibot

#endif  // WWMOTOR_APP_MOTOR2_COMM_CONTROLPROTOCOL_HPP_
