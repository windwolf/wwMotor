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
    MODE_OPEN_LOOP_REQ           = 0x02,  // 8, udq_ref
    MODE_CURRENT_CLOSE_LOOP_REQ  = 0x03,  // 8, idq_ref
    MODE_SPEED_CLOSE_LOOP_REQ    = 0x04,  // 4, spdm_ref
    MODE_POSITION_CLOSE_LOOP_REQ = 0x05,  // 4, posm_ref

    PID_CURRENT_SET_REQ   = 0x11,         // 12, pid
    PID_CURRENT_GET_REQ   = 0x13,
    PID_CURRENT_GET_RESP  = 0x14,         // 12, pid
    PID_SPEED_SET_REQ     = 0x15,         // 12, pid
    PID_SPEED_GET_REQ     = 0x17,
    PID_SPEED_GET_RESP    = 0x18,         // 12, pid
    PID_POSITION_SET_REQ  = 0x19,         // 12, pid
    PID_POSITION_GET_REQ  = 0x1B,
    PID_POSITION_GET_RESP = 0x1C,         // 12, pid

    MONITOR_RESP = 0x30,

    MONITOR_UBUS_START_REQ = 0x40,  // ubus, 1
    MONITOR_UBUS_STOP_REQ  = 0x50,
    MONITOR_IBUS_START_REQ = 0x41,  // ibus, 1
    MONITOR_IBUS_STOP_REQ  = 0x51,

    MONITOR_SPDM_START_REQ    = 0x42,  // spdm 1
    MONITOR_SPDM_STOP_REQ     = 0x52,
    MONITOR_SPDMREF_START_REQ = 0x43,  // spdm_ref 1
    MONITOR_SPDMREF_STOP_REQ  = 0x53,

    MONITOR_POSM_START_REQ    = 0x44,  // posm 1
    MONITOR_POSM_STOP_REQ     = 0x54,
    MONITOR_POSMREF_START_REQ = 0x45,  // posm_ref 1
    MONITOR_POSMREF_STOP_REQ  = 0x55,

    MONITOR_UABC_START_REQ    = 0x46,  // uabc 3
    MONITOR_UABC_STOP_REQ     = 0x56,
    MONITOR_DABCREF_START_REQ = 0x47,  // dabc_ref 3
    MONITOR_DABCREF_STOP_REQ  = 0x57,
    MONITOR_IABC_START_REQ    = 0x48,  // iabc 3
    MONITOR_IABC_STOP_REQ     = 0x58,
    //    MONITOR_IABCREF_START_REQ = 0x49,  // iabc_ref 3
    //    MONITOR_IABCREF_STOP_REQ  = 0x59,

    //    MONITOR_UDQ_START_REQ    = 0x4A,  // udq 2 (foc only)
    //    MONITOR_UDQ_STOP_REQ     = 0x5A,
    MONITOR_UDQREF_START_REQ = 0x4B,  // udq_ref 2 (foc only)
    MONITOR_UDQREF_STOP_REQ  = 0x5B,
    MONITOR_IDQ_START_REQ    = 0x4C,  // idq 2 (foc only)
    MONITOR_IDQ_STOP_REQ     = 0x5C,
    MONITOR_IDQREF_START_REQ = 0x4D,  // idq_ref 2 (foc only)
    MONITOR_IDQREF_STOP_REQ  = 0x5D,

    MONITOR_SEC_START_REQ = 0x4E,  // section 0.25
    MONITOR_SEC_STOP_REQ  = 0x5E,

    MONITOR_SECREF_START_REQ = 0x4F,  // section_ref 0.25
    MONITOR_SECREF_STOP_REQ  = 0x5F,

    MONITOR_SPDE_START_REQ = 0x60,  // spde 1
    MONITOR_SPDE_STOP_REQ  = 0x70,
    //    MONITOR_SPDEREF_START_REQ = 0x61,  // spde_ref 1
    //    MONITOR_SPDEREF_STOP_REQ  = 0x71,

    MONITOR_POSE_START_REQ = 0x62,  // pose 1
    MONITOR_POSE_STOP_REQ  = 0x72,
    //    MONITOR_POSEREF_START_REQ = 0x63,  // pose_ref 1
    //    MONITOR_POSEREF_STOP_REQ  = 0x73,

    MONITOR_IBUSREF_START_REQ = 0x64,  // ibus_ref 1 (6step only)
    MONITOR_IBUSREF_STOP_REQ  = 0x74,
    MONITOR_DBUSREF_START_REQ = 0x65,  // dbus_ref 1 (6step only)
    MONITOR_DBUSREF_STOP_REQ  = 0x75,
    MONITOR_SWREF_START_REQ   = 0x66,  // sw_channel 0.25 (6step only)
    MONITOR_SWREF_STOP_REQ    = 0x76,

    MONITOR_DSPLREF_START_REQ = 0x67,  // dsample 1
    MONITOR_DSPLREF_STOP_REQ  = 0x77,
};

union MonitorState {
    uint32_t value;
    struct {
        bool ubus     : 1;
        bool ibus     : 1;
        bool spdm     : 1;
        bool spdm_ref : 1;
        bool posm     : 1;
        bool posm_ref : 1;

        bool uabc     : 1;
        bool dabc_ref : 1;
        bool iabc     : 1;
        // bool iabc_ref : 1;
        // bool udq      : 1;
        bool udq_ref  : 1;
        bool idq      : 1;
        bool idq_ref  : 1;
        bool sec      : 1;
        bool sec_ref  : 1;

        bool spde : 1;
        //        bool spde_ref : 1;
        bool pose : 1;
        //        bool pose_ref : 1;

        bool ibus_ref    : 1;
        bool dbus_ref    : 1;
        bool sw_ref      : 1;
        bool dsample_ref : 1;
    };
};
union ReqState {
    uint32_t value;
    struct {
        bool pidCurrent  : 1;
        bool pidSpeed    : 1;
        bool pidPosition : 1;
    };
};
enum class PidType {
    current,
    speed,
    position,
};

class ControlProtocol : public Initializable {
   private:
    static constexpr uint32_t RX_BUFFER_SIZE = 64;
    static constexpr uint32_t TX_BUFFER_SIZE = 256;
    UART&                     _uart;
    uint8_t                   _rx_buffer[RX_BUFFER_SIZE];
    uint8_t                   _tx_buffer[TX_BUFFER_SIZE];
    CircularBuffer<uint8_t>   _rx_cir_buffer;
    // CircularBuffer<uint8_t>   _tx_cir_buffer;
    MessageParser             _parser;
    os::EventGroup            _eventGroup;
    WaitHandler               _rxWaitHandle;
    WaitHandler               _txWaitHandle;
    uint8_t                   _frameBuffer[16];
    FocControl&               _focCtrl;
    Motor&                    _motor;
    MonitorState              _monitorState;
    ReqState                  _reqState;

   public:
    ControlProtocol(UART& uart, FocControl& focCtrl, Motor& motor)
        : _uart(uart), _rx_cir_buffer(_rx_buffer, RX_BUFFER_SIZE),
          //_tx_cir_buffer(_tx_buffer, TX_BUFFER_SIZE),
          _parser(_rx_cir_buffer), _eventGroup("comm"), _rxWaitHandle(_eventGroup),
          _txWaitHandle(_eventGroup), _focCtrl(focCtrl), _motor(motor) {
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
    void startRx();
    void startTx();

    Buffer8 doTxWork();

    void doRxWork();

   private:
    Buffer8 _doMonitor();

    Buffer8 _doGetPid(PidType type);

    void _doSetPid(CommandType type, float p, float i, float d);
    void _doStop();
    ;
    void _doCalibrate();
    ;
    void _doOpenLoop(float ud, float uq);
    ;
    void _doCurrentCloseLoop(float id, float iq);
    ;
    void _doSpeedLoop(float ref);
    ;
    void _doPositionLoop(float ref);

    void _resetUart();

   private:
    constexpr static MessageLengthSchemaDefinition definitions[8] = {
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
        },
        {
            .command = {static_cast<uint8_t>(CommandType::MONITOR_RESP)},
            .length{
                .mode = MESSAGE_LENGTH_SCHEMA_MODE::DYNAMIC_LENGTH,
                .dynamic{
                    .lengthSize = MESSAGE_SCHEMA_SIZE::BIT8,
                    .endian     = MESSAGE_SCHEMA_LENGTH_ENDIAN::BIG,
                },
            },
        },
    };
    constexpr static MessageSchema schema = {

        .prefix        = {0x55, 0x50},
        .prefixSize    = 2,
        .commandSize   = MESSAGE_SCHEMA_SIZE::BIT8,
        .lengthSchemas = const_cast<MessageLengthSchemaDefinition*>(ControlProtocol::definitions),
        .lengthSchemaCount = 8,
        .defaultLength{
            .mode = MESSAGE_LENGTH_SCHEMA_MODE::FIXED_LENGTH,
            .fixed{.length = 0},
        },

        .crcSize = MESSAGE_SCHEMA_SIZE::BIT8,
        .crcRange =
            MESSAGE_SCHEMA_RANGE_CMD | MESSAGE_SCHEMA_RANGE_LENGTH | MESSAGE_SCHEMA_RANGE_CONTENT,
    };
};

}  // namespace motor
}  // namespace wibot

#endif  // WWMOTOR_APP_MOTOR2_COMM_CONTROLPROTOCOL_HPP_
