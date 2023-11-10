//
// Created by zhouj on 2022/11/17.
//

#ifndef WWMOTOR_APP_MOTOR2_ABSOLUTEENCODERPOSITIONSPEEDSENSOR_HPP_
#define WWMOTOR_APP_MOTOR2_ABSOLUTEENCODERPOSITIONSPEEDSENSOR_HPP_

#include "motor2/PositionSpeedSensor.hpp"
#include "filter/lp.hpp"
#include "DataSource.hpp"
#include "mapper/piecewise-linear-value-mapper.hpp"

namespace wibot::motor {

enum class EncoderDirection {
    Forward = 1,
    Reverse = -1
};
#define ENCODER_MAPPER_POINT_COUNT 64

struct AbsoluteEncoderPositionSpeedSensorConfig {
    DataSource*      codex;
    uint32_t         resolution;  // 12bit = 4096
    uint8_t          pole_pairs;
    EncoderDirection direction = EncoderDirection::Forward;
    float            mech_pos_cutoff_freq;
    float            mech_speed_cutoff_freq;
    float            sample_time;

    float calibration_voltage;
};

class AbsoluteEncoderPositionSpeedSensor
    : public PositionSpeedSensor,
      public Configurable<AbsoluteEncoderPositionSpeedSensorConfig> {
   public:
    AbsoluteEncoderPositionSpeedSensor(){};
    Result setConfig(&config);

    /**
     * @brief get position and speed in electrical and mechanical domain.
     * speed is in rad/s, position is in rad.
     * @param motor
     * @param position
     * @param speed
     */
    void position_speed_get(Motor& motor, Vector2f& position, Vector2f& speed) override;

    float get_position_nowrap() const;

    /**
     * for test only
     * @return
     */
    float get_position_without_filter() const;

    /**
     * Calibrate the zero index of the encoder, and the linear mapping between encoder value and
     * mechanical angle.
     *
     * @note Make sure the motor controller is in CALIBERATION mode, and the innerloop and outerloop
     * task is Started, before calling this function.
     * @note During calibration, the motor will rotate at forward direction 1.5 rounds, and at
     * reverse direction 1.5 rounds.
     * @param motor
     */
    void calibrate(Motor& motor) override;

   private:
    float                                                  last_pos_m_raw_;
    int32_t                                                round_m_;
    float                                                  last_pos_m_;
    PiecewiseLinearValueMapper<ENCODER_MAPPER_POINT_COUNT> _mapper;
    FirstOrderLowPassFilter                                filter_pos_m_;
    FirstOrderLowPassFilter                                filter_pos_e_;
    FirstOrderLowPassFilter                                filter_spd_m_;
    FirstOrderLowPassFilter                                filter_spd_e_;

    float _2PI_RES;
    float _1_TS;
    float _PP_TS;
    void  calibrate_rotate(Motor& motor);
    void  rotate(Motor& motor, float start_pos, float travel, uint32_t step,
                 uint32_t step_delay) const;
    void  sample_data(const Motor& motor, uint32_t step);
};

}  // namespace wibot::motor

#endif  // WWMOTOR_APP_MOTOR2_ABSOLUTEENCODERPOSITIONSPEEDSENSOR_HPP_
