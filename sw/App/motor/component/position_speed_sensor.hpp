#ifndef __wwMotor_POSITION_SPEED_SENSOR_HPP__
#define __wwMotor_POSITION_SPEED_SENSOR_HPP__

#include "motor/base.hpp"
#include "buffer.hpp"

namespace wwMotor
{
using namespace ww;

#define ENCODER_BUFFER_SIZE 16
class EncoderPositionSpeedSensor
{

  public:
    struct Config
    {
        Buffer16 encoder_buffer;
        uint8_t position_idx;
        Scalar sample_time;
        Scalar full_scalar;
        uint8_t pole_pairs;
    };

  public:
    EncoderPositionSpeedSensor(Config &&config) : _config(config){};
    EncoderPositionSpeedSensor(Config &config) : _config(config){};
    void calibrate_begin();
    void calibrate();
    void calibrate_end();

    void init();

    /**
     * @brief
     *
     * @return Vector4f
     * v1: mechanical position: 0~2PI
     * v2: electrical position: 0~2PI
     * v3: mechanical speed: 0~2PI/s
     * v4: electrical speed: 0~2PI/s
     */
    Vector4f speed_position_get();

  private:
    uint16_t _buf_pos[ENCODER_BUFFER_SIZE];
    // Scalar _buf_spd[ENCODER_BUFFER_SIZE];
    uint8_t _buf_idx;

    Config _config;
    uint16_t _zero_position;
    uint16_t _var;

    Scalar _speed_factor; // _2PI / (Scalar)_config.full_scalar / _config.sample_time /
                          // (float)ENCODER_BUFFER_SIZE;
    Scalar _scale_factor; // _2PI / (Scalar)_config.full_scalar
    uint16_t _half_scale; // _config.full_scalar / 2
};

} // namespace wwMotor

#endif // __wwMotor_POSITION_SPEED_SENSOR_HPP__