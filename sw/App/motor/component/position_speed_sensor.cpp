#include "position_speed_sensor.hpp"
#include "os.hpp"
namespace wwMotor
{

#define POS_BUF_GET() _config.encoder_buffer.data[_config.position_idx]

void EncoderPositionSpeedSensor::calibrate_begin()
{
    for (size_t i = 0; i < ENCODER_BUFFER_SIZE; i++)
    {
        _buf_pos[i] = 0;
    }
    _buf_idx = 0;
};

void EncoderPositionSpeedSensor::calibrate()
{
    _buf_pos[_buf_idx] = POS_BUF_GET();
    _buf_idx = (_buf_idx + 1) % ENCODER_BUFFER_SIZE;
};

void EncoderPositionSpeedSensor::calibrate_end()
{
    uint32_t sum = 0;
    uint32_t var = 0;
    for (size_t i = 0; i < ENCODER_BUFFER_SIZE; i++)
    {
        sum += _buf_pos[i];
    }
    _zero_position = sum / ENCODER_BUFFER_SIZE;
    for (size_t i = 0; i < ENCODER_BUFFER_SIZE; i++)
    {
        var += (_buf_pos[i] - _zero_position) * (_buf_pos[i] - _zero_position);
    }
    _var = var / ENCODER_BUFFER_SIZE;
};

void EncoderPositionSpeedSensor::init()
{
    Scalar v = (Scalar)(POS_BUF_GET() - _zero_position);
    for (size_t i = 0; i < ENCODER_BUFFER_SIZE; i++)
    {
        _buf_pos[i] = v;
    }
    _buf_idx = 0;

    _scale_factor = _2PI / (Scalar)_config.full_scalar;
    _speed_factor =
        _2PI / (Scalar)_config.full_scalar / _config.sample_time / (float)ENCODER_BUFFER_SIZE;
    _half_scale = _config.full_scalar / 2;
};

Vector4f EncoderPositionSpeedSensor::speed_position_get()
{
    uint16_t raw_pos = POS_BUF_GET() - _zero_position;
    int32_t raw_pos_diff = (int32_t)raw_pos - (int32_t)_buf_pos[_buf_idx];

    // a->b: 1-2 dir=1; 1-9 dir=-1; 8-7 dir=-1; 8-1 dir=1;
    if (raw_pos_diff > 0)
    {
        if (raw_pos_diff < _half_scale)
        {
        }
        else
        {
            raw_pos_diff = raw_pos_diff - _config.full_scalar;
        }
    }
    else if (raw_pos_diff < 0)
    {
        if (raw_pos_diff > -_half_scale)
        {
        }
        else
        {
            raw_pos_diff = raw_pos_diff + _config.full_scalar;
        }
    }
    else
    {
    }

    Vector4f ret;
    ret.v1 = Math::circle_normalize((Scalar)raw_pos * _scale_factor);
    ret.v2 = Math::circle_normalize(ret.v1 * _config.pole_pairs);
    ret.v3 = (Scalar)raw_pos_diff * _speed_factor;
    ret.v4 = ret.v3 * _config.pole_pairs;

    _buf_pos[_buf_idx] = raw_pos;
    _buf_idx++;

    return ret;
};

} // namespace wwMotor
