#include "pos_spd_observer.hpp"
#include "math.h"
#include "foc_math.hpp"

namespace wwMotor
{

void SmoPllPositionSpeedOboserver::init()
{
    a = exp(-1.0f * motor.rs / motor.ld * _config.sample_time);
    b = (1 - a) / motor.rs;
};
Vector2f SmoPllPositionSpeedOboserver::pos_spd_get(Vector3f uabc, Vector3f iabc)
{
    FocMath::abc2ab(iabc, _i);
    FocMath::abc2ab(uabc, _u);
    smo();
    pll();
    return _pos_spd;
};

void SmoPllPositionSpeedOboserver::smo()
{
    Vector2f i_err_now = _i_obs - _i;
    Vector2f zk = Math::sign(i_err_now) * _config.current_gain;
    _e_obs = (i_err_now - _i_err * a + _zk) * b + _e_obs;

    _i_obs = (_u - _e_obs) * b - zk + _i_obs * a;
    _zk = zk;
    _i_err = i_err_now;
};

void SmoPllPositionSpeedOboserver::pll()
{
    Scalar e_alpha = _filter.filter(_e_obs.v1);
    Scalar e_beta = _filter.filter(_e_obs.v2);

    Scalar sin, cos;
    Math::sincos(_pos_spd.v1, &sin, &cos);

    Scalar err = e_alpha * cos + e_beta * sin; // 下一步pid作用再0-sum上, 所以这里不用取负值了
    _pos_spd.v2 = _pid.update(0, err);
    _pos_spd.v1 += _pos_spd.v2 * _config.sample_time;
};

} // namespace wwMotor
