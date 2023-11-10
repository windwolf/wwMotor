//
// Created by zhouj on 2022/11/28.
//

#include "SamplingPhaseVoltageSensor.hpp"

namespace wibot::motor {
Result SamplingPhaseVoltageSensor::apply_config() {
    Result rst;
    if (config.u_a == nullptr || config.u_b == nullptr || config.u_c == nullptr) {
        return Result::kInvalidParameter;
    }
    _a_mapper.config.zero_offset    = 0;
    _a_mapper.config.value_per_unit = config.u_pu;
    rst                             = _a_mapper.apply_config();
    if (rst != Result::OK) {
        return rst;
    }
    _b_mapper.config.zero_offset    = 0;
    _b_mapper.config.value_per_unit = config.u_pu;
    rst                             = _b_mapper.apply_config();
    if (rst != Result::OK) {
        return rst;
    }
    _c_mapper.config.zero_offset    = 0;
    _c_mapper.config.value_per_unit = config.u_pu;
    rst                             = _c_mapper.apply_config();
    if (rst != Result::OK) {
        return rst;
    }
    _a_filter.config.cutoff_freq = config.cutoff_freq;
    _a_filter.config.sample_time = config.sample_time;
    rst                          = _a_filter.apply_config();
    if (rst != Result::OK) {
        return rst;
    }
    _b_filter.config.cutoff_freq = config.cutoff_freq;
    _b_filter.config.sample_time = config.sample_time;
    rst                          = _b_filter.apply_config();
    if (rst != Result::OK) {
        return rst;
    }
    _c_filter.config.cutoff_freq = config.cutoff_freq;
    _c_filter.config.sample_time = config.sample_time;
    return _c_filter.apply_config();
}

void SamplingPhaseVoltageSensor::u_abc_get(Motor& motor, Vector3f& u_abc) {
    Vector3f u;
    u.v1     = _a_mapper.value_get(config.u_a->get_data());
    u.v2     = _b_mapper.value_get(config.u_b->get_data());
    u.v3     = _c_mapper.value_get(config.u_c->get_data());
    u_abc.v1 = _a_filter.filter(u_abc.v1);
    u_abc.v2 = _b_filter.filter(u_abc.v2);
    u_abc.v3 = _c_filter.filter(u_abc.v3);
}
}  // namespace wibot::motor
