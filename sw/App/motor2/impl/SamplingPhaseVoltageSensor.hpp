//
// Created by zhouj on 2022/11/28.
//

#ifndef WWMOTOR_APP_MOTOR2_SAMPLINGPHASEVOLTAGESENSOR_HPP_
#define WWMOTOR_APP_MOTOR2_SAMPLINGPHASEVOLTAGESENSOR_HPP_

#include "motor2/base.hpp"
#include "motor2/PhaseVoltageSensor.hpp"
#include "LinearValueMapper.hpp"
#include "lp.hpp"
#include "DataSource.hpp"
namespace wibot::motor {
using namespace wibot::accessor;
using namespace wibot::control;
struct SamplingPhaseVoltageSensorConfig {
    DataSource* u_a;
    DataSource* u_b;
    DataSource* u_c;

    float cutoff_freq;  // 务必大于最大电频率.
    float sample_time;

    float u_pu;
};

class SamplingPhaseVoltageSensor : public PhaseVoltageSensor,
                                   public Configurable<SamplingPhaseVoltageSensorConfig> {
   public:
    Result apply_config() override;

    void u_abc_get(wibot::motor::Motor& motor, Vector3f& u_abc) override;

   private:
    LinearValueMapper       _a_mapper;
    LinearValueMapper       _b_mapper;
    LinearValueMapper       _c_mapper;
    FirstOrderLowPassFilter _a_filter;
    FirstOrderLowPassFilter _b_filter;
    FirstOrderLowPassFilter _c_filter;
};

}  // namespace wibot::motor

#endif  // WWMOTOR_APP_MOTOR2_SAMPLINGPHASEVOLTAGESENSOR_HPP_
