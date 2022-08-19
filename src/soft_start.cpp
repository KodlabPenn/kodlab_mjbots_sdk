// BSD 3-Clause License
// Copyright (c) 2021 The Trustees of the University of Pennsylvania. All Rights Reserved
// Authors:
// Shane Rozen-Levy <srozen01@seas.upenn.edu>
// J. Diego Caporale <jdcap@seas.upenn.edu>


#include "kodlab_mjbots_sdk/soft_start.h"

namespace kodlab {

void SoftStart::Constrain(std::vector<float> &values,
                          float min_val,
                          float max_val) {
  for (auto &value : values) {
    value = fmin(fmax(value, min_val), max_val);
  }
}

float SoftStart::Constrain(float values, float min_val, float max_val) {
  return fmin(fmax(values, min_val), max_val);
}

void SoftStart::ConstrainTorques(std::vector<float> &torques,
                                 float time_since_start_ms) {
  if (time_since_start_ms > duration_ms_) {
    Constrain(torques, -max_torque_, max_torque_);
  } else {
    float max_val = time_since_start_ms * slope_;
    Constrain(torques, -max_val, max_val);
  }

}

SoftStart::SoftStart(float max_torque, float duration_ms)
    : max_torque_(fabs(max_torque)),
      duration_ms_(duration_ms),
      slope_(fabs(max_torque) / (fmax(duration_ms, 1.0))) {}

} // kodlab
