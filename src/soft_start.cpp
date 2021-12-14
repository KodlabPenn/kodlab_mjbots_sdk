// BSD 3-Clause License
// Copyright (c) 2021 The Trustees of the University of Pennsylvania. All Rights Reserved
// Authors:
// Shane Rozen-Levy <srozen01@seas.upenn.edu>


#include "kodlab_mjbots_sdk/soft_start.h"

namespace kodlab {
void SoftStart::Constrain(std::vector<float> &values, float min_val, float max_val) {
  for (auto &value : values) {
    value = fmin(fmax(value, min_val), max_val);
  }
}

float SoftStart::Constrain(float values, float min_val, float max_val) {
  return fmin(fmax(values, min_val), max_val);
}

void SoftStart::ConstrainTorques(std::vector<float> &torques, int count) {
  if (count > duration_) {
    Constrain(torques, -max_torque_, max_torque_);
  } else {
    float max_val = count * slope_;
    Constrain(torques, -max_val, max_val);

  }

}
SoftStart::SoftStart(float max_torque, int duration)
    : max_torque_(fabs(max_torque)), duration_(duration), slope_(fabs(max_torque) / (fmax(duration, 1))) {}
} // namespace kodlab