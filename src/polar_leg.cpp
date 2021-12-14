// BSD 3-Clause License
// Copyright (c) 2021 The Trustees of the University of Pennsylvania. All Rights Reserved
// Authors:
// Shane Rozen-Levy <srozen01@seas.upenn.edu>


#include "kodlab_mjbots_sdk/polar_leg.h"
#include <math.h>
namespace kodlab {
PolarLeg::PolarLeg(float l1, float l2) : l1_(l1), l2_(l2) {}

void PolarLeg::FK(const std::vector<float> &angles, float &r, float &theta) const {
  float denom = l1_ * l1_ + l2_ * l2_ + 2 * l1_ * l2_ * cosf(angles[1]);

  theta = -atan2f(-l1_ * sinf(angles[0]) - l2_ * sinf(angles[1] + angles[0]),
                  l1_ * cosf(angles[0]) + l2_ * cosf(angles[1] + angles[0]));
  r = sqrtf(denom);
}

std::vector<float> PolarLeg::InverseDynamics(const std::vector<float> &angles,
                                             const float r_effort,
                                             const float theta_effort) const {
  float denom = l1_ * l1_ + l2_ * l2_ + 2 * l1_ * l2_ * cosf(angles[1]);
  float tau0 = theta_effort;
  float tau1 = -(l1_ * l2_ * sinf(angles[1]) / sqrtf(denom)) * r_effort
      + (l2_ * l2_ + l1_ * l2_ * cosf(angles[1])) / denom * theta_effort;
  return {tau0, tau1};
}

void PolarLeg::FkVel(const std::vector<float> &angles,
                     const std::vector<float> &d_angles,
                     float &d_r,
                     float &d_theta) const {
  float denom = l1_ * l1_ + l2_ * l2_ + 2 * l1_ * l2_ * cosf(angles[1]);
  d_r = -(l1_ * l2_ * sinf(angles[1]) / sqrtf(denom)) * d_angles[1];
  d_theta = d_angles[0] + (l2_ * l2_ + l1_ * l2_ * cosf(angles[1])) / denom * d_angles[1];
}
} // namespace kodlab