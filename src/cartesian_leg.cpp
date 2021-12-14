// BSD 3-Clause License
// Copyright (c) 2021 The Trustees of the University of Pennsylvania. All Rights Reserved
// Authors:
// Shane Rozen-Levy <srozen01@seas.upenn.edu>


#include "kodlab_mjbots_sdk/cartesian_leg.h"
#include <cmath>
namespace kodlab {
CartesianLeg::CartesianLeg(float l1, float l2) : l1_(l1), l2_(l2) {}

void CartesianLeg::FK(const std::vector<float> &angles, float &z, float &x) const {
  z = l1_ * cosf(angles[0]) + l2_ * cosf(angles[0] + angles[1]);
  x = l1_ * sinf(angles[0]) + l2_ * sinf(angles[0] + angles[1]);
}

std::vector<float> CartesianLeg::InverseDynamics(const std::vector<float> &angles,
                                                 float z_effort,
                                                 float x_effort) const {
  float tau0 = (-l1_ * sinf(angles[0]) - l2_ * sinf(angles[0] + angles[1])) * z_effort +
      (l1_ * cosf(angles[0]) + l2_ * cosf(angles[0] + angles[1])) * x_effort;
  float tau1 = (-l2_ * sinf(angles[0] + angles[1])) * z_effort + (l2_ * cosf(angles[0] + angles[1])) * x_effort;
  return {tau0, tau1};
}

void CartesianLeg::FkVel(const std::vector<float> &angles,
                         const std::vector<float> &d_angles,
                         float &d_z,
                         float &d_x) const {
  d_z = (-l1_ * sinf(angles[0]) - l2_ * sinf(angles[0] + angles[1])) * d_angles[0]
      + (-l2_ * sinf(angles[0] + angles[1])) * d_angles[1];
  d_x = (l1_ * cosf(angles[0]) + l2_ * cosf(angles[0] + angles[1])) * d_angles[0]
      + (l2_ * cosf(angles[0] + angles[1])) * d_angles[1];
}
} // namespace kodlab