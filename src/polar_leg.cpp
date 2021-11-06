//
// Created by shane on 11/5/21.
//

#include "kodlab_mjbots_sdk/polar_leg.h"
#include <math.h>
Polar_Leg::Polar_Leg(float l1, float l2):m_l1(l1),m_l2(l2) {}


void Polar_Leg::fk(const std::vector<float> &angles, float &r, float &theta) const {
  theta = angles[0] + angles[1];
  r = m_l1 + m_l2 * cosf(angles[1]);
}

std::vector<float> Polar_Leg::inverse_dynamics(const std::vector<float> &angles,
                                               const float r_effort,
                                               const float theta_effort) const {
  float tau0 = theta_effort;
  float tau1 = theta_effort + -m_l2 * r_effort * sinf(angles[1]);
  return {tau0, tau1};
}

void Polar_Leg::fk_vel(const std::vector<float> &angles,
                       const std::vector<float> &d_angles,
                       float &d_r,
                       float &d_theta) const {
  d_r = -m_l2 * sinf(angles[1]) * d_angles[1];
  d_theta = d_angles[0] + d_angles[1];
}
