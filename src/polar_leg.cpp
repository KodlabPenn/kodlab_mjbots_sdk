// BSD 3-Clause License
// Copyright (c) 2021 The Trustees of the University of Pennsylvania. All Rights Reserved
// Authors:
// Shane Rozen-Levy <srozen01@seas.upenn.edu>


#include "kodlab_mjbots_sdk/polar_leg.h"
#include <math.h>
Polar_Leg::Polar_Leg(float l1, float l2):m_l1(l1),m_l2(l2) {}


void Polar_Leg::fk(const std::vector<float> &angles, float &r, float &theta) const {
  float denom = m_l1*m_l1 + m_l2 * m_l2 + 2 * m_l1 * m_l2 * cosf(angles[1]);

  theta = -atan2f(-m_l1 * sinf(angles[0]) - m_l2 * sinf(angles[1]+angles[0]),
                  m_l1 * cosf(angles[0]) + m_l2 * cosf(angles[1]+angles[0]));
  r = sqrtf(denom);
}

std::vector<float> Polar_Leg::inverse_dynamics(const std::vector<float> &angles,
                                               const float r_effort,
                                               const float theta_effort) const {
  float denom = m_l1*m_l1 + m_l2 * m_l2 + 2 * m_l1 * m_l2 * cosf(angles[1]);
  float tau0 = theta_effort;
  float tau1 = -(m_l1*m_l2*sinf(angles[1]) /sqrtf(denom)) * r_effort + (m_l2 * m_l2 + m_l1 * m_l2 * cosf(angles[1]))/denom * theta_effort;
  return {tau0, tau1};
}

void Polar_Leg::fk_vel(const std::vector<float> &angles,
                       const std::vector<float> &d_angles,
                       float &d_r,
                       float &d_theta) const {
  float denom = m_l1*m_l1 + m_l2 * m_l2 + 2 * m_l1 * m_l2 * cosf(angles[1]);
  d_r =-(m_l1*m_l2*sinf(angles[1]) /sqrtf(denom))* d_angles[1];
  d_theta = d_angles[0] + (m_l2 * m_l2 + m_l1 * m_l2 * cosf(angles[1]))/denom * d_angles[1];
}
