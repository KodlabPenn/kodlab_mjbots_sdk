//
// Created by shane on 11/5/21.
//

#include "kodlab_mjbots_sdk/cartesian_leg.h"
#include <math.h>
Cartesian_Leg::Cartesian_Leg(float l1, float l2):m_l1(l1),m_l2(l2) {}


void Cartesian_Leg::fk(const std::vector<float> &angles, float &z, float &x) const {
  z = m_l1 * cosf(angles[0]) + m_l2 * cosf(angles[0] + angles[1]);
  x = m_l1 * sinf(angles[0]) + m_l2 * sinf(angles[0] + angles[1]);
}

std::vector<float> Cartesian_Leg::inverse_dynamics(const std::vector<float> &angles,
                                               const float z_effort,
                                               const float x_effort) const {
  float tau0 = (-m_l1 * sinf(angles[0]) - m_l2 * sinf(angles[0] + angles[1])) * z_effort +
               ( m_l1 * cosf(angles[0]) + m_l2 * cosf(angles[0] + angles[1])) * x_effort;
  float tau1 = (-m_l2 * sinf(angles[0]+angles[1])) * z_effort + ( m_l2 * cosf(angles[0]+angles[1])) * x_effort;
  return {tau0, tau1};
}

void Cartesian_Leg::fk_vel(const std::vector<float> &angles,
                       const std::vector<float> &d_angles,
                       float &d_z,
                       float &d_x) const {
  d_z = (-m_l1 * sinf(angles[0]) - m_l2 * sinf(angles[0] + angles[1])) * d_angles[0] + (-m_l2 * sinf(angles[0]+angles[1])) * d_angles[1];
  d_x = ( m_l1 * cosf(angles[0]) + m_l2 * cosf(angles[0] + angles[1])) * d_angles[0] + ( m_l2 * cosf(angles[0]+angles[1])) * d_angles[1];
}
