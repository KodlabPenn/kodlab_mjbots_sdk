//
// Created by shane on 11/5/21.
//

#pragma once

#include <vector>

/// Implementation of a polar leg. Assumes theta and the joint angles are in the same direction
/// Assumes 0 = leg straight down
class Polar_Leg {
 public:
  Polar_Leg(float l1, float l2);


  void fk(const std::vector<float>& angles, float& r, float& theta) const;

  std::vector<float> inverse_dynamics(const std::vector<float>& angles, const float r_effort, const float theta_effort) const;

  void fk_vel(const std::vector<float>& angles, const std::vector<float>& d_angles, float &d_r, float & d_theta) const;

 private:
  float m_l1;
  float m_l2;
};

