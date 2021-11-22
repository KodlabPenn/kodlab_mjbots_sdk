//
// Created by shane on 11/5/21.
//

#pragma once

#include <vector>

/// Implementation of a polar leg. Assumes theta and the joint angles are in the same direction
/// Assumes 0 = leg straight down
class Cartesian_Leg {
 public:
  Cartesian_Leg(float l1, float l2);


  void fk(const std::vector<float>& angles, float& z, float& x) const;

  std::vector<float> inverse_dynamics(const std::vector<float>& angles, const float z_effort, const float x_effort) const;

  void fk_vel(const std::vector<float>& angles, const std::vector<float>& d_angles, float &d_z, float & d_x) const;

 private:
  float m_l1;
  float m_l2;
};

