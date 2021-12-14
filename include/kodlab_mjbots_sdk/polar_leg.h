// BSD 3-Clause License
// Copyright (c) 2021 The Trustees of the University of Pennsylvania. All Rights Reserved
// Authors:
// Shane Rozen-Levy <srozen01@seas.upenn.edu>


#pragma once

#include <vector>

/// Implementation of a polar leg. Assumes theta and the joint angles are in the same direction
/// Assumes 0 = leg straight down
class PolarLeg {
 public:
  PolarLeg(float l1, float l2);


  void FK(const std::vector<float>& angles, float& r, float& theta) const;

  std::vector<float> InverseDynamics(const std::vector<float>& angles, const float r_effort, const float theta_effort) const;

  void FkVel(const std::vector<float>& angles, const std::vector<float>& d_angles, float &d_r, float & d_theta) const;

 private:
  float l1_;
  float l2_;
};

