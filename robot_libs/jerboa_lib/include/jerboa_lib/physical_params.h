//
// Created by shane on 9/29/21.
//
#include <math.h>

#pragma once
struct PhysicalParams{
  const float r0 = 0.1658; // leg rest length
  const float body_mass = 1.835; // body mass
  const float tail_mass = 0.338; // tail mass
  const float k = 1607; // spring constant
  const float m = body_mass + tail_mass; // total mass
  const float omega_v = sqrt(k/m); // natural frequency
  const float tail_N = 19.0/17.0; // raw tail gear ratio
  const float encoder_offset = .789;
  const float femur_length = 0.09028;
  const float toe_length = 0.039;
};


