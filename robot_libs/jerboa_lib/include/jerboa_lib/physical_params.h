//
// Created by shane on 9/29/21.
//
#include <math.h>

#include <cmath>

#pragma once
struct PhysicalParams{
  float r0 = 0.166183; // leg rest length
  const float body_mass = 1.835+0.3; // body mass
  const float tail_mass = 0.338-.118+0.064+0.055; // tail mass
  const float k = 1607; // spring constant
  const float m = body_mass + tail_mass; // total mass
  const float omega_v = std::sqrt(k/m); // natural frequency
  const float tail_N = 19.0/17.0; // raw tail gear ratio
  const float encoder_offset = 0;
  const float femur_length = 0.09028;
  const float toe_length = 0.039;
  const float tail_length = 0.4;
};


