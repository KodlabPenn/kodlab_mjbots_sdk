//
// Created by shane on 9/29/21.
//
#include <math.h>

#pragma once
struct PhysicalParams{
  float r0 = 0.1658; // leg rest length
  float body_mass = 3; // body mass
  float tail_mass = 0.3; // tail mass
  float k = 4000; // spring constant
  float m = body_mass + tail_mass; // total mass
  float omega_v = sqrt(k/m); // natural frequency
  float tail_N = 19.0/17.0; // raw tail gear ratio
  float encoder_offset = .789;
  float femur_length = 0.09028;
  float toe_length = 0.039;
};


