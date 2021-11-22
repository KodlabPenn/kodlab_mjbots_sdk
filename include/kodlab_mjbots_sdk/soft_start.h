//
// Created by shane on 11/10/21.
//

#pragma once
#include <vector>
#include <math.h>

class Soft_Start {
 private:
  float m_slope = 0;
  float m_max_torque = 100;
  int m_duration = 0;

 public:
  void constrainTorques(std::vector<float>& torques, int count);

  static void constrain(std::vector<float>& values, float min_val, float max_val);

  static float constrain(float values, float min_val, float max_val);

  Soft_Start(float max_torque, int duration);
};

