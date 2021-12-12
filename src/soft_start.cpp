// BSD 3-Clause License
// Copyright (c) 2021 The Trustees of the University of Pennsylvania. All Rights Reserved
// Authors:
// Shane Rozen-Levy <srozen01@seas.upenn.edu>


#include "kodlab_mjbots_sdk/soft_start.h"

 void Soft_Start::constrain(std::vector<float>& values, float min_val, float max_val){
  for(auto& value : values){
    value = fmin(fmax(value, min_val), max_val);
  }
}

float Soft_Start::constrain(float values, float min_val, float max_val){
    return fmin(fmax(values, min_val), max_val);
}


void Soft_Start::constrainTorques(std::vector<float>& torques, int count) {
  if(count > m_duration){
    constrain(torques, -m_max_torque, m_max_torque);
  }
  else{
    float max_val = count * m_slope;
    constrain(torques, -max_val, max_val);

  }

}
Soft_Start::Soft_Start(float max_torque, int duration):m_max_torque(fabs(max_torque)),m_duration(duration),m_slope(fabs(max_torque)/(fmax(duration,1))) {}
