//
// Created by shane on 11/10/21.
//

#include "kodlab_mjbots_sdk/soft_start.h"

void Soft_Start::constrain(std::vector<float>& values, float min_val, float max_val){
  for(auto& value : values){
    value = fmin(fmax(value, min_val), max_val);
  }
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
