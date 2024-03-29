// BSD 3-Clause License
// Copyright (c) 2021 The Trustees of the University of Pennsylvania. All Rights Reserved
// Authors:
// Shane Rozen-Levy <srozen01@seas.upenn.edu>
// J. Diego Caporale <jdcap@seas.upenn.edu>


#include "kodlab_mjbots_sdk/soft_start.h"
#include "kodlab_mjbots_sdk/log.h"

namespace kodlab {

void torque_limiter::Constrain(std::vector<float> &values,
                          float min_val,
                          float max_val) {
  for (auto &value : values) {
    value = std::min(std::max(value, min_val), max_val);
  }
}

void torque_limiter::Constrain(float& value, float min_val, float max_val) {
  value = std::min(std::max(value, min_val), max_val);
}

void SoftStart::ConstrainTorques(std::vector<float> &torques) const {
  if(timer_initialized_){
    float time_since_start_ms = SoftStart::timer_.tac()/1000.0;
    if (time_since_start_ms > duration_ms_) {
      torque_limiter::Constrain(torques, -max_torque_, max_torque_);
    } else {
      float max_val = time_since_start_ms * slope_;
      torque_limiter::Constrain(torques, -max_val, max_val);
    }
  }else{
    LOG_ERROR("Soft start not initialized, constraining torque to zero");
    torque_limiter::Constrain(torques, -0, 0);
  }
}

void SoftStart::ConstrainTorque(float &torque) const {
  if(timer_initialized_) {
    float time_since_start_ms = SoftStart::timer_.tac() / 1000.0;
    if (time_since_start_ms > duration_ms_) {
      torque_limiter::Constrain(torque, -max_torque_, max_torque_);
    } else {
      float max_val = time_since_start_ms * slope_;
      torque_limiter::Constrain(torque, -max_val, max_val);
    }
  } else{
    LOG_ERROR("Soft start not initialized, constraining torque to zero");
    torque = 0;
  }
}

SoftStart::SoftStart(float max_torque, float duration_ms)
    : max_torque_(fabs(max_torque)),
      duration_ms_(duration_ms),
      slope_(fabs(max_torque) / (fmax(duration_ms, 1.0))) {}

bool SoftStart::timer_initialized_ = false;
real_time_tools::Timer SoftStart::timer_;

void SoftStart::InitializeTimer() {
  timer_.tic();
  timer_initialized_ = true;
}
} // kodlab
