// BSD 3-Clause License
// Copyright (c) 2021 The Trustees of the University of Pennsylvania. All Rights Reserved
// Authors:
// Shane Rozen-Levy <srozen01@seas.upenn.edu>

#include <cmath>

#include "kodlab_mjbots_sdk/mjbots_control_loop.h"
#include "TVHLog.hpp"
#include "TVHGains.hpp"
#include "jerboa_lib/tvh.h"
#include "kodlab_mjbots_sdk/PID.h"

class Hop : public kodlab::mjbots::MjbotsControlLoop<TVHLog, TVHGains> {
  using MjbotsControlLoop::MjbotsControlLoop;
  void CalcTorques() override {
    tvh_.UpdateState(robot_, time_now_);

    if(tvh_.GetTailAngle() > safety_tail_hard_max_ or tvh_.GetTailAngle() < safety_tail_hard_min_){
      tvh_.Disable();
    }
    float tail_effort = 0;
    switch (tvh_.GetMode()){
      case HybridMode::UNINITIALIZED:{
        break;
      }

      case HybridMode::STANCE:{
//        float av = sqrt(pow(tvh_.GetLegCompression() * tvh_.params_.omega_v, 2) + pow(tvh_.GetLegSpeed(), 2));
//        tail_effort = - kv_ * tvh_.GetLegSpeed()/av;
//        tail_effort += tail_ffwd_gain_* tvh_.params_.tail_length * tvh_.params_.tail_mass * std::cos(tvh_.GetTailAngle());
//        tail_pos_loop_.reset();

        tail_pos_loop_.calc_effort(0, tvh_.GetTailAngle(), tvh_.GetTailSpeed(), tail_effort);
        tail_effort += tail_ffwd_gain_* tvh_.params_.tail_length * tvh_.params_.tail_mass * std::cos(tvh_.GetTailAngle());
        break;
      }

      case HybridMode::FLIGHT:{
        tail_pos_loop_.calc_effort(0, tvh_.GetTailAngle(), tvh_.GetTailSpeed(), tail_effort);
        tail_effort += tail_ffwd_gain_* tvh_.params_.tail_length * tvh_.params_.tail_mass * std::cos(tvh_.GetTailAngle());
        break;
      }
      default:
        break;
    }

    if (tail_effort > 0 and tvh_.GetTailAngle() > safety_tail_soft_max_){
      tail_effort = 0;
    } else if (tail_effort < 0 and tvh_.GetTailAngle() < safety_tail_soft_min_){
      tail_effort = 0;
    }
    tvh_.SetEffort(robot_,tail_effort);
  }

  void PrepareLog() override {
    log_data_.tail_angle = tvh_.GetTailAngle();
    log_data_.tail_speed = tvh_.GetTailSpeed();
    log_data_.leg_encoder = robot_->GetJointPositions()[1];
    log_data_.leg_encoder_speed = robot_->GetJointVelocities()[1];
    log_data_.leg_length = tvh_.GetLegLength();
    log_data_.leg_velocity = tvh_.GetLegSpeed();
    log_data_.hybrid_mode = tvh_.GetMode();
    log_data_.torque_cmd = robot_->GetJointTorqueCmd()[0];
    std::cout<<log_data_.torque_cmd<<std::endl;
  }

  void ProcessInput() override {
    std::cout << "Response received" << std::endl;
    kv_ = lcm_sub_.data_.kv;
    tail_pos_loop_ = PID(lcm_sub_.data_.tail_kp, lcm_sub_.data_.tail_ki, lcm_sub_.data_.tail_kd);
    tail_ffwd_gain_ = lcm_sub_.data_.tail_stance_ffwd;
  }

  TVH tvh_;
 private:
  float safety_tail_soft_max_ = 1.5;
  float safety_tail_hard_max_ = 1.6;
  float safety_tail_soft_min_ = -0.584;
  float safety_tail_hard_min_ = -0.94;
  float kv_ = 0;
  float tail_ffwd_gain_ = 0;

  PID tail_pos_loop_ = PID(26, 0.000, 1.5);

};

int main(int argc, char **argv) {
  kodlab::mjbots::ControlLoopOptions options;
  // Define the motors in the robot
  options.motor_list.emplace_back(2, 1, 1, 2.89074*19/17);
  options.encoder_list.emplace_back(1, -1, 4.60507, 0.5, 0.5);

  options.log_channel_name = "jerboa_data";
  options.input_channel_name = "jerboa_gains";

  options.frequency = 1000;
  options.realtime_params.main_cpu = 3;
  options.realtime_params.can_cpu  = 2;

  options.max_torque = 4;
  options.soft_start_duration = 4000;

  // Create control loop
  Hop control_loop(options);
  // Starts the loop, and then join it
  control_loop.Start();
  control_loop.Join();
  return 0;
}
