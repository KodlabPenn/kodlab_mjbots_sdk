// BSD 3-Clause License
// Copyright (c) 2021 The Trustees of the University of Pennsylvania. All Rights Reserved
// Authors:
// Shane Rozen-Levy <srozen01@seas.upenn.edu>

#include "kodlab_mjbots_sdk/mjbots_control_loop.h"
#include "TVHLog.hpp"
#include "kodlab_mjbots_sdk/lcm_subscriber.h"
#include "jerboa_lib/tvh.h"

class Hop : public kodlab::mjbots::MjbotsControlLoop<TVHLog> {
  using MjbotsControlLoop::MjbotsControlLoop;
  void CalcTorques() override {
    std::vector<float> torques(num_motors_, 0);
    robot_->SetTorques(torques);
  }

  void PrepareLog() override {
    log_data_.tail_angle = robot_->GetJointPositions()[0];
    log_data_.tail_speed = robot_->GetJointVelocities()[0];
    log_data_.leg_encoder = robot_->GetJointPositions()[1];
    log_data_.leg_encoder_speed = robot_->GetJointVelocities()[1];
    std::cout<<log_data_.leg_encoder<<std::endl;
  }
};

int main(int argc, char **argv) {
  kodlab::mjbots::ControlLoopOptions options;
  // Define the motors in the robot
  options.motor_list.emplace_back(2, 1, 1, 0.0025);
  options.encoder_list.emplace_back(1, 1, -4.60507, 0.5, 0.5);

  options.log_channel_name = "jerboa_data";

  options.frequency = 1000;
  options.realtime_params.main_cpu = 3;
  options.realtime_params.can_cpu  = 2;

  // Create control loop
  Hop control_loop(options);
  // Starts the loop, and then join it
  control_loop.Start();
  control_loop.Join();
  return 0;
}
