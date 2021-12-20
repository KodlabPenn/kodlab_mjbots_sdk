// BSD 3-Clause License
// Copyright (c) 2021 The Trustees of the University of Pennsylvania. All Rights Reserved
// Authors:
// Shane Rozen-Levy <srozen01@seas.upenn.edu>

/* Basic example script demonstrating how to use the mjbots_control_loop to 2 motors. The functions to implement are
 * CalcTorques and PrepareLog. In this example we send a torque cmd of all zeros and log the motor information.
 */

#include "kodlab_mjbots_sdk/mjbots_control_loop.h"
#include "ManyMotorLog.hpp"
#include "kodlab_mjbots_sdk/lcm_subscriber.h"
#include <sys/mman.h>

class Spin_Motor : public kodlab::mjbots::MjbotsControlLoop<ManyMotorLog> {
  using MjbotsControlLoop::MjbotsControlLoop;
  void CalcTorques() override {
    std::vector<float> torques(num_motors_, 0.0);
    robot_->SetTorques(torques);
  }

  void PrepareLog() override {
    for (int servo = 0; servo < num_motors_; servo++) {
      log_data_.positions[servo] = robot_->GetJointPositions()[servo];
      log_data_.velocities[servo] = robot_->GetJointVelocities()[servo];
      log_data_.modes[servo] = static_cast<int>(robot_->GetJointModes()[servo]);
      log_data_.torques[servo] = robot_->GetJointTorqueCmd()[servo];
    }
    for (int servo = num_motors_; servo < 13; servo++) {
      log_data_.positions[servo] = 0;
      log_data_.velocities[servo] = 0;
      log_data_.modes[servo] = 0;
      log_data_.torques[servo] = 0;
    }
    log_data_.cycle_duration = *robot_->cycle_duration_;
    log_data_.reply_duration = *robot_->reply_duration_;
    log_data_.send_duration  = *robot_->send_duration_;
    log_data_.child_cycle_duration = *robot_->child_cycle_duration_;
  }
};

int main(int argc, char **argv) {
  kodlab::mjbots::ControlLoopOptions options;
  // Define the motors in the robot
  options.motor_list.emplace_back(10, 3);
  options.motor_list.emplace_back(11, 3);
  options.motor_list.emplace_back(12, 3);

  options.motor_list.emplace_back(13, 1);
  options.motor_list.emplace_back(14, 1);
  options.motor_list.emplace_back(15, 1);

  options.motor_list.emplace_back(16, 4);
  options.motor_list.emplace_back(17, 4);
  options.motor_list.emplace_back(18, 4);

  options.motor_list.emplace_back(19, 2);
  options.motor_list.emplace_back(20, 2);
  options.motor_list.emplace_back(21, 2);
  options.motor_list.emplace_back(22, 2);

  options.log_channel_name = "motor_data";

  options.frequency = 800;
  options.realtime_params.main_cpu = 3;
  options.realtime_params.can_cpu  = 2;

  // Create control loop
  Spin_Motor control_loop(options);
  // Starts the loop, and then join it
  control_loop.Start();
  control_loop.Join();
  return 0;
}
