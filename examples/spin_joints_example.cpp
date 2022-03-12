// BSD 3-Clause License
// Copyright (c) 2021 The Trustees of the University of Pennsylvania. All Rights Reserved
// Authors:
// Shane Rozen-Levy <srozen01@seas.upenn.edu>, J. Diego Caporale <jdcap@seas.upenn.edu>

/* Basic example script demonstrating how to use the mjbots_control_loop to 2 motors. The functions to implement are
 * CalcTorques and PrepareLog. In this example we send a torque cmd of all zeros and log the motor information.
 */

#include "kodlab_mjbots_sdk/mjbots_control_loop.h"
#include "kodlab_mjbots_sdk/jointMoteus.h"
#include "ManyMotorLog.hpp"
#include "kodlab_mjbots_sdk/lcm_subscriber.h"
#include <sys/mman.h>

class Spin_Joint : public kodlab::mjbots::MjbotsControlLoop<ManyMotorLog> {
  using MjbotsControlLoop::MjbotsControlLoop;
  void CalcTorques() override {
    std::vector<float> torques(num_motors_, 0);
    robot_->SetTorques(torques);
  }

  void PrepareLog() override {
    for (int servo = 0; servo < num_motors_; servo++) {
      log_data_.positions[servo] = robot_->GetJointPositions()[servo];
      log_data_.velocities[servo] = robot_->GetJointVelocities()[servo];
      log_data_.modes[servo] = static_cast<int>(robot_->GetJointModes()[servo].get());
      log_data_.torques[servo] = robot_->GetJointTorqueCmd()[servo];
    }
    for (int servo = num_motors_; servo < 13; servo++) {
      log_data_.positions[servo] = 0;
      log_data_.velocities[servo] = 0;
      log_data_.modes[servo] = 0;
      log_data_.torques[servo] = 0;
    }
  }
};

int main(int argc, char **argv) {
  kodlab::mjbots::ControlLoopOptions options;
  std::vector<kodlab::JointMoteus> joints;
  joints.emplace_back(4, 10, 1, 0, 0, 1);
  joints.emplace_back(4, 11, 1, 0, 0, 1);
  // Define the motors in the robot
  options.log_channel_name = "motor_data";

  options.frequency = 1000;
  options.realtime_params.main_cpu = 3;
  options.realtime_params.can_cpu  = 2;

  // Create control loop
  Spin_Joint control_loop(joints, options);
  // Starts the loop, and then join it
  control_loop.Start();
  control_loop.Join();
  return 0;
}
