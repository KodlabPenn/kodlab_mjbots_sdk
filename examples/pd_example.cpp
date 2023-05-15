// BSD 3-Clause License
// Copyright (c) 2021 The Trustees of the University of Pennsylvania. All Rights Reserved
// Authors:
// Shane Rozen-Levy <srozen01@seas.upenn.edu>, J. Diego Caporale <jdcap@seas.upenn.edu>

/* Basic example script demonstrating how to use the mjbots_control_loop to 2 joints. The functions to implement are
 * Update and PrepareLog. In this example we send a torque cmd of all zeros and log the motor information.
 */

#include "kodlab_mjbots_sdk/mjbots_control_loop.h"
#include "kodlab_mjbots_sdk/joint_moteus.h"
#include "ManyMotorLog.hpp"
#include "kodlab_mjbots_sdk/lcm_subscriber.h"
#include "kodlab_mjbots_sdk/log.h"  // provides console logging macros
#include <sys/mman.h>
#include <Eigen/Core>
#include <Eigen/Dense>

class Spin_Joint : public kodlab::mjbots::MjbotsControlLoop<ManyMotorLog,VoidLcm>
  {
  using MjbotsControlLoop::MjbotsControlLoop;
  void Update() override {
    std::vector<float> positions=robot_->GetJointPositions();
    std::vector<float> vels=robot_->GetJointVelocities();
    float ctrl_torque=1.5*(-vels[0]-50.0*positions[0]);

    std::vector<float> torques(num_joints_, 0);
    torques[0]=ctrl_torque;
    robot_->SetTorques(torques);


  }

  void PrepareLog() override {
    // Populate log message with data from current control loop cycle
    for (int servo = 0; servo < num_joints_; servo++) {
      log_data_->positions[servo] = robot_->GetJointPositions()[servo];
      log_data_->velocities[servo] = robot_->GetJointVelocities()[servo];
      log_data_->modes[servo] = -1;
      log_data_->torques[servo] = robot_->GetJointTorqueCmd()[servo];
    }

    // Fill remaining log message fields with zeros
    for (int servo = num_joints_; servo < 13; servo++) {
      log_data_->positions[servo] = 0;
      log_data_->velocities[servo] = 0;
      log_data_->modes[servo] = 0;
      log_data_->torques[servo] = 0;
    }
  }
};

int main(int argc, char **argv) {
  //Setup joints
  std::vector<kodlab::mjbots::JointMoteus> joints;
  joints.emplace_back(100,
                      4,
                      1,
                      0,
                      1,
                      100,
                      -std::numeric_limits<float>::infinity(),
                      std::numeric_limits<float>::infinity(),
                      10000);


  // Define robot options
  kodlab::mjbots::ControlLoopOptions options;
  options.log_channel_name = "motor_data";
  options.frequency = 1000;
  options.realtime_params.main_cpu = 1;
  options.realtime_params.can_cpu  = 0;

  options.imu_mounting_deg.yaw = 0;
  options.imu_mounting_deg.roll = 0;
  options.imu_mounting_deg.pitch = 180;
  options.attitude_rate_hz = 1000;
  options.xml_model_path="../../model/invertedPendulum.xml";
  options.interface="simulation_interface";
  // Create control loop
  LOG_INFO("Constructing Spin_Joint with %zu joints.", joints.size());
  Spin_Joint control_loop(std::move(joints), options);
  // Starts the loop, and then join it
  control_loop.Start();
  control_loop.Join();
  return 0;
}
