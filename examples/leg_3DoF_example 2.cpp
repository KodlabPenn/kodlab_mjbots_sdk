// BSD 3-Clause License
// Copyright (c) 2021 The Trustees of the University of Pennsylvania. All Rights Reserved
// Authors:
// J. Diego Caporale <jdcap@seas.upenn.edu>

/* Basic PD Control example script demonstrating how to use the mjbots_control_loop to 3 motors. The functions to implement are
 * Update and PrepareLog. In this example we send a PD control torques and log the motor information.
 * Also, an implementation that uses Eigen conversions.
 */

#include "kodlab_mjbots_sdk/mjbots_control_loop.h"
#include "kodlab_mjbots_sdk/joint_moteus.h"
#include "ManyMotorLog.hpp"
#include "kodlab_mjbots_sdk/lcm_subscriber.h"
#include "kodlab_mjbots_sdk/log.h"  // provides console logging macros
#include <sys/mman.h>
#include <limits>
#include <Eigen/Core>
#include <Eigen/Dense>

class Joints3DoF : public kodlab::mjbots::MjbotsControlLoop<ManyMotorLog> {
  using MjbotsControlLoop::MjbotsControlLoop;
  void Update() override {
    std::vector<float> torques(num_joints_, 0);

    if (num_joints_==3){
      Eigen::Vector3f leg_pos_des;
      leg_pos_des<<1-M_PI_2,2,0;
      Eigen::Vector3f positions  = Eigen::Map<Eigen::VectorXf,Eigen::Unaligned> (robot_->GetJointPositions().data(), num_joints_);
      Eigen::Vector3f velocities = Eigen::Map<Eigen::VectorXf,Eigen::Unaligned> (robot_->GetJointVelocities().data(), num_joints_);
      float kp = 100;
      float kd = 1;
      Eigen::VectorXf tau = kp*(leg_pos_des-positions) + kd*(Eigen::Vector3f::Zero()-velocities);

      Eigen::VectorXf::Map(&torques[0], num_joints_) = tau;
    }
    else{
      LOG_ERROR("Wrong number of motors");
    }

    robot_->SetTorques(torques);   
  }

  void PrepareLog() override {
    // Populate log message with data from current control loop cycle
    for (int servo = 0; servo < num_joints_; servo++) {
      log_data_->positions[servo]  = robot_->GetJointPositions()[servo];
      log_data_->velocities[servo] = robot_->GetJointVelocities()[servo];
      log_data_->modes[servo] = static_cast<int>(mjbots_interface_->GetJointModes()[servo]);
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
  joints.emplace_back(100, 4, 1, -1.3635165,    1,  1);
  joints.emplace_back(101, 4,-1,  2.688,  5.0/3.0,  1);
  joints.emplace_back(108, 4, 1, -0.4674585,    1,  1);

  // Define robot options
  kodlab::mjbots::ControlLoopOptions options;
  options.log_channel_name = "motor_data";
  options.frequency = 1000;
  options.realtime_params.main_cpu = 3;
  options.realtime_params.can_cpu  = 2;
  options.parallelize_control_loop = true; 

  // Create control loop
  Joints3DoF control_loop(std::move(joints), options);

  // Starts the loop, and then join it
  control_loop.Start();
  control_loop.Join();
  return 0;
}
