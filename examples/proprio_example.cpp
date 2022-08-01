// BSD 3-Clause License
// Copyright (c) 2021 The Trustees of the University of Pennsylvania. All Rights Reserved
// Authors:
// J. Diego Caporale <jdcap@seas.upenn.edu>

/* Example script running joints to their mechanical limits before switching direction. The functions to implement are
 * Update and PrepareLog. In this example we run the motor forward until its
 * position error is large enough to detect a obstacle, then switching directions
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

class ProprioJoints : public kodlab::mjbots::MjbotsControlLoop<ManyMotorLog> {
  using MjbotsControlLoop::MjbotsControlLoop;
  void Update() override {

    float omega_des = 0.3; //velocity in rads per sec
    Eigen::VectorXf omega = Eigen::VectorXf::Ones(num_motors_)*omega_des; //radians
 
    Eigen::VectorXf positions  = Eigen::Map<Eigen::VectorXf,Eigen::Unaligned> ( robot_->GetJointPositions().data(), num_motors_);
    Eigen::VectorXf velocities = Eigen::Map<Eigen::VectorXf,Eigen::Unaligned> (robot_->GetJointVelocities().data(), num_motors_);

    static Eigen::VectorXf  phase = positions; //radians
    static Eigen::VectorXf    dir = Eigen::VectorXf::Ones(num_motors_); //radians
    static Eigen::VectorXf max_pos = Eigen::VectorXf::Ones(num_motors_) * ( -std::numeric_limits<float>::infinity() );
    static Eigen::VectorXf min_pos = Eigen::VectorXf::Ones(num_motors_) * (  std::numeric_limits<float>::infinity() );

    phase.array() += dir.array()*(omega.array() / frequency_) ;
    
    // Detect collision
    float switch_threshold = 0.1;
    auto switch_bool =  (phase.array()-positions.array()).abs() < switch_threshold;
    dir = (switch_bool).select(dir,-dir);
    phase = (switch_bool).select(phase,positions);

    // Update limits
    max_pos = (phase.array() > max_pos.array()).select(phase, max_pos);
    min_pos = (phase.array() < min_pos.array()).select(phase, min_pos);
    
    //Calculate torques
    std::vector<float> torques(num_motors_, 0);
    Eigen::VectorXf tau = 100*(phase-positions) + 1*(omega-velocities);
    Eigen::VectorXf::Map(&torques[0], num_motors_) = tau;

    // Print limits
    LOG_DEBUG("Max. Limits:");
    std::cout << max_pos.transpose() << "\n";
    LOG_DEBUG("Min. Limits:");
    std::cout << min_pos.transpose() << "\n";

    robot_->SetTorques(torques);   
  }

  void PrepareLog() override {
    for (int servo = 0; servo < num_motors_; servo++) {
      log_data_.positions[servo]  = robot_->GetJointPositions()[servo];
      log_data_.velocities[servo] = robot_->GetJointVelocities()[servo];
      log_data_.modes[servo] = static_cast<int>(mjbots_interface_->GetJointModes()[servo]);
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

  //Setup joints
  std::vector<kodlab::mjbots::JointMoteus> joints;

  joints.emplace_back(108, 4, 1, -0.4674585, 1, 1);

  // Define robot options
  kodlab::mjbots::ControlLoopOptions options;
  options.log_channel_name = "motor_data";
  options.frequency = 1000;
  options.realtime_params.main_cpu = 3;
  options.realtime_params.can_cpu  = 2;
  
  // Create control loop
  ProprioJoints control_loop(std::move(joints), options);

  // Starts the loop, and then join it
  control_loop.Start();
  control_loop.Join();
  return 0;
}
