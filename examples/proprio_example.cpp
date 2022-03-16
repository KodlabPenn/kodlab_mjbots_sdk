// BSD 3-Clause License
// Copyright (c) 2021 The Trustees of the University of Pennsylvania. All Rights Reserved
// Authors:
// J. Diego Caporale <jdcap@seas.upenn.edu>

/* Basic example script demonstrating how to use the mjbots_control_loop to 2 motors. The functions to implement are
 * CalcTorques and PrepareLog. In this example we send a torque cmd of all zeros and log the motor information.
 */

#include "kodlab_mjbots_sdk/mjbots_control_loop.h"
#include "kodlab_mjbots_sdk/jointMoteus.h"
#include "ManyMotorLog.hpp"
#include "kodlab_mjbots_sdk/lcm_subscriber.h"
#include <sys/mman.h>
#include <limits>
#include <Eigen/Core>
#include <Eigen/Dense>

class Spin_Joint : public kodlab::mjbots::MjbotsControlLoop<ManyMotorLog> {
  using MjbotsControlLoop::MjbotsControlLoop;
  void CalcTorques() override {

    float omega_des = 0.3; //velocity in rads per sec
    Eigen::VectorXf omega = Eigen::VectorXf::Ones(num_motors_)*omega_des; //radians
 
    Eigen::VectorXf positions  = Eigen::Map<Eigen::VectorXf,Eigen::Unaligned> ( robot_->GetJointPositions().data(), num_motors_);
    Eigen::VectorXf velocities = Eigen::Map<Eigen::VectorXf,Eigen::Unaligned> (robot_->GetJointVelocities().data(), num_motors_);

    static Eigen::VectorXf  phase = positions; //radians
    static Eigen::VectorXf    dir = Eigen::VectorXf::Ones(num_motors_); //radians
    static Eigen::VectorXf maxPos = Eigen::VectorXf::Ones(num_motors_) * ( -std::numeric_limits<float>::infinity() );
    static Eigen::VectorXf minPos = Eigen::VectorXf::Ones(num_motors_) * (  std::numeric_limits<float>::infinity() );

    phase.array() += dir.array()*(omega.array() / frequency_) ;
    
    
    
    auto switchBool =  (phase.array()-positions.array()).abs() < 0.1;
    dir = (switchBool).select(dir,-dir);
    phase = (switchBool).select(phase,positions);

    maxPos = (phase.array() > maxPos.array()).select(phase, maxPos);
    minPos = (phase.array() < minPos.array()).select(phase, minPos);
    
    std::vector<float> torques(num_motors_, 0);


    
    Eigen::VectorXf tau = 100*(phase-positions) + 1*(omega-velocities);

    std::cout<<"*********\n";
    std::cout<<maxPos.transpose()<<"\n";
    std::cout<<minPos.transpose()<<"\n";
    std::cout<<"*********"<<std::endl;

    Eigen::VectorXf::Map(&torques[0], num_motors_) = tau;

    robot_->SetTorques(torques);   
  }

  void PrepareLog() override {
    for (int servo = 0; servo < num_motors_; servo++) {
      log_data_.positions[servo]  = robot_->GetJointPositions()[servo];
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
  // joints.emplace_back(4, 100, 1,  1.7785, 1, 1);
  // joints.emplace_back(4, 101, 1, -2.966, 1, 5.0/3.0);
  // joints.emplace_back(4, 108, 1,  -0.4674585, 1, 1);
  joints.emplace_back(4, 101, 1,  0, 1, 5.0/3.0);
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
