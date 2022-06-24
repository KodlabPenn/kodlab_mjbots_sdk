// BSD 3-Clause License
// Copyright (c) 2022 The Trustees of the University of Pennsylvania. All Rights Reserved
// Authors:
// Ethan Musser <emusser@seas.upenn.edu>

/* Basic example script demonstrating how to use the mjbots_control_loop to read 
 * attitude data.
 */

#include "kodlab_mjbots_sdk/mjbots_control_loop.h"
#include "kodlab_mjbots_sdk/joint_moteus.h"
#include "ManyMotorLog.hpp"
#include "kodlab_mjbots_sdk/lcm_subscriber.h"
#include <sys/mman.h>
#include <Eigen/Core>
#include <Eigen/Dense>

#include <iostream>
#include <iomanip>

class AttitudeExample : public kodlab::mjbots::MjbotsControlLoop<ManyMotorLog, VoidLcm, kodlab::Attitude<float>>
{
  using MjbotsControlLoop::MjbotsControlLoop;
  void CalcTorques() override
  {
    std::vector<float> torques(num_motors_, 0);
    robot_->SetTorques(torques);
    kodlab::Attitude<float> att = robot_->GetAttitude();
    kodlab::rotations::EulerAngles euler = kodlab::rotations::QuaternionToEulerAngles(att.attitude);
    Eigen::Matrix3<float> R = kodlab::rotations::QuaternionToRotationMatrix(att.attitude);
    std::cout << std::fixed << std::setprecision(2) << "=================================================" << "\r" << std::endl;
    std::cout << std::fixed << std::setprecision(2) << " qR: " << euler.roll      << "\t| qP: " << euler.pitch     << "\t| qY: " << euler.yaw       << std::endl;
    std::cout << std::fixed << std::setprecision(2) << " wx: " << att.ang_rate(0) << "\t| wy: " << att.ang_rate(1) << "\t| wz: " << att.ang_rate(0) << std::endl;
    std::cout << std::fixed << std::setprecision(2) << "-------------------------------------------------" << "\r" << std::endl;
    std::cout << std::fixed << std::setprecision(2) << "RotMat:" << std::endl;
    std::cout << std::fixed << std::setprecision(2) << R << std::endl;
  }

  void PrepareLog() override
  {
    for (int servo = 0; servo < num_motors_; servo++)
    {
      log_data_.positions[servo] = robot_->GetJointPositions()[servo];
      log_data_.velocities[servo] = robot_->GetJointVelocities()[servo];
      log_data_.modes[servo] = static_cast<int>(robot_->GetJointModes()[servo]);
      log_data_.torques[servo] = robot_->GetJointTorqueCmd()[servo];
    }
    for (int servo = num_motors_; servo < 13; servo++)
    {
      log_data_.positions[servo] = 0;
      log_data_.velocities[servo] = 0;
      log_data_.modes[servo] = 0;
      log_data_.torques[servo] = 0;
    }
  }
};

int main(int argc, char **argv)
{

  // Setup joints
  std::vector<kodlab::mjbots::JointMoteus> joints;
  joints.emplace_back(100, 4, 1, -1.3635165, 1, 1);
  joints.emplace_back(101, 4, -1, 2.688, 5.0 / 3.0, 1);
  joints.emplace_back(108, 4, 1, -0.4674585, 1, 1);

  // Define robot options
  kodlab::mjbots::ControlLoopOptions options;
  options.log_channel_name = "motor_data";
  options.frequency = 1000;
  options.realtime_params.main_cpu = 3;
  options.realtime_params.can_cpu = 2;

  options.imu_mounting_deg.roll = 0;
  options.imu_mounting_deg.pitch = 0;
  options.imu_mounting_deg.yaw = 180;
  options.attitude_rate_hz = 1000;

  // Create control loop
  AttitudeExample control_loop(joints, options);
  // Starts the loop, and then join it
  control_loop.Start();
  control_loop.Join();
  return 0;
}
