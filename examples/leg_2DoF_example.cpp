// BSD 3-Clause License
// Copyright (c) 2021 The Trustees of the University of Pennsylvania. All Rights Reserved
// Authors:
// Shane Rozen-Levy <srozen01@seas.upenn.edu>

/*
 * This example demonstrates how you can use the mjbots_control_loop and the cartesian leg to create a hopping robot
 */

#include "kodlab_mjbots_sdk/mjbots_control_loop.h"
#include "kodlab_mjbots_sdk/joint_moteus.h"
#include "LegGains.hpp"
#include "LegLog.hpp"
#include "kodlab_mjbots_sdk/lcm_subscriber.h"
#include "kodlab_mjbots_sdk/cartesian_leg.h"
#include "kodlab_mjbots_sdk/log.h"  // provides console logging macros

class Hopping : public kodlab::mjbots::MjbotsControlLoop<LegLog, LegGains> {
  using MjbotsControlLoop::MjbotsControlLoop;

  // We use Update as the main control loop
  void Update() override {
    std::vector<float> torques = {0, 0};

    // Run the FK to get leg state
    leg_.FK(robot_->GetJointPositions(), z_, x_);
    leg_.FkVel(robot_->GetJointPositions(), robot_->GetJointVelocities(), d_z_, d_x_);

    //Check hybrid modes and switch if need be
    if (mode_ != HybridMode::kSoftStart && z0_ - z_ > kTouchdownTol && d_z_ < 0) {
      mode_ = HybridMode::kStance;
    } else if (mode_ != HybridMode::kSoftStart && z0_ - z_ < 0 && d_z_ > 0) {
      mode_ = HybridMode::kFlight;
    }

    // Switch per mode and Run behavior
    switch (mode_) {
      case HybridMode::kSoftStart: {
        // In soft Start, control to a joint space position
        double q1_goal = kSoftStartAngle;
        double q2_goal = -2 * kSoftStartAngle;

        torques[0] = (kSoftKp * (q1_goal - robot_->GetJointPositions()[0]) - kSoftKd * robot_->GetJointVelocities()[0]);
        torques[1] = (kSoftKp * (q2_goal - robot_->GetJointPositions()[1]) - kSoftKd * robot_->GetJointVelocities()[1]);

        // If reached target and slow enough, switch to limp space, and set leg length as rest length
        if (std::abs(q1_goal - robot_->GetJointPositions()[0]) < kSoftPositionThreshold &&
            std::abs(q2_goal - robot_->GetJointPositions()[1]) < kSoftPositionThreshold &&
            std::abs(robot_->GetJointVelocities()[0]) < kSoftVelocityThreshold &&
            std::abs(robot_->GetJointVelocities()[1]) < kSoftVelocityThreshold) {
          mode_ = kFlight;
          z0_ = z_;
          LOG_INFO("Starting Limb mode.");
        }
        break;
      }
      case HybridMode::kFlight: {
        // In flight we use pd loops to control in limb space
        f_z_ = k_stiff_ * (z0_ - z_) - b_stiff_ * d_z_;
        // We Constrain f_x to prevent large jumps in x force
        f_x_ = kodlab::SoftStart::Constrain(-kp_ * x_ - kd_ * d_x_, -kMaxF_x, kMaxF_x);

        // convert from limb space to joint space
        torques = leg_.InverseDynamics(robot_->GetJointPositions(), f_z_, f_x_);
        break;
      }
      case HybridMode::kStance: {
        // In stance we use an Avik style active damping controller with gravity ffwd term
        float av = sqrtf((z_ - z0_) * (z_ - z0_) * w_v_ * w_v_ + d_z_ * d_z_);
        float F = kv_ * d_z_ / av + m_ * 9.81 * 1;

        // Confirm z direction force is positive
        f_z_ = fmax(k_ * (z0_ - z_) - b_ * d_z_ + F, 0.0);
        // Set x direction force to 0 since robot is constrained in the x direction
        f_x_ = 0;

        // Convert from limb space to join space
        torques = leg_.InverseDynamics(robot_->GetJointPositions(), f_z_, f_x_);
        break;
      }
    }
    //ffwd term for gravity comp
    torques[0] = torques[0] + 1 * 9.81 * 0.15 * 0.56 * sinf(robot_->GetJointPositions()[0]); // based on motor mass and length
    robot_->SetTorques(torques);
  }

  void PrepareLog() override {
    // Populate log message with data from current control loop cycle
    for (int servo = 0; servo < 2; servo++) {
      log_data_->positions[servo] = robot_->GetJointPositions()[servo];
      log_data_->velocities[servo] = robot_->GetJointVelocities()[servo];
      log_data_->modes[servo] = static_cast<int>(mjbots_interface_->GetJointModes()[servo]);
      log_data_->torque_cmd[servo] = robot_->GetJointTorqueCmd()[servo];
    }
    log_data_->limb_position[0] = z_ - z0_;
    log_data_->limb_position[1] = x_;
    log_data_->limb_vel[0] = d_z_;
    log_data_->limb_vel[1] = d_x_;
    log_data_->limb_wrench[0] = f_z_;
    log_data_->limb_wrench[1] = f_x_;
    log_data_->hybrid_mode = mode_;
  }

  void ProcessInput(const LegGains &input_data) override {
    kv_ = input_data.kv;
    k_ = input_data.k;
    k_stiff_ = input_data.k_stiff;
    b_ = input_data.b;
    b_stiff_ = input_data.b_stiff;
    kp_ = input_data.kp;
    kd_ = input_data.kd;
    LOG_INFO(
        "Response received: { kv = % 5.2f, k = % 5.2f, k_stiff = % 5.2f, "
        "b = % 5.2f, b_stiff = % 5.2f, kp = % 5.2f, kd = % 5.2f }",
        kv_, k_, k_stiff_, b_, b_stiff_, kp_, kd_);
  }

  enum HybridMode {
    kSoftStart = 0,
    kFlight = 1,
    kStance = 2
  };

  // Member classes
  kodlab::CartesianLeg leg_ = kodlab::CartesianLeg(0.15, 0.15);
  HybridMode mode_ = HybridMode::kSoftStart;

  // States
  float z_, x_, d_z_, d_x_; /// Limb space state
  float f_z_, f_x_; /// Limp space effort

  // Gains that can be set via input
  float kv_ = 0; /// Active damping gain
  float k_ = 800; /// Stance spring stiffness
  float k_stiff_ = 1600; /// Flight spring stiffness
  float m_ = 1.2; /// Robot mass
  float b_ = 15; /// Damping in stance
  float b_stiff_ = 15; /// Damping in flight
  float z0_ = 0; /// Rest length of leg
  float kp_ = 800; /// flight spring stiffness in x direction
  float kd_ = 10; /// flight damping in x direction
  float w_v_ = sqrtf(k_ / m_); /// Natural frequency

  // Other gains
  const float kTouchdownTol = 0.002;   /// Amount of spring compression in m needed to be considered in td
  const float kMaxF_x = 20;            /// Minimum and maximum force in the x direction in flight, used to
                                        /// Prevent the leg from overshooting do to large x direction error at liftoff
  // Soft start parameters
  const float kSoftStartAngle = -0.6; /// Initial soft start angle for hip
  const float kSoftKp = 6;             /// Joint space soft start position gain
  const float kSoftKd = 0.2;           /// Joint space soft start velocity gain
  const float kSoftPositionThreshold = 0.05;/// Threshold error from target in rad for terminating soft start
  const float kSoftVelocityThreshold = 0.08;/// Threshold error from 0 in rad/s for terminating soft start
};

int main(int argc, char **argv) {
  //Setup joints
  std::vector<kodlab::mjbots::JointMoteus> joints;
  joints.emplace_back(1, 1, 1, 0.1949);
  joints.emplace_back(2, 1, -1, 0.0389);

  // Define robot options
  kodlab::mjbots::ControlLoopOptions options;
  options.log_channel_name = "leg_data";
  options.input_channel_name = "leg_gains";
  options.soft_start_duration_ms = 5000;

  // Create control loop
  Hopping control_loop(std::move(joints), options);

  // Starts the loop, and then join it
  control_loop.Start();
  control_loop.Join();
  return 0;
}
