/**
 * @file simple_io_behavior.h
 * @author Ethan J. Musser (emusser@seas.upenn.edu)
 * @brief Defines a simple behavior class which provides position control for a
 * single joint in a `SimpleRobot`, accepting gain commands via LCM input, and
 * logging joint position via LCM output.
 * @date 8/10/22
 * 
 * @copyright (c) Copyright 2022 The Trustees of the University of Pennsylvania. All rights reserved.
 * 
 */

#pragma once

#include <vector>

#include "PDGains.hpp"  // PDGains
#include "MotorLog.hpp"  // MotorLog
#include "real_time_tools/timer.hpp"  // real_time_tools::Timer

#include "kodlab_mjbots_sdk/io_behavior.h"  // kodlab::IOBehavior
#include "kodlab_mjbots_sdk/log.h"  // logging macros
#include "examples/simple_robot.h"  // SimpleRobot

/**
 * @brief Simple behavior that runs a positional PD control loop on the first
 * joint in a SimpleRobot with gains adjustable via LCM input and motor state
 * logged via LCM output
 */
class SimpleControlIOBehavior : public kodlab::IOBehavior<SimpleRobot,
                                                          PDGains,
                                                          MotorLog> {

 public:

  /**
   * @brief Un-hides `robot_` from the `IOBehavior` namespace
   * @note This is not necessary, but allows you to call `robot_` instead of
   * `this->robot_`.
   */
  using kodlab::IOBehavior<SimpleRobot, PDGains, MotorLog>::robot_;

  /**
   * @brief Using the default `IOBehavior` constructor
   */
  using kodlab::IOBehavior<SimpleRobot, PDGains, MotorLog>::IOBehavior;

  /**
   * @brief Initialize class members and return true upon completion
   * @return true if initialization successful, false otherwise
   */
  bool Init() override {
    // Initializes desired torques to zero
    desired_torques_ = std::vector<float>(robot_->joints.size(), 0);

    // Initialize joint positions and velocities
    joint_positions_ = robot_->GetJointPositions();
    joint_velocities_ = robot_->GetJointVelocities();

    // Return true to indicate successful initialization
    return true;
  }

  /**
   * @brief Begin the behavior
   * @details Record the current position as a set point and start the internal
   * behavior timer from zero.
   * @param prev_behavior the previously active behavior
   */
  void Begin(const kodlab::Behavior<SimpleRobot> &prev_behavior) override {
    // Begin realtime timer
    time_now_ = 0.0;
    timer_.tic();

    // Set desired position as current position
    desired_position_ = robot_->GetJointPositions()[0];
  }

  /**
   * @brief Compute the PD-controller update and set robot torques
   * @details This method updates the behavior timer, retrieves the latest joint
   * state, and uses this information to compute desired control input via a
   * proportional-derivative scheme using the member gains.
   */
  void Update() override {
    // Update time
    time_now_ = timer_.tac();

    // Retrieve current joint positions
    joint_positions_ = robot_->GetJointPositions();
    joint_velocities_ = robot_->GetJointVelocities();

    // Compute command torque for joint 0 via PD, set all others to zero
    desired_torques_ = std::vector<float>(robot_->joints.size(), 0.0);
    float pos_error = desired_position_ - joint_positions_[0];
    float vel_error = DESIRED_VELOCITY - joint_velocities_[0];
    desired_torques_[0] = gains.kp * pos_error + gains.kd * vel_error;

    // Set joint torques
    robot_->SetTorques(desired_torques_);
  }

  /**
   * @brief Process new input data on receipt
   * @details Overwrite the member gains with the control torques received via
   * the LCM input.
   * @param input_msg newly received input message
   */
  void ProcessInput(const PDGains &input_msg) override {
    gains.kp = input_msg.kp;
    gains.kd = input_msg.kd;
    std::fprintf(stdout,
                 "[INFO][SimpleControlIOBehavior] Setting kp = %.3f, kd = %.3f\n",
                 gains.kp,
                 gains.kd);
  }

  /**
   * @brief Prepare output log for publishing
   * @details Populate the fields of a MotorLog message with the controlled
   * joint's state.
   * @param output_msg message to be published
   */
  void PrepareOutput(MotorLog &output_msg) override {
    output_msg.timestamp = time_now_;
    output_msg.position = joint_positions_[0];
    output_msg.velocity = joint_velocities_[0];
    output_msg.torque = robot_->GetJointTorqueCmd()[0];
    output_msg.mode = 0;
  }

 private:

  std::vector<float> joint_positions_;  ///< Robot joint positions
  std::vector<float> joint_velocities_;  ///< Robot joint velocities

  float desired_position_;  ///< PD-controller position set point
  const float DESIRED_VELOCITY = 0.0;  ///< PD-controller velocity set point

  struct { float kp = 0.1; float kd = 0.005; } gains;  ///< PD gains
  std::vector<float> desired_torques_;  ///< Desired input torques

  real_time_tools::Timer timer_;  ///< Realtime timer
  float time_now_;  ///< Current behavior time

};
