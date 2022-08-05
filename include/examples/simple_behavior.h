/**
 * @file simple_behavior.h
 * @author Ethan J. Musser (emusser@seas.upenn.edu)
 * @brief Defines a simple behavior class which commands torques to all joints
 *        in a SimpleRobot
 * @date 7/26/22
 * 
 * @copyright Copyright 2022 The Trustees of the University of Pennsylvania. All
 *            rights reserved.
 * 
 */

#pragma once

#include <memory>  // std::unique_ptr
#include <iostream>  // std::cout, std::endl
#include <numeric>  // std::accumulate
#include <vector>  // std::vector
#include "kodlab_mjbots_sdk/behavior.h"  // kodlab::Behavior
#include "examples/simple_robot.h"  // SimpleRobot
#include "ManyMotorLog.hpp"

/**
 * @brief Simple behavior that commands torques to all joints in a SimpleRobot
 */
class SimpleBehavior : virtual public kodlab::Behavior<SimpleRobot, ManyMotorLog>
{

private:
  /**
   * @brief The torque to be commanded while motors are running fully
   */
  const float RUN_TORQUE = 0;

  /**
   * @brief The torque to be commanded while motors are being slowed
   */
  const float SLOW_TORQUE = 0;

  /**
   * @brief Enumeration of valid states while behavior is active
   */
  enum States { RUNNING, STOPPING } state_;

  /**
   * @brief The initial joint positions when the behavior begins
   */
  std::vector<float> joint_start_positions_;

  /**
   * @brief The joint velocities
   */
  std::vector<float> joint_velocities_;

  /**
   * @brief The desired joint torques
   */
  std::vector<float> desired_torques_;

public:

  /**
   * @brief Using default Behavior constructor
   */
  // using Behavior::Behavior;
  SimpleBehavior(std::shared_ptr<SimpleRobot> robot_in, std::string name_in = "")
      : Behavior<SimpleRobot, ManyMotorLog>(robot_in, name_in) {}

  /**
   * @brief Initializes the behavior
   * @details This is where initialization code, which runs when a behavior is
   *          first added to a behavior manager, should be put. This is where
   *          initialization that did not take place in the constructor should
   *          occur, e.g., setting up pointers. In this example, the desired
   *          torques are initialized to zero.
   * @return true if initialization successful, false otherwise
   */
  bool Init() override
  {
    // Initializes desired torques (for user safety, these are set to zero)
    desired_torques_ = std::vector<float>(robot_->joints.size(), 0);

    // Indicate that behavior has been initialized and return
    std::fprintf(stdout, "Initialized %s\n", get_name().c_str());
    return true;  // indicates successful initialization
  }

  /**
   * @brief Starts the behavior
   * @details This code runs a single time when the behavior starts up. This is
   *          a good place to read in the initial robot state and set the
   *          appropriate variables to begin the behavior. The previously active
   *          behavior is passed as an argument, permitting different
   *          functionalities for different prior behaviors. In this example,
   *          the behavior is marked as active and the initial robot state is
   *          read.
   * @param prev_behavior the previously active behavior
   */
  void Begin(const kodlab::AbstractBehavior<SimpleRobot> &prev_behavior) override
  {
    // Update internal state
    state_ = RUNNING;

    // Read starting joint positions
    joint_start_positions_ = robot_->GetJointPositions();
    std::fprintf(stdout, "%s has begun\n", get_name().c_str());
  }

  /**
   * @brief Updates the behavior
   * @details This code runs after Begin, and is called on every control loop
   *          update. Use this to define the primary content of your behavior,
   *          e.g., to update and command new joint torques.  Additionally, this
   *          code can monitor if the behavior has terminated and update flags
   *          appropriately. In this example, this function reads the updated
   *          joint velocities and maintains state machine corresponding to the
   *          member enum `state_`.
   */
  void Update() override
  {
    // Update joint velocities from robot
    joint_velocities_ = robot_->GetJointVelocities();

    switch (state_)
    {
      case RUNNING:  // in a running state
        // Set joint torques to running levels
        desired_torques_ =
            std::vector<float>(robot_->joints.size(), RUN_TORQUE);
        break;

      case STOPPING:  // in a stopping state
        // Lower the joint torques to slowing levels
        desired_torques_ =
            std::vector<float>(robot_->joints.size(), SLOW_TORQUE);
        break;
    }

    // Set joint torques
    robot_->SetTorques(desired_torques_);
  }

  /**
   * @brief Stops the behavior
   * @details This code is run when a behavior is directed to stop running by
   *          the behavior manager. This should trigger a shutdown process for
   *          the behavior, e.g. slowing the motor speed. The upcoming behavior
   *          is passed as an argument to permit different shutdown procedures
   *          for different on-deck behaviors. In this example, the internal
   *          state is modified to indicate the behavior should stop.
   * @param next_behavior the upcoming behavior
   */
  void Stop(const kodlab::AbstractBehavior<SimpleRobot> &next_behavior) override
  {
    // Indicate that behavior should stop
    state_ = STOPPING;
    std::fprintf(stdout, "Stopping %s\n", get_name().c_str());
  }

  /**
   * @brief Returns the active state of the behavior
   * @details This function identifies if the behavior is active. By default, it
   *          simply accesses the `active_` variable. More advanced
   *          implementations may use robot state to make this determination.
   * @return true if behavior is running, false otherwise
   */
  bool Running() override { return active_; }

  /**
   * @brief Indicates if behavior is finished and ready to transition
   * @details This function indicates to the behavior manager whether the
   *          behavior is prepared to terminate and transition to the next
   *          behavior. In its simplest form, this function simply accesses
   *          the internal behavior active flag. This function can be used as a
   *          mode switch guard.
   * @param next_behavior the upcoming behavior
   * @return true if behavior is ready to transition, false otherwise
   */
  bool ReadyToSwitch(const kodlab::AbstractBehavior<SimpleRobot> &next_behavior) override
  {
    // Ready to switch if average velocity is below a threshold
    float avg_velocity =
        std::accumulate(joint_velocities_.begin(),
                        joint_velocities_.end(),
                        0.0) / static_cast<float>(joint_velocities_.size());
    if (avg_velocity <= 0.1)
    {
      std::fprintf(stdout, "%s is no longer active\n", get_name().c_str());
      return true;
    }
    return false;
  }

  /**
   * @brief PrepareLog implementation
   * @todo Probably find a way to pass a log function that maybe is written elsewhere?
   *       don't want to be rewriting this same function for every behavior
   * @todo Add some sort of default capability that allows the control to log instead?
   */
  void PrepareLog() override
  {
    for (int servo = 0; servo < robot_->joints.size(); servo++)
    {
        log_data_.positions[servo] = robot_->GetJointPositions()[servo];
        log_data_.velocities[servo] = robot_->GetJointVelocities()[servo];
        //TODO Can't log mjbots_interface, how do we feel about that?
        log_data_.modes[servo] = 0; // static_cast<int>(mjbots_interface_->GetJointModes()[servo]);
        log_data_.torques[servo] = robot_->GetJointTorqueCmd()[servo];
    }
    for (int servo = robot_->joints.size(); servo < 13; servo++)
    {
        log_data_.positions[servo] = 0;
        log_data_.velocities[servo] = 0;
        log_data_.modes[servo] = 0;
        log_data_.torques[servo] = 0;
    }
  }
};

