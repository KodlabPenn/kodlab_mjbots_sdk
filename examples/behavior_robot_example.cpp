/**
 * @file behavior_robot_example.cpp
 * @author Ethan J. Musser (emusser@seas.upenn.edu)
 * @brief Example demonstrating the usage of the `MjbotsBehaviorLoop` class with
 *        a SimpleRobot (derived from `kodlab::RobotBase`) and a
 *        `SimpleBehavior` (derived from `kodlab::Behavior`). This file
 *        constructs the control loop, which contains a behavior manager, for a
 *        `SimpleRobot` and provides several behaviors which can be switched
 *        between via LCM broadcast.
 * @date 7/27/22
 * 
 * @copyright Copyright 2022 The Trustees of the University of Pennsylvania. All
 *            rights reserved.
 * 
 */

#include <vector>  // std::vector
#include "ManyMotorLog.hpp"  // ManyMotorLog
#include "ModeInput.hpp"  // ModeInput
#include "kodlab_mjbots_sdk/mjbots_behavior_loop.h"  // kodlab::mjbots::MjbotsBehaviorLoop
#include "examples/simple_robot.h"  // SimpleBehavior
#include "examples/simple_behavior.h"  // SimpleRobot

/**
 * @brief Behavior loop for a simple robot
 * @note Unlike the `SimpleRobotControlLoop` provided in `robot_example.cpp`,
 *       this control loop does not override the `Update` method. This is
 *       handled internally by the `MjbotsBehaviorLoop`.
 */
class SimpleRobotBehaviorLoop : public kodlab::mjbots::MjbotsBehaviorLoop<
    ManyMotorLog, ModeInput, SimpleRobot>
{
  /**
   * @brief Use `MjbotsBehaviorLoop` constructor
   */
  using MjbotsBehaviorLoop::MjbotsBehaviorLoop;

  /**
   * @brief Logs joint state information
   * @details This method is called every control loop update, and logs general
   *          joint data to a `ManyMotorLog` via the LCM log data topic.
   */
  void PrepareLog() override
  {
    for (int servo = 0; servo < num_joints_; servo++)
    {
      log_data_->positions[servo] = robot_->GetJointPositions()[servo];
      log_data_->velocities[servo] = robot_->GetJointVelocities()[servo];
      log_data_->modes[servo] =
          static_cast<int>(mjbots_interface_->GetJointModes()[servo]);
      log_data_->torques[servo] = robot_->GetJointTorqueCmd()[servo];
    }
    for (int servo = num_joints_; servo < 13; servo++)
    {
      log_data_->positions[servo] = 0;
      log_data_->velocities[servo] = 0;
      log_data_->modes[servo] = 0;
      log_data_->torques[servo] = 0;
    }
  }

  /**
   * @brief Callback for new mode input LCM input message
   * @details This function passes input mode selection integers received via
   *          LCM to the behavior loop's internal behavior manager. The behavior
   *          manager then handles the actual behavior switching.
   */
  void ProcessInput(const ModeInput &input_data) override
  {
    // Set input behavior on internal behavior manager
    behavior_mgr.SetBehavior(input_data.mode);
  }
};

/**
 * @brief Main method
 * @return 0 if program exited without issue
 */
int main(int argc, char **argv)
{
  // Setup joints with a std::vector
  std::vector<kodlab::mjbots::JointMoteus> joints;
  joints.emplace_back(100, 4, 1, 0, 1, 0);
  joints.emplace_back(101, 4, -1, 0, 5.0 / 3.0, 0);

  // Define robot options
  kodlab::mjbots::ControlLoopOptions options;
  options.log_channel_name = "motor_data";  // LCM topic for logging
  options.input_channel_name =
      "mode_input";  // LCM topic for input (must match broadcasting topic)
  options.frequency = 1000;  // control loop frequency
  options.realtime_params.main_cpu = 3;
  options.realtime_params.can_cpu = 2;

  // Create control loop
  SimpleRobotBehaviorLoop simple_robot(joints, options);

  // Build and initialize behaviors list
  // Note: the behaviors list includes a kodlab::OffBehavior as the first
  // element by default. This can be replaced with an alternative default
  // behavior via SetDefaultBehavior, as follows. See the BehaviorManager
  // definition for details.
  simple_robot.SetDefaultBehavior<kodlab::OffBehavior<SimpleRobot>>("MY_OFF");
  simple_robot.AddBehavior<SimpleBehavior>("SIMPLE");
  simple_robot.AddBehavior<SimpleBehavior>("ANOTHER_SIMPLE");
  simple_robot.AddBehavior<kodlab::OffBehavior<SimpleRobot>>("ANOTHER_OFF");

  // Display completed behaviors list
  simple_robot.behavior_mgr.PrintBehaviorList();

  // Starts the loop, and then join it
  simple_robot.Start();
  simple_robot.Join();

  // Return 0 if exited successfully
  return 0;
}


