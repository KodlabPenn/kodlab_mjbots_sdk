/**
 * @file behavior_robot_example.cpp
 * @author Ethan J. Musser (emusser@seas.upenn.edu)
 * @brief Example demonstrating the usage of the `MjbotsBehaviorLoop` class with
 *        a SimpleRobot (derived from `kodlab::RobotBase`) and a
 *        `SimpleSpinJointsBehavior` (derived from `kodlab::Behavior`). This file
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
#include "examples/simple_robot.h"  // SimpleRobot
#include "examples/simple_spin_joints_behavior.h"  // SimpleSpinJointsBehavior
#include "examples/simple_control_io_behavior.h" // SimpleControlIOBehavior

namespace kodlab::examples {

/**
 * @brief Behavior loop for a simple robot
 * @note Unlike the `SimpleRobotControlLoop` provided in `robot_example.cpp`,
 *       this control loop does not override the `Update` method. This is
 *       handled internally by the `MjbotsBehaviorLoop`.
 */
class SimpleRobotBehaviorLoop : public kodlab::mjbots::MjbotsBehaviorLoop<
    ManyMotorLog, ModeInput, SimpleRobot> {
  /**
   * @brief Use `MjbotsBehaviorLoop` constructor
   */
  using MjbotsBehaviorLoop::MjbotsBehaviorLoop;

  /**
   * @brief Logs joint state information
   * @details This method is called every control loop update, and logs general
   *          joint data to a `ManyMotorLog` via the LCM log data topic.
   */
  void PrepareLog() override {
    for (int servo = 0; servo < num_joints_; servo++) {
      log_data_->positions[servo] = robot_->GetJointPositions()[servo];
      log_data_->velocities[servo] = robot_->GetJointVelocities()[servo];
      log_data_->modes[servo] =
          static_cast<int>(mjbots_interface_->GetJointModes()[servo]);
      log_data_->torques[servo] = robot_->GetJointTorqueCmd()[servo];
    }
    for (int servo = num_joints_; servo < 13; servo++) {
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
  void ProcessInput(const ModeInput &input_data) override {
    // Set input behavior on internal behavior manager
    SetBehavior(input_data.mode);
  }
};

} // kodlab::examples


/**
 * @brief Main method
 * @return 0 if program exited without issue
 */
int main(int argc, char **argv) {
  // Setup joints with a std::vector
  std::vector<kodlab::mjbots::JointMoteus> joints;
  joints.emplace_back(22, 1, 1, 0, 1.0, 1.0);

  // Define robot options
  kodlab::mjbots::ControlLoopOptions options;
  options.log_channel_name = "motor_data";  // LCM topic for logging
  options.input_channel_name =
      "mode_input";  // LCM topic for input (must match broadcasting topic)
  options.frequency = 1000;  // control loop frequency
  options.realtime_params.main_cpu = 3;
  options.realtime_params.can_cpu = 2;

  // Create control loop
  kodlab::examples::SimpleRobotBehaviorLoop simple_robot(joints, options);

  // Build and initialize behaviors list
  // Note: the behaviors list includes a kodlab::OffBehavior as the first
  // element by default. This can be replaced with an alternative default
  // behavior via SetDefaultBehavior, as follows. See the BehaviorManager
  // definition for details.
  simple_robot.SetDefaultBehavior<kodlab::OffBehavior<SimpleRobot>>("SIMPLE OFF");
  simple_robot.AddBehavior<SimpleSpinJointsBehavior>("SIMPLE BEHAVIOR");
  simple_robot.AddIOBehavior<SimpleControlIOBehavior>("pd_gain_input",
                                                      "motor_position_output",
                                                      "SIMPLE I/O BEHAVIOR");

  // Display completed behaviors list
  simple_robot.PrintBehaviorList();

  // Starts the loop, and then join it
  simple_robot.Start();
  simple_robot.Join();

  // Return 0 if exited successfully
  return 0;
}


