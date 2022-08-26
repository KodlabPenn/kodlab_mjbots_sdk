/*!
 * @file robot_example.cpp
 * @author J. Diego Caporale <jdcap@seas.upenn.edu>
 * @brief Example that uses SimpleRobot(a RobotInterfaceDerived class defined in a seperate header file) separating the 
 *        controller, state update, and state machine from the lower lever control loop. 
 * @date 2022-07-17
 * 
 * @copyright Copyright (c) 2021 The Trustees of the University of Pennsylvania. All Rights Reserved
 *            BSD 3-Clause License
 * 
 */
#include <vector>
#include <iostream>
#include "kodlab_mjbots_sdk/common_header.h"
#include "ManyMotorLog.hpp"
#include "ModeInput.hpp"
#include "kodlab_mjbots_sdk/robot_base.h"
#include "kodlab_mjbots_sdk/mjbots_control_loop.h"

#include "examples/simple_robot.h"

class SimpleRobotControlLoop : public kodlab::mjbots::MjbotsControlLoop<ManyMotorLog, ModeInput, SimpleRobot>
{
    using MjbotsControlLoop::MjbotsControlLoop;

    void Update() override
    {
        robot_->Update();
    }
    void PrepareLog() override
    {
        // Populate log message with data from current control loop cycle
        for (int servo = 0; servo < num_joints_; servo++)
        {
            log_data_->positions[servo] = robot_->GetJointPositions()[servo];
            log_data_->velocities[servo] = robot_->GetJointVelocities()[servo];
            log_data_->modes[servo] = static_cast<int>(mjbots_interface_->GetJointModes()[servo]);
            log_data_->torques[servo] = robot_->GetJointTorqueCmd()[servo];
        }

        // Fill remaining log message fields with zeros
        for (int servo = num_joints_; servo < 13; servo++)
        {
            log_data_->positions[servo] = 0;
            log_data_->velocities[servo] = 0;
            log_data_->modes[servo] = 0;
            log_data_->torques[servo] = 0;
        }
    }

    void ProcessInput(const ModeInput &input_data) override
    {
        robot_->mode = input_data.mode;
        std::cout << "Switching to behavior " << robot_->mode << std::endl;
        // If the kill robot mode is detected kill robot using CTRL_C flag handler.
        if (robot_->mode == robot_->KILL_ROBOT)
        { // KILL_ROBOT
            kodlab::CTRL_C_DETECTED = true;
        }
    }
};

int main(int argc, char **argv)
{
    // Setup joints with a std::vector or JointSharedVector class
    std::vector<kodlab::mjbots::JointMoteus> joints;
    joints.emplace_back(100, 4, 1, 0,   1, 0);
    joints.emplace_back(101, 4,-1, 0, 5.0/3.0, 0);
    // JointSharedVector<kodlab::mjbots::JointMoteus> joints;
    // joints.addJoint(100, 4, 1, 0, 1, 0);
    // joints.addJoint(101, 4, -1, 0, 5.0 / 3.0, 0);



    // Define robot options
    kodlab::mjbots::ControlLoopOptions options;
    options.log_channel_name = "motor_data";
    options.input_channel_name = "mode_input";
    options.frequency = 1000;
    options.realtime_params.main_cpu = 3;
    options.realtime_params.can_cpu = 2;

    // Create control loop
    // Starts the loop, and then join it
    SimpleRobotControlLoop simple_robot(joints, options);
    simple_robot.Start();
    simple_robot.Join();
}
