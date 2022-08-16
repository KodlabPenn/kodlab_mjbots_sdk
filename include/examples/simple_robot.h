/**
 * @file simple_robot.h
 * @author J. Diego Caporale
 * @brief A simple RobotBase derived class example. An example of where you 
 *        would implement any state updates, behaviors, or control system.
 * @date 2022-07-19
 *
 * @copyright Copyright (c) 2022
 *
 */

#pragma once

#include "kodlab_mjbots_sdk/robot_base.h"

/**
 * @brief Simple Robot class, updates state variables and sets torques to zero.
 * @note Setting torque is not necessarily something that should be done in
 *       a robot class this example just shows that it can be
 */
class SimpleRobot : virtual public kodlab::RobotBase
{

    using kodlab::RobotBase::RobotBase;

public:
    int mode = 0; // Member variables encode whatever added state we need
    // Set up robot update function for state and torques
    void Update() override {
        std::vector<float> torques(num_joints_, 0);
        // Get states you want to use.
        const auto& imu_data = GetIMUData();
        auto pos = GetJointPositions();
        auto vel = GetJointVelocities();

        SetTorques(torques);
    }
};
