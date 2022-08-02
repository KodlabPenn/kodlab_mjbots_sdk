/**
 * @file simple_robot.h
 * @author J. Diego Caporale
 * @brief A simple RobotBase derived class example. Here is where you would implement any state updates,
 *        behaviors, or control system
 * @date 2022-07-19
 *
 * @copyright Copyright (c) 2022
 *
 */

#pragma once

#include "kodlab_mjbots_sdk/robot_base.h"

class SimpleRobot : virtual public kodlab::RobotBase
{

    using kodlab::RobotBase::RobotBase;

public:
    int mode = 0; // Member variables encode whatever added state we need
    // Set up robot update function for state and torques
    void Update() override
    {
        std::vector<float> torques(num_joints_, 0);
        SetTorques(torques);
    }
};