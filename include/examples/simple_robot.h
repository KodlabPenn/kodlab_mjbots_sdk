/**
 * @file simple_robot.h
 * @author J. Diego Caporale
 * @brief A simple RobotInterface derived class example. Here is where you would implement any state updates,
 *        behaviors, or control system
 * @date 2022-07-19
 *
 * @copyright Copyright (c) 2022
 *
 */

#pragma once

#include "kodlab_mjbots_sdk/robot_interface.h"

class SimpleRobot : virtual public kodlab::RobotInterface
{

    using kodlab::RobotInterface::RobotInterface;
    public:
        int mode = 0;
};