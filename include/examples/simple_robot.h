#include "kodlab_mjbots_sdk/robot_interface.h"

#pragma once

class SimpleRobot : virtual public kodlab::RobotInterface
{

    using kodlab::RobotInterface::RobotInterface;
    public:
        int mode = 0;
        
    void Update() override {
      std::vector<float> torques(num_joints_, 0);
      SetTorques(torques);
    }
};