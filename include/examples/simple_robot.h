#include "kodlab_mjbots_sdk/robot_interface.h"

class SimpleRobot : public kodlab::RobotInterface
{
    int mode = 0;
    using kodlab::RobotInterface::RobotInterface;
};