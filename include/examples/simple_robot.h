#include "kodlab_mjbots_sdk/robot_interface.h"

class SimpleRobot : virtual public kodlab::RobotInterface
{

    using kodlab::RobotInterface::RobotInterface;
    public:
        int mode = 0;
};