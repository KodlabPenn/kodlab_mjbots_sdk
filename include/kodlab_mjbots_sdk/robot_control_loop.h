#pragma once 
#include "kodlab_mjbots_sdk/mjbots_control_loop.h"
#include "kodlab_mjbots_sdk/lcm_subscriber.h"
#include <sys/mman.h>


template <class LogClass = VoidLcm, class InputClass = VoidLcm>
class RobotControlLoop : public mjbots::MjbotsControlLoop<LogClass, InputClass>;
#include "kodlab_mjbots_sdk/robot_super.h"

namespace kodlab
{
    template <class LogClass = VoidLcm, class InputClass = VoidLcm>
    class RobotControlLoop : public mjbots::MjbotsControlLoop<LogClass, InputClass>
    {
    public:
        kodlab::RobotSuperBase<LogClass,InputClass> *robot;
        RobotControlLoop<LogClass, InputClass>(
                    kodlab::RobotSuperBase<LogClass,InputClass>  *k_robot_ptr, 
                    const kodlab::mjbots::ControlLoopOptions &options)
            : robot(k_robot_ptr), kodlab::mjbots::MjbotsControlLoop<LogClass, InputClass>(robot->joints, options)
        {
            robot.Init();
        }

    protected:
        void CalcTorques() override { robot->Update(); }
    };
} // namespace kodlab