#pragma once
#include "kodlab_mjbots_sdk/joint_moteus.h"
#include "kodlab_mjbots_sdk/mjbots_control_loop.h"
#include "kodlab_mjbots_sdk/mjbots_robot_interface.h"
#include "VoidLcm.hpp"
// #include "kodlab_mjbots_sdk/robot_control_loop.h"
// #include "kodlab_mjbots_sdk/behavior.h"

namespace kodlab
{
    template <class LogClass = VoidLcm, class InputClass = VoidLcm>
    class RobotControlLoop;

    template <class LogClass = VoidLcm, class InputClass = VoidLcm>
    class RobotSuperBase;

    template <class LogClass, class InputClass>
    class RobotControlLoop : public mjbots::MjbotsControlLoop<LogClass, InputClass>
    {
    public:
        kodlab::RobotSuperBase<LogClass, InputClass> *robot = nullptr;
        RobotControlLoop<LogClass, InputClass>(
            kodlab::RobotSuperBase<LogClass, InputClass> *k_robot_ptr,
            const kodlab::mjbots::ControlLoopOptions &options);

    protected:
        void CalcTorques() override;
        void PrepareLog() override;
        void ProcessInput() override;

        friend class kodlab::RobotSuperBase<LogClass, InputClass>;
    };

    template <class LogClass, class InputClass>
    class RobotSuperBase
    {
    public:
        std::unique_ptr<kodlab::RobotControlLoop<LogClass, InputClass>> ctrl_loop = nullptr;
        std::vector<std::shared_ptr<kodlab::mjbots::JointMoteus>> joints;

        RobotSuperBase(std::vector<kodlab::mjbots::JointMoteus> joint_vect, const kodlab::mjbots::ControlLoopOptions &options);
        int Loop();
        virtual void Init() {}
        virtual void Update() {}
        virtual void PrepareLog() {}
        virtual void ProcessInput() {}
    protected:
        LogClass &log_data(){return ctrl_loop->log_data_;}
        InputClass &input_data(){return ctrl_loop->lcm_sub_.data_;}
        std::shared_ptr<kodlab::mjbots::MjbotsRobotInterface> mjbots_interface_;
    };
    /******************************IMPLEMENTATION******************************/

    template <class LogClass, class InputClass>
    RobotControlLoop<LogClass, InputClass>::RobotControlLoop(
        kodlab::RobotSuperBase<LogClass, InputClass> *k_robot_ptr,
        const kodlab::mjbots::ControlLoopOptions &options)
        : robot(k_robot_ptr),
          kodlab::mjbots::MjbotsControlLoop<LogClass, InputClass>(k_robot_ptr->joints, options) // had to use k_robot_ptr or it would seg fault
    {
        std::cout << "Control Loop Constructed" << std::endl;
    }

    template <class LogClass, class InputClass>
    void RobotControlLoop<LogClass, InputClass>::CalcTorques() { robot->Update(); }
    template <class LogClass, class InputClass>
    void RobotControlLoop<LogClass, InputClass>::PrepareLog() { robot->PrepareLog(); }
    template <class LogClass, class InputClass>
    void RobotControlLoop<LogClass, InputClass>::ProcessInput() { robot->ProcessInput(); }

    template <class LogClass, class InputClass>
    int RobotSuperBase<LogClass, InputClass>::Loop()
    {
        if (!ctrl_loop)
        {
            std::cout << "No control loop present, please init control with member function, InitCtrlLoop(...)" << std::endl;
            return 1;
        }
        ctrl_loop->Start();
        ctrl_loop->Join();
        return 0;
    }

    template <class LogClass, class InputClass>
    RobotSuperBase<LogClass, InputClass>::RobotSuperBase(
        std::vector<kodlab::mjbots::JointMoteus> joint_vect,
        const kodlab::mjbots::ControlLoopOptions &options)
        : joints(make_share_vector(joint_vect))
    {
          ctrl_loop = std::make_unique<kodlab::RobotControlLoop<LogClass, InputClass>>(this, options);
          mjbots_interface_ = ctrl_loop->robot_;   
          std::cout << "SuperBase Constructed" << std::endl;
    }
} // namespace kodlab