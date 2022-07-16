
#include <vector>
#include <iostream>
#include <memory>
#include "kodlab_mjbots_sdk/common_header.h"
#include "ManyMotorLog.hpp"
#include "ModeInput.hpp"
#include "kodlab_mjbots_sdk/robot_interface.h"
#include "kodlab_mjbots_sdk/mjbots_control_loop.h"

#include "examples/simple_robot.h"

class MjbotsSimpleRobot : public kodlab::mjbots::MjbotsRobotInterface, public SimpleRobot
{
public:
    MjbotsSimpleRobot(std::vector<std::shared_ptr<kodlab::mjbots::JointMoteus>> joints_in,
                      kodlab::mjbots::ControlLoopOptions &options)
        : RobotInterface(joints_in, options.max_torque, options.soft_start_duration),
          kodlab::mjbots::MjbotsRobotInterface(joints_in,
                                               options.realtime_params,
                                               options.soft_start_duration,
                                               options.max_torque,
                                               options.imu_mounting_deg,
                                               options.attitude_rate_hz)
    {
        mode = 0;
    }

    MjbotsSimpleRobot(std::vector<std::shared_ptr<kodlab::mjbots::JointMoteus>> joints_in,
                      const kodlab::mjbots::RealtimeParams &realtime_params,
                      int soft_start_duration,
                      float robot_max_torque,
                      ::mjbots::pi3hat::Euler imu_mounting_deg,
                      int imu_rate_hz)
        : RobotInterface(joints_in, robot_max_torque, soft_start_duration),
          kodlab::mjbots::MjbotsRobotInterface(joints_in,
                                               realtime_params,
                                               soft_start_duration,
                                               robot_max_torque,
                                               imu_mounting_deg,
                                               imu_rate_hz)
    {
        mode = 0;
    }
};

class SimpleRobotControlLoop : public kodlab::mjbots::MjbotsControlLoop<ManyMotorLog, ModeInput, MjbotsSimpleRobot>
{
    using MjbotsControlLoop::MjbotsControlLoop;

    void CalcTorques() override
    {
        robot_->Update();
    }
    void PrepareLog() override
    {
        for (int servo = 0; servo < num_motors_; servo++)
        {
            log_data_.positions[servo] = robot_->GetJointPositions()[servo];
            log_data_.velocities[servo] = robot_->GetJointVelocities()[servo];
            log_data_.modes[servo] = static_cast<int>(robot_->GetJointModes()[servo]);
            log_data_.torques[servo] = robot_->GetJointTorqueCmd()[servo];
        }
        for (int servo = num_motors_; servo < 13; servo++)
        {
            log_data_.positions[servo] = 0;
            log_data_.velocities[servo] = 0;
            log_data_.modes[servo] = 0;
            log_data_.torques[servo] = 0;
        }
    }

    void ProcessInput() override
    {
        robot_->mode = lcm_sub_.data_.mode;
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
    // Setup joints
    // std::vector<kodlab::mjbots::JointMoteus> joints;
    // joints.emplace_back(100, 4, 1, 0,   1, 0);
    // joints.emplace_back(101, 4,-1, 0, 5.0/3.0, 0);

    JointSharedVector<kodlab::mjbots::JointMoteus> joints;
    joints.addJoint(100, 4, 1, 0, 1, 0);
    joints.addJoint(101, 4, -1, 0, 5.0 / 3.0, 0);

    // Define robot options
    kodlab::mjbots::ControlLoopOptions options;
    options.log_channel_name = "motor_data";
    options.frequency = 1000;
    options.realtime_params.main_cpu = 3;
    options.realtime_params.can_cpu = 2;
    // MjbotsSimpleRobot rob(joints, options);

    // Create control loop
    // Starts the loop, and then join it
    SimpleRobotControlLoop simple_robot(joints, options);
    simple_robot.Start();
    simple_robot.Join();
}
