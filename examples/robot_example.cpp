
// #include "kodlab_mjbots_sdk/robot_control_loop.h"
// #include "kodlab_mjbots_sdk/behavior.h"
// #include "kodlab_mjbots_sdk/robot_super.h"
#include <vector>
#include <iostream>
#include <memory>
#include "kodlab_mjbots_sdk/common_header.h"
#include "ManyMotorLog.hpp"
#include "ModeInput.hpp"
#include "kodlab_mjbots_sdk/robot_interface.h"
#include "kodlab_mjbots_sdk/mjbots_control_loop.h"  

#include "examples/simple_robot.h"

class MjbotsSimpleRobot : public SimpleRobot, public kodlab::mjbots::MjbotsRobotInterface
{   
public:
    int mode = 0;
    MjbotsSimpleRobot( std::vector<std::shared_ptr<kodlab::mjbots::JointMoteus>> joints_in , 
                        kodlab::mjbots::ControlLoopOptions &options)
    : RobotInterface(joints_in,options.max_torque,options.soft_start_duration),
      kodlab::mjbots::MjbotsRobotInterface(joints_in, options.realtime_params,options.soft_start_duration,options.max_torque,options.imu_mounting_deg,options.attitude_rate_hz)
      {}

};

class SimpleRobotControlLoop : public kodlab::mjbots::MjbotsControlLoop<ManyMotorLog,ModeInput> {
  using MjbotsControlLoop::MjbotsControlLoop;

  void CalcTorques() override {
    std::vector<float> torques(num_motors_, 0);
    robot_->SetTorques(torques);
  }  
  void PrepareLog() override {
      for (int servo = 0; servo < num_motors_; servo++) {
          log_data_.positions[servo] = robot_->GetJointPositions()[servo];
          log_data_.velocities[servo] = robot_->GetJointVelocities()[servo];
          log_data_.modes[servo] = static_cast<int>(robot_->GetJointModes()[servo]);
          log_data_.torques[servo] = robot_->GetJointTorqueCmd()[servo];
      }
      for (int servo = num_motors_; servo < 13; servo++) {
          log_data_.positions[servo] = 0;
          log_data_.velocities[servo] = 0;
          log_data_.modes[servo] = 0;
          log_data_.torques[servo] = 0;
      }
  } 
  
  void ProcessInput() override {
    int behavior = 0;
      behavior = lcm_sub_.data_.mode;
      std::cout << "Switching to behavior " << behavior  << std::endl;
      // If the kill robot mode is detected kill robot using CTRL_C flag handler.
      if (behavior  == -1){ //KILL_ROBOT
          kodlab::CTRL_C_DETECTED = true;
      }
  }
};
template<class joint_type>
class JointSharedVector {
    std::vector<std::shared_ptr<joint_type>> v_;
  public:
    template<typename... Args>
    void addJoint(Args... args){
      v_.push_back(std::make_shared<joint_type>(args...));
    }
    operator std::vector<std::shared_ptr<joint_type>> () { return v_; }
};



int main(int argc, char **argv) {


  //Setup joints
  // std::vector<kodlab::mjbots::JointMoteus> joints2;
  // joints2.emplace_back(100, 4, 1, 0,   1, 0);
  // joints2.emplace_back(101, 4,-1, 0, 5.0/3.0, 0);

  JointSharedVector<kodlab::mjbots::JointMoteus> joints;
  joints.addJoint(100, 4, 1, 0,   1, 0);
  joints.addJoint(101, 4,-1, 0, 5.0/3.0, 0);

  // Define robot options
  kodlab::mjbots::ControlLoopOptions options;
  options.log_channel_name = "motor_data";
  options.frequency = 1000;
  options.realtime_params.main_cpu = 3;
  options.realtime_params.can_cpu  = 2;
  MjbotsSimpleRobot rob(joints, options);

  // Create control loop
  // Starts the loop, and then join it
  SimpleRobotControlLoop simple_robot(std::make_shared<MjbotsSimpleRobot>(std::move(rob)),options);
  simple_robot.Start();
  simple_robot.Join();
}
