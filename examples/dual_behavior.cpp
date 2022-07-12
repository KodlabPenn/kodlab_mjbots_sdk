
// #include "kodlab_mjbots_sdk/robot_control_loop.h"
// #include "kodlab_mjbots_sdk/behavior.h"
#include "kodlab_mjbots_sdk/robot_super.h"
#include "kodlab_mjbots_sdk/mjbots_control_loop.h"  
#include "ManyMotorLog.hpp"
#include "ModeInput.hpp"
#include <iostream>


class DualActuatorRobot : public kodlab::RobotSuperBase<ManyMotorLog,ModeInput>{
  using kodlab::RobotSuperBase<ManyMotorLog, ModeInput>::RobotSuperBase;
  void Update(){
    //TODO add behaviors or implementation code
  }
  
  void PrepareLog() override {
      for (int servo = 0; servo < joints.size(); servo++) {
          log_data().positions[servo] = mjbots_interface_->GetJointPositions()[servo];
          log_data().velocities[servo] = mjbots_interface_->GetJointVelocities()[servo];
          log_data().modes[servo] = static_cast<int>(mjbots_interface_->GetJointModes()[servo]);
          log_data().torques[servo] = mjbots_interface_->GetJointTorqueCmd()[servo];
      }
      for (int servo = joints.size(); servo < 13; servo++) {
          log_data().positions[servo] = 0;
          log_data().velocities[servo] = 0;
          log_data().modes[servo] = 0;
          log_data().torques[servo] = 0;
      }
  } 
  
  void ProcessInput() override {
    int behavior = 0;
      behavior = input_data().mode;
      std::cout << "Switching to behavior " << behavior  << std::endl;
      // If the kill robot mode is detected kill robot using CTRL_C flag handler.
      if (behavior  == -1){ //KILL_ROBOT
          kodlab::CTRL_C_DETECTED = true;
      }
  }
};





int main(int argc, char **argv) {


  //Setup joints
  std::vector<kodlab::mjbots::JointMoteus> joints;
  joints.emplace_back(100, 4, 1, 0,   1, 0);
  joints.emplace_back(101, 4,-1, 0, 5.0/3.0, 0);

  // Define robot options
  kodlab::mjbots::ControlLoopOptions options;
  options.log_channel_name = "motor_data";
  options.frequency = 1000;
  options.realtime_params.main_cpu = 3;
  options.realtime_params.can_cpu  = 2;

  // Create control loop
  // Starts the loop, and then join it
  std::cout << "Building Super Base" << std::endl;
  DualActuatorRobot dual_behavior(joints,options);
  std::cout << "Super Base Ready" << std::endl;

  return dual_behavior.Loop();
}










// #include "kodlab_mjbots_sdk/robot_control_loop.h"
// #include "kodlab_mjbots_sdk/behavior.h"
// #include "kodlab_mjbots_sdk/robot_base.h"
// #include "ManyMotorLog.hpp"
// #include "ModeInput.hpp"
// #include <iostream>

// class DualActuator : public kodlab::RobotBase{
//     //int behavior = 0;

//     public:
//         DualActuator(std::vector<kodlab::mjbots::JointMoteus> joint_vect)
//             : kodlab::RobotBase(joint_vect){}
//     // protected:
//         void Init() {
//             if(joints.size()!=2)
//             {
//                 std::cout<<"Wrong number of joints in the DualActuator"<<std::endl;
//             }
//         }
//         void Update(){
//             // behaviors(behavior).update();
//         }
// };

// class DualActuatorLoop : public kodlab::RobotControlLoop<ManyMotorLog, ModeInput>{
//     public:

//     protected:
//         // void CalcTorques(){robot.Update()}
//         // void CalcTorques(){
//         //     robot.attitude.Update(robot_.GetAttitude());
//         //     RobotControlLoop::CalcTorques();
//         //     }
//         void PrepareLog() override {
//             for (int servo = 0; servo < num_motors_; servo++) {
//                 log_data_.positions[servo] = robot_->GetJointPositions()[servo];
//                 log_data_.velocities[servo] = robot_->GetJointVelocities()[servo];
//                 log_data_.modes[servo] = static_cast<int>(robot_->GetJointModes()[servo]);
//                 log_data_.torques[servo] = robot_->GetJointTorqueCmd()[servo];
//             }
//             for (int servo = num_motors_; servo < 13; servo++) {
//                 log_data_.positions[servo] = 0;
//                 log_data_.velocities[servo] = 0;
//                 log_data_.modes[servo] = 0;
//                 log_data_.torques[servo] = 0;
//             }
//         } 
//         void ProcessInput() override {
//             robot.behavior = lcm_sub_.data_.mode;
//             std::cout << "Switching to behavior " << robot.behavior  << std::endl;
//             // If the kill robot mode is detected kill robot using CTRL_C flag handler.
//             if (robot.behavior  == -1){ //KILL_ROBOT
//                 kodlab::CTRL_C_DETECTED = true;
//             }
//         }
// };

// class Actuator1 : public Behavior{

// };
// class Actuator2 : public Behavior{

// };




// int main(int argc, char **argv) {


// //   //Setup joints
// //   std::vector<kodlab::mjbots::JointMoteus> joints;
// //   joints.emplace_back(100, 4, 1, 0,   1, 1);
// //   joints.emplace_back(101, 4,-1, 0, 5.0/3.0, 1);

// //   // Define robot options
// //   kodlab::mjbots::ControlLoopOptions options;
// //   options.log_channel_name = "motor_data";
// //   options.frequency = 1000;
// //   options.realtime_params.main_cpu = 3;
// //   options.realtime_params.can_cpu  = 2;

// //   DualActuator robot(joints);
  
// //   // Create control loop
// //   // Starts the loop, and then join it
// //   kodlab::RobotControlLoop dual_behavior(robot, options);
// //   dual_behavior.Start();
// //   dual_behavior.Join();
//   return 0;
// }
