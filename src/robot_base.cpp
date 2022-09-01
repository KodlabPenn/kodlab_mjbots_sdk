/*!
 * @file robot_base.cpp
 * @author Kodlab - J. Diego Caporale (jdcap@seas.upenn.edu)
 * @brief Robot Base Class
 * @date 2022-07-12
 * 
 * @copyright Copyright (c) 2022
 * 
 */
#include "kodlab_mjbots_sdk/robot_base.h"

std::vector<float> kodlab::RobotBase::GetJointPositions() { //Copy of positions
  std::vector<float>pos(positions_.begin(), positions_.end());
  return pos;
}

std::vector<float> kodlab::RobotBase::GetJointVelocities() { //Copy of velocities
  std::vector<float>vel(velocities_.begin(), velocities_.end());
  return vel;
}

std::vector<std::shared_ptr<kodlab::JointBase>> kodlab::RobotBase::GetJoints(std::vector<int> joint_indices){
  std::vector<std::shared_ptr<kodlab::JointBase>> joint_list;
  for (int ind: joint_indices){
    joint_list.emplace_back(joints[ind]);
  }
  return joint_list;
}

std::vector<std::shared_ptr<kodlab::JointBase>> kodlab::RobotBase::GetJoints(std::initializer_list<int> joint_indices){
  std::vector<int> joint_vect (joint_indices); 
  return kodlab::RobotBase::GetJoints(joint_vect);
}

template <size_t N>
std::vector<std::shared_ptr<kodlab::JointBase>> kodlab::RobotBase::GetJoints(std::array<int, N> joint_indices){
  std::vector<int> joint_vect (joint_indices.begin(), joint_indices.end()); 
  return kodlab::RobotBase::GetJoints(joint_vect);
}

std::vector<float> kodlab::RobotBase::GetJointTorqueCmd() { //Cop of torque_cmds
  std::vector<float>torques(torque_cmd_.begin(), torque_cmd_.end());
  return torques;
}

void kodlab::RobotBase::SetTorques(std::vector<float> torques) {
  soft_start_.ConstrainTorques(torques, run_timer_.tac()/1000.0);
  float torque_cmd;
  for (int joint_ind = 0; joint_ind < num_joints_; joint_ind++) {
    torque_cmd = joints[joint_ind]->UpdateTorque(torques[joint_ind]);
  }
}

void kodlab::RobotBase::AddLimb(std::shared_ptr<LimbBase> Limb) {
  limbs.push_back(Limb);
}

