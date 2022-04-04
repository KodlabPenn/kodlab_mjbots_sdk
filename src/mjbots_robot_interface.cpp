// BSD 3-Clause License
// Copyright (c) 2021 The Trustees of the University of Pennsylvania. All Rights Reserved
// Authors:
// Shane Rozen-Levy <srozen01@seas.upenn.edu>


#include "kodlab_mjbots_sdk/mjbots_robot_interface.h"

#include "iostream"

namespace kodlab::mjbots {
void MjbotsRobotInterface::InitializeCommand() {
  for (const auto &joint : joints_) {
    commands_.push_back({});
    commands_.back().id = joint.get_can_id(); //id
  }

  ::mjbots::moteus::PositionResolution res; // This is just for the command
  res.position = ::mjbots::moteus::Resolution::kIgnore;
  res.velocity = ::mjbots::moteus::Resolution::kIgnore;
  res.feedforward_torque = ::mjbots::moteus::Resolution::kInt16;
  res.kp_scale = ::mjbots::moteus::Resolution::kIgnore;
  res.kd_scale = ::mjbots::moteus::Resolution::kIgnore;
  res.maximum_torque = ::mjbots::moteus::Resolution::kIgnore;
  res.stop_position = ::mjbots::moteus::Resolution::kIgnore;
  res.watchdog_timeout = ::mjbots::moteus::Resolution::kIgnore;
  for (auto &cmd : commands_) {
    cmd.resolution = res;
    cmd.mode = ::mjbots::moteus::Mode::kStopped;
  }
}

void MjbotsRobotInterface::PrepareTorqueCommand() {
  for (auto &cmd : commands_) {
    cmd.mode = ::mjbots::moteus::Mode::kPosition;
    cmd.position.kd_scale = 0;
    cmd.position.kp_scale = 0;
  }
}

::mjbots::moteus::QueryResult MjbotsRobotInterface::Get(const std::vector<::mjbots::moteus::Pi3HatMoteusInterface::ServoReply> &replies,
                                                      int id) {
  for (const auto &item : replies) {
    if (item.id == id) { return item.result; }
  }
  return {};
}

MjbotsRobotInterface::MjbotsRobotInterface(const std::vector<JointMoteus> &joint_list,
                                           const RealtimeParams &realtime_params,
                                           int soft_start_duration,
                                           float robot_max_torque ) :
                                            soft_start_(robot_max_torque, soft_start_duration) { 
  joints_ = std::move(joint_list);    
  for( auto & j: joints_){
    positions_.push_back( j.get_position_reference() );
    velocities_.push_back( j.get_velocity_reference() );
    torque_cmd_.push_back( j.get_servo_torque_reference()   ); 
    modes_.push_back(j.get_mode_reference());
  }
  num_servos_ = joint_list.size();
  for (size_t i = 0; i < num_servos_; ++i)
    servo_bus_map_[joint_list[i].get_can_id()] = joint_list[i].get_can_bus();

  // Create moteus interface
  ::mjbots::moteus::Pi3HatMoteusInterface::Options moteus_options;
  moteus_options.cpu = realtime_params.can_cpu;
  moteus_options.realtime_priority = realtime_params.can_rtp;
  moteus_options.servo_bus_map = servo_bus_map_;
  moteus_interface_ = std::make_shared<::mjbots::moteus::Pi3HatMoteusInterface>(moteus_options);

  // Initialize and send basic command
  InitializeCommand();
  replies_ = std::vector<::mjbots::moteus::Pi3HatMoteusInterface::ServoReply>{commands_.size()};
  moteus_data_.commands = {commands_.data(), commands_.size()};
  moteus_data_.replies = {replies_.data(), replies_.size()};
  moteus_data_.timeout = timeout_;

}


void MjbotsRobotInterface::Init() {
    SendCommand();
    ProcessReply();
    // Setup message for basic torque commands
    PrepareTorqueCommand();
    SendCommand();
    ProcessReply();
}

void MjbotsRobotInterface::ProcessReply() {

  // Make sure the m_can_result is valid before waiting otherwise undefined behavior
  moteus_interface_->WaitForCycle();
  // Copy results to object so controller can use
  for (auto & joint : joints_) {
    const auto servo_reply = Get(replies_, joint.get_can_id());
    joint.UpdateMoteus(servo_reply.position, servo_reply.velocity, servo_reply.mode);
  }
}

void MjbotsRobotInterface::SendCommand() {
  cycle_count_++;
  
  for (int servo=0; servo < num_servos_;servo++) {// TODO: Move to a seperate update method (allow non-ff torque commands)?
    commands_[servo].position.feedforward_torque = torque_cmd_[servo];
  }

  moteus_interface_->Cycle(moteus_data_);
}

void MjbotsRobotInterface::SetTorques(std::vector<float> torques) {
  soft_start_.ConstrainTorques(torques, cycle_count_);
  float torque_cmd;
  for (int servo = 0; servo < num_servos_; servo++) {
    torque_cmd = joints_[servo].UpdateTorque(torques[servo]);
    
  }
}

std::vector<float> MjbotsRobotInterface::GetJointPositions() { //Copy of positions
  std::vector<float>pos(positions_.begin(), positions_.end());
  return pos;
}

std::vector<float> MjbotsRobotInterface::GetJointVelocities() { //Copy of velocities
  std::vector<float>vel(velocities_.begin(), velocities_.end());
  return vel;
}

std::vector<::mjbots::moteus::Mode> MjbotsRobotInterface::GetJointModes() {
  std::vector<::mjbots::moteus::Mode>modes(modes_.begin(), modes_.end());
  return modes;
}

std::vector<float> MjbotsRobotInterface::GetJointTorqueCmd() {
  std::vector<float>torques(torque_cmd_.begin(), torque_cmd_.end());
  return torques;
}
void MjbotsRobotInterface::SetModeStop() {
  for (auto &cmd : commands_) {
    cmd.mode = ::mjbots::moteus::Mode::kStopped;
  }
}

void MjbotsRobotInterface::Shutdown() {
  moteus_interface_->shutdown();
}
} // namespace kodlab::mjbots
