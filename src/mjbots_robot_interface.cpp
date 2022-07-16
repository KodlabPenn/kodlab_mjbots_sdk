// BSD 3-Clause License
// Copyright (c) 2021 The Trustees of the University of Pennsylvania. All Rights Reserved
// Authors:
// Shane Rozen-Levy <srozen01@seas.upenn.edu>


#include "kodlab_mjbots_sdk/mjbots_robot_interface.h"

#include <iostream>
#include <algorithm>

namespace kodlab::mjbots {
void MjbotsRobotInterface::InitializeCommand() {
  for (const auto &joint : joints_moteus_) {
    commands_.push_back({});
    commands_.back().id = joint->get_can_id(); //id
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
                                           float robot_max_torque,
                                           ::mjbots::pi3hat::Euler imu_mounting_deg,
                                           int imu_rate_hz)
                                           : MjbotsRobotInterface(make_share_vector(joint_list),realtime_params,soft_start_duration,robot_max_torque,imu_mounting_deg,imu_rate_hz){}

MjbotsRobotInterface::MjbotsRobotInterface(std::vector<std::shared_ptr<JointMoteus>> joint_ptrs,
                                           const RealtimeParams &realtime_params,
                                           int soft_start_duration,
                                           float robot_max_torque,
                                           ::mjbots::pi3hat::Euler imu_mounting_deg,
                                           int imu_rate_hz) 
                                           :  kodlab::RobotInterface( joint_ptrs, 
                                                                      robot_max_torque,
                                                                      soft_start_duration)
  { 
  joints_moteus_ = joint_ptrs;

  for( auto & j: joints_moteus_){
    modes_.push_back(j->get_mode_reference());
  }
  
  for (size_t i = 0; i < num_joints_; ++i)
    servo_bus_map_[joints_moteus_[i]->get_can_id()] = joints_moteus_[i]->get_can_bus();

  // Create moteus interface
  ::mjbots::moteus::Pi3HatMoteusInterface::Options moteus_options;
  moteus_options.cpu = realtime_params.can_cpu;
  moteus_options.realtime_priority = realtime_params.can_rtp;
  moteus_options.servo_bus_map = servo_bus_map_;
  moteus_options.attitude_rate_hz = imu_rate_hz;
  moteus_options.imu_mounting_deg = imu_mounting_deg;
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
  for (auto & joint : joints_moteus_) {
    const auto servo_reply = Get(replies_, joint->get_can_id());
    if(std::isnan(servo_reply.position)){
      std::cout<<"Missing can frame for servo: " << joint->get_can_id()<< std::endl;
    } else{
      joint->UpdateMoteus(servo_reply.position, servo_reply.velocity, servo_reply.mode);
    }
  }
  attitude_ = *(moteus_data_.attitude);
}

void MjbotsRobotInterface::SendCommand() {
  cycle_count_++;
  
  for (int servo=0; servo < num_joints_;servo++) {// TODO: Move to a seperate update method (allow non-ff torque commands)?
    commands_[servo].position.feedforward_torque = torque_cmd_[servo];
  }

  moteus_interface_->Cycle(moteus_data_);
}

std::vector<::mjbots::moteus::Mode> MjbotsRobotInterface::GetJointModes() {
  std::vector<::mjbots::moteus::Mode>modes(modes_.begin(), modes_.end());
  return modes;
}

void MjbotsRobotInterface::SetModeStop() {
  for (auto &cmd : commands_) {
    cmd.mode = ::mjbots::moteus::Mode::kStopped;
  }
}

void MjbotsRobotInterface::Stop() {
  // Send a few stop commands
  ProcessReply();
  SetModeStop();
  SendCommand();
  ProcessReply();
  SendCommand();
  ProcessReply();
}

void MjbotsRobotInterface::Shutdown() {
  moteus_interface_->shutdown();
}
::mjbots::pi3hat::Attitude MjbotsRobotInterface::GetAttitude() {
  return  attitude_;
}
} // namespace kodlab::mjbots
