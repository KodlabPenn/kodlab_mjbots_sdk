// BSD 3-Clause License
// Copyright (c) 2021 The Trustees of the University of Pennsylvania. All Rights Reserved
// Authors:
// Shane Rozen-Levy <srozen01@seas.upenn.edu>


#include "kodlab_mjbots_sdk/mjbots_robot_interface.h"

#include "iostream"

namespace kodlab::mjbots {
void MjbotsRobotInterface::InitializeCommand() {
  for (const auto &id : servo_id_list_) {
    commands_.push_back({});
    commands_.back().id = id; //id
  }

  ::mjbots::moteus::PositionResolution res; // This is just for the command
  res.position = ::mjbots::moteus::Resolution::kIgnore; //Can be 0
  res.velocity = ::mjbots::moteus::Resolution::kIgnore; //Can be 0
  res.feedforward_torque = ::mjbots::moteus::Resolution::kInt16;
  res.kp_scale = ::mjbots::moteus::Resolution::kIgnore; //Can be 0 iff kp is set to 0 on motors
  res.kd_scale = ::mjbots::moteus::Resolution::kIgnore; //Can be 0 iff kd is set to 0 on motors during config
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

MjbotsRobotInterface::MjbotsRobotInterface(const std::vector<Motor> &motor_list,
                                           const RealtimeParams &realtime_params,
                                           float max_torque,
                                           int soft_start_duration) :
    soft_start_(max_torque, soft_start_duration) {
  // Process motor list
  num_servos_ = motor_list.size();
  for (const auto &motor_elem : motor_list) {
    servo_id_list_.push_back(motor_elem.id);
    servo_bus_list_.push_back(motor_elem.can_bus);
    offsets_.push_back(motor_elem.offset);
    directions_.push_back(motor_elem.direction);
  }
  for (size_t i = 0; i < num_servos_; ++i)
    servo_bus_map_[servo_id_list_[i]] = servo_bus_list_[i];

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
  SendCommand();

  // Prepare variables for saving response and process reply
  for (int servo = 0; servo < num_servos_; servo++) {
    positions_.push_back(0);
    velocities_.push_back(0);
    torque_cmd_.push_back(0);
    modes_.push_back(::mjbots::moteus::Mode::kStopped);
  }
  ProcessReply();

  // Setup message for basic torque commands
  PrepareTorqueCommand();
}

void MjbotsRobotInterface::ProcessReply() {

  // Make sure the m_can_result is valid before waiting otherwise undefined behavior
  if (can_result_.valid()) {
    can_result_.wait();
  }

  // Copy results to object so controller can use
  for (int servo = 0; servo < num_servos_; servo++) {
    const auto servo_reply = Get(replies_, servo_id_list_[servo]);

    positions_[servo] = directions_[servo] * (servo_reply.position * 2 * M_PI) + offsets_[servo];
    velocities_[servo] = directions_[servo] * (servo_reply.velocity * 2 * M_PI);
    modes_[servo] = servo_reply.mode;
  }
  timeout_ = moteus_data_.timeout;
  if (timeout_) {
    std::cout << "Error, pi3 hat timeout" << std::endl;
  }
}

void MjbotsRobotInterface::SendCommand() {
  cycle_count_++;
  auto promise = std::make_shared<std::promise<::mjbots::moteus::Pi3HatMoteusInterface::Output>>();
  moteus_interface_->Cycle(
      moteus_data_,
      [promise](const ::mjbots::moteus::Pi3HatMoteusInterface::Output &output) {
        // This is called from an arbitrary thread, so we just set
        // the promise value here.
        promise->set_value(output);
      });
  can_result_ = promise->get_future();
}

void MjbotsRobotInterface::SetTorques(std::vector<float> torques) {
  soft_start_.ConstrainTorques(torques, cycle_count_);
  torque_cmd_ = torques;

  for (int servo = 0; servo < num_servos_; servo++) {
    commands_[servo].position.feedforward_torque = directions_[servo] * (torques[servo]);
  }
}

std::vector<float> MjbotsRobotInterface::GetJointPositions() {
  return positions_;
}

std::vector<float> MjbotsRobotInterface::GetJointVelocities() {
  return velocities_;
}

std::vector<::mjbots::moteus::Mode> MjbotsRobotInterface::GetJointModes() {
  return modes_;
}

std::vector<float> MjbotsRobotInterface::GetJointTorqueCmd() {
  return torque_cmd_;
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