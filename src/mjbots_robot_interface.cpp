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
  res.position = ::mjbots::moteus::Resolution::kInt16;
  res.velocity = ::mjbots::moteus::Resolution::kInt16;
  res.feedforward_torque = ::mjbots::moteus::Resolution::kInt16;
  res.kp_scale = ::mjbots::moteus::Resolution::kInt16;
  res.kd_scale = ::mjbots::moteus::Resolution::kInt16;
  res.maximum_torque = ::mjbots::moteus::Resolution::kInt8;
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
    cmd.position.maximum_torque = 100;
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
                                           const std::vector<ExternalEncoder> &encoder_list,
                                           const RealtimeParams &realtime_params,
                                           float max_torque,
                                           int soft_start_duration):MjbotsRobotInterface(motor_list,
                                                                                         realtime_params,
                                                                                         max_torque,
                                                                                         soft_start_duration) {
  num_external_encoders_ = encoder_list.size();
  for(const auto &encoder_elem : encoder_list){
    offsets_.push_back(encoder_elem.offset);
    directions_.push_back(encoder_elem.direction);
    encoder_cs_list_.push_back(encoder_elem.cs);
    encoder_alpha_.push_back(encoder_elem.alpha);
    encoder_speed_alpha_.push_back(encoder_elem.speed_alpha);
    positions_.push_back(0);
    velocities_.push_back(0);
    torque_cmd_.push_back(0);
    torque_measured_.push_back(0);
    raw_encoder_positions_.push_back(0);
    raw_encoder_velocities_.push_back(0);
    modes_.push_back(::mjbots::moteus::Mode::kPosition);

    positions_target_.push_back(0);
    velocity_target_.push_back(0);
    kp_.push_back(0);
    kd_.push_back(0);
    max_torques_.push_back(max_torque);
    encoder_wrap_.push_back(0);
  }

  for(int encoder = 0; encoder < num_external_encoders_; encoder++){
    // Apply offsets and direction
    float raw_position = directions_[num_servos_ + encoder] *
        moteus_interface_->pi3hat_->readEncoder(encoder_cs_list_[encoder]) +
        offsets_[num_servos_ + encoder];

    // Initialize filter
    positions_[num_servos_ + encoder] = raw_position;
  }

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
  moteus_data_.timeout = timeout_;
  SendCommand();

  // Prepare variables for saving response and process reply
  for (int servo = 0; servo < num_servos_; servo++) {
    positions_.push_back(0);
    velocities_.push_back(0);
    torque_cmd_.push_back(0);
    torque_measured_.push_back(0);
    modes_.push_back(::mjbots::moteus::Mode::kStopped);
    positions_target_.push_back(0);
    velocity_target_.push_back(0);
    kp_.push_back(0);
    kd_.push_back(0);
    max_torques_.push_back(max_torque);
  }
  ProcessReply();

  // Setup message for basic torque commands
  PrepareTorqueCommand();
  SendCommand();
  ProcessReply();
}

void MjbotsRobotInterface::ProcessReply() {

  // Make sure the m_can_result is valid before waiting otherwise undefined behavior
  moteus_interface_->WaitForCycle();

  for(int encoder = 0; encoder < num_external_encoders_; encoder++){
    float measurment = moteus_interface_->pi3hat_->readEncoder(encoder_cs_list_[encoder]);

    if (measurment == 0 && encoder == 1){
      std::cout<<"Catching zero"<<std::endl;
      raw_encoder_positions_[encoder] += raw_encoder_velocities_[encoder] /1000;
    } else{
      // Apply offsets and direction
      float raw_position = directions_[num_servos_ + encoder] *measurment
          + offsets_[num_servos_ + encoder] + encoder_wrap_[encoder];

      // Calculate raw velocity
      if (raw_position - raw_encoder_positions_[encoder] > 1){
        if (encoder == 1){
          std::cout<<"Wrapping. Prev = "<<raw_encoder_positions_[encoder] << " new = "<< raw_position;
        }
        raw_position-=M_PI * 2;
        encoder_wrap_[encoder]-=M_PI*2;
        if (encoder == 1) {
          std::cout << " Update = " << raw_position << std::endl;
          std::cout << "Measurment" << measurment << std::endl;
        }
      }else if (raw_encoder_positions_[encoder]-raw_position > 1){
        if (encoder == 1){
          std::cout<<"Wrapping. Prev = "<<raw_encoder_positions_[encoder] << " new = "<< raw_position;
        }
        raw_position+=M_PI * 2;
        encoder_wrap_[encoder]+=M_PI*2;
        if (encoder == 1) {
          std::cout << " Update = " << raw_position << std::endl;
          std::cout << "Measurment" << measurment << std::endl;
        }
      }
      raw_encoder_velocities_[encoder] = (raw_position - raw_encoder_positions_[encoder])* 1000;
      raw_encoder_positions_[encoder] = raw_position;
    }
    // Filter position and velocity
    positions_[num_servos_ + encoder] = (1 - encoder_alpha_[encoder]) * positions_[num_servos_ + encoder] +
        encoder_alpha_[encoder] * raw_encoder_positions_[encoder];
    velocities_[num_servos_ + encoder] = (1 - encoder_speed_alpha_[encoder]) * velocities_[num_servos_ + encoder] +
        encoder_speed_alpha_[encoder] * raw_encoder_velocities_[encoder];
  }

  // Copy results to object so controller can use
  for (int servo = 0; servo < num_servos_; servo++) {
    const auto servo_reply = Get(replies_, servo_id_list_[servo]);
    if(std::isnan(servo_reply.position)){
      std::cout<<"Missing can frame for servo: " << servo_id_list_[servo]<< std::endl;
    }else {
      positions_[servo] = directions_[servo] * (servo_reply.position * 2 * M_PI) + offsets_[servo];
      velocities_[servo] = directions_[servo] * (servo_reply.velocity * 2 * M_PI);
      modes_[servo] = servo_reply.mode;
      torque_measured_[servo] = servo_reply.torque * directions_[servo];
    }
  }
  attitude_ = *(moteus_data_.attitude);
}

void MjbotsRobotInterface::SendCommand() {
  cycle_count_++;
  moteus_interface_->Cycle(
      moteus_data_);
}

void MjbotsRobotInterface::SetTorques(std::vector<float> torques) {
  soft_start_.ConstrainTorques(torques, cycle_count_);

  for (int servo = 0; servo < num_servos_; servo++) {
    torque_cmd_[servo] = torques[servo];
    commands_[servo].position.feedforward_torque = directions_[servo] * (torque_cmd_[servo]);
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

std::vector<float> MjbotsRobotInterface::GetEncoderRawPositions() {
  return raw_encoder_positions_;
}

std::vector<float> MjbotsRobotInterface::GetEncoderRawVelocities() {
  return raw_encoder_velocities_;
}
std::vector<float> MjbotsRobotInterface::GetJointTorqueMeasured() {
  return torque_measured_;
}

::mjbots::pi3hat::Attitude MjbotsRobotInterface::GetAttitude(){
  return  attitude_;
}
void MjbotsRobotInterface::SetPDGains(std::vector<float> kp, std::vector<float> kd) {
  for (int servo = 0; servo < num_servos_; servo++) {
    kp_[servo] = kp[servo];
    kd_[servo] = kd[servo];

    commands_[servo].position.kp_scale = kp_[servo]; // Converting between rad and cycles
    commands_[servo].position.kd_scale = kd_[servo]; // Converting between Hz and rad/s
  }
}
void MjbotsRobotInterface::SetPDTarget(std::vector<float> position, std::vector<float> velocity) {
  for (int servo = 0; servo < num_servos_; servo++) {
    positions_target_[servo] = position[servo];
    velocity_target_[servo] = velocity[servo];

    commands_[servo].position.position = (position[servo] - offsets_[servo])/directions_[servo]/2/M_PI;
    commands_[servo].position.velocity = velocity[servo]/directions_[servo]/2/M_PI;
  }

}
void MjbotsRobotInterface::SetJointTorqueLimit(std::vector<float> torque) {
  for (int servo = 0; servo < num_servos_; servo++) {
    max_torques_[servo] = torque[servo];

    commands_[servo].position.maximum_torque = torque[servo];
  }
}
} // namespace kodlab::mjbots
