/**
 * @file joint_moteus.cpp
 * @author J. Diego Caporale (jdcap@seas.upenn.edu), Lucien Peach 
 * (peach@seas.upenn.edu), Shane Rozen-Levy (srozen01@seas.upenn.edu)
 * @brief Moteus powered joint class implementation
 * @date 2023-07-13
 * 
 * @copyright BSD 3-Clause License, Copyright (c) 2021 The Trustees of the University of Pennsylvania. All Rights Reserved
 * 
 */


#include "kodlab_mjbots_sdk/joint_moteus.h"

namespace kodlab{
namespace mjbots{

// kDefaultQuery defintion
const ::mjbots::moteus::QueryCommand JointMoteus::kDefaultQuery{
  ::mjbots::moteus::Resolution::kInt8,
  ::mjbots::moteus::Resolution::kInt16,
  ::mjbots::moteus::Resolution::kInt16,
  ::mjbots::moteus::Resolution::kIgnore,
  ::mjbots::moteus::Resolution::kIgnore,
  ::mjbots::moteus::Resolution::kIgnore,
  ::mjbots::moteus::Resolution::kIgnore,
  ::mjbots::moteus::Resolution::kIgnore,
  ::mjbots::moteus::Resolution::kIgnore,
  ::mjbots::moteus::Resolution::kIgnore
};

// kTorqueQuery definition
const ::mjbots::moteus::QueryCommand JointMoteus::kTorqueQuery{
  ::mjbots::moteus::Resolution::kInt8,
  ::mjbots::moteus::Resolution::kInt16,
  ::mjbots::moteus::Resolution::kInt16,
  ::mjbots::moteus::Resolution::kInt32,
  ::mjbots::moteus::Resolution::kIgnore,
  ::mjbots::moteus::Resolution::kIgnore,
  ::mjbots::moteus::Resolution::kIgnore,
  ::mjbots::moteus::Resolution::kIgnore,
  ::mjbots::moteus::Resolution::kIgnore,
  ::mjbots::moteus::Resolution::kIgnore
};

// kDebugQuery definition
const ::mjbots::moteus::QueryCommand JointMoteus::kDebugQuery{
  ::mjbots::moteus::Resolution::kInt8,
  ::mjbots::moteus::Resolution::kInt16,
  ::mjbots::moteus::Resolution::kInt16,
  ::mjbots::moteus::Resolution::kInt32,
  ::mjbots::moteus::Resolution::kIgnore,
  ::mjbots::moteus::Resolution::kIgnore,
  ::mjbots::moteus::Resolution::kIgnore,
  ::mjbots::moteus::Resolution::kIgnore,
  ::mjbots::moteus::Resolution::kInt16,
  ::mjbots::moteus::Resolution::kInt8
};

// kComprehensiveQuery definition
const ::mjbots::moteus::QueryCommand JointMoteus::kComprehensiveQuery{
  ::mjbots::moteus::Resolution::kInt8,
  ::mjbots::moteus::Resolution::kInt16,
  ::mjbots::moteus::Resolution::kInt16,
  ::mjbots::moteus::Resolution::kInt32,
  ::mjbots::moteus::Resolution::kInt16,
  ::mjbots::moteus::Resolution::kInt16,
  ::mjbots::moteus::Resolution::kIgnore,
  ::mjbots::moteus::Resolution::kInt16,
  ::mjbots::moteus::Resolution::kInt16,
  ::mjbots::moteus::Resolution::kInt8
};

JointMoteus::JointMoteus(std::string name,
                         int can_id,
                         int can_bus,
                         int direction,
                         float zero_offset,
                         float gear_ratio,
                         float max_torque,
                         float pos_min,
                         float pos_max,
                         float soft_start_duration_ms,
                         ::mjbots::moteus::QueryCommand kDefaultQuery,
                         float moteus_kp,
                         float moteus_kd)
    : JointBase(name, direction, zero_offset, gear_ratio, max_torque, pos_min, pos_max, soft_start_duration_ms),
      can_bus_(can_bus),
      can_id_(can_id),
      query_type_(kDefaultQuery),
      moteus_kp_(moteus_kp),
      moteus_kd_(moteus_kd) {}

JointMoteus::JointMoteus(int can_id,
                         int can_bus,
                         int direction,
                         float zero_offset,
                         float gear_ratio,
                         float max_torque,
                         float pos_min,
                         float pos_max,
                         float soft_start_duration_ms,
                         ::mjbots::moteus::QueryCommand kDefaultQuery,
                         float moteus_kp,
                         float moteus_kd)
    : JointBase("", direction, zero_offset, gear_ratio, max_torque, pos_min, pos_max, soft_start_duration_ms),
      can_bus_(can_bus),
      can_id_(can_id),
      query_type_(kDefaultQuery),
      moteus_kp_(moteus_kp),
      moteus_kd_(moteus_kd) {}

JointMoteus::JointMoteus(MoteusJointConfig config)
    : JointBase(config.name, 
                config.direction,
                config.zero_offset,
                config.gear_ratio,
                config.max_torque,
                config.pos_min,
                config.pos_max,
                config.soft_start_duration_ms),
        can_id_(config.can_id),
        can_bus_(config.can_bus),
        query_type_(config.query_type),
        moteus_kp_(config.moteus_kp),
        moteus_kd_(config.moteus_kd) {}
        
void JointMoteus::UpdateMoteus(::mjbots::moteus::QueryResult reply_message){
                    UpdateState(2 * M_PI * reply_message.position, 2 * M_PI * reply_message.velocity, reply_message.torque);
                    mode_ = reply_message.mode;
                    q_current_ = reply_message.q_current;
                    d_current_ = reply_message.d_current;
                    voltage_ = reply_message.voltage;
                    temperature_ = reply_message.temperature;
                    fault_ = reply_message.fault;
}

bool JointMoteus::is_open_loop() {
  bool rv;
  if(query_type_.position == ::mjbots::moteus::Resolution::kIgnore ||
  query_type_.velocity == ::mjbots::moteus::Resolution::kIgnore){
    rv = true;
  } else {
    rv = false;
  }
  return rv;
}

int JointMoteus::get_can_id() const {
  return can_id_;
}

int JointMoteus::get_can_bus() const {
  return can_bus_;
}

const ::mjbots::moteus::QueryCommand JointMoteus::get_query_command() const {
  return query_type_;
}

const ::mjbots::moteus::Mode & JointMoteus::get_mode_reference() const {
  if(query_type_.mode == ::mjbots::moteus::Resolution::kIgnore){
    LOG_WARN("Mode resolution is set to kIgnore, while attempting to log register");
  }
  return mode_;
}

const ::mjbots::moteus::Mode JointMoteus::get_mode() const {
  if(query_type_.mode == ::mjbots::moteus::Resolution::kIgnore){
    LOG_WARN("Mode resolution is set to kIgnore, while attempting to log register");
  }
  return mode_;
}

float JointMoteus::get_position() const {
  if(query_type_.position == ::mjbots::moteus::Resolution::kIgnore){
    LOG_WARN("Position resolution is set to kIgnore, while attempting to log register");
    return 0;
  } else {
    return position_;
  }
}

float JointMoteus::get_velocity() const {
  if(query_type_.velocity == ::mjbots::moteus::Resolution::kIgnore){
    LOG_WARN("Velocity resolution is set to kIgnore, while attempting to log register");
    return 0;
  } else {
    return velocity_;
  }
}

float JointMoteus::get_servo_torque() const {
  if(query_type_.torque == ::mjbots::moteus::Resolution::kIgnore){
    LOG_WARN("Torque resolution is set to kIgnore, while attempting to log register");
    return 0;
  } else {
    return servo_torque_;
  }
}

float JointMoteus::get_measured_torque() const {
  if(query_type_.torque == ::mjbots::moteus::Resolution::kIgnore){
    LOG_WARN("Torque resolution is set to kIgnore, while attempting to log register");
    return 0;
  } else {
    return measured_torque_;
  }
}

float JointMoteus::get_temperature() const {
  if(query_type_.temperature == ::mjbots::moteus::Resolution::kIgnore){
    LOG_WARN("Temperature resolution is set to kIgnore, while attempting to log register");
    return 0;
  } else {
    return temperature_;
  }
}

float JointMoteus::get_q_current() const {
  if(query_type_.q_current == ::mjbots::moteus::Resolution::kIgnore){
    LOG_WARN("Q Current resolution is set to kIgnore, while attempting to log register");
    return 0;
  } else {
    return q_current_;
  }
}

float JointMoteus::get_d_current() const {
  if(query_type_.d_current == ::mjbots::moteus::Resolution::kIgnore){
    LOG_WARN("D Current resolution is set to kIgnore, while attempting to log register");
    return 0;
  } else {
    return d_current_;
  }
}

float JointMoteus::get_voltage() const {
  if(query_type_.voltage == ::mjbots::moteus::Resolution::kIgnore){
    LOG_WARN("Voltage resolution is set to kIgnore, while attemping to log register");
    return 0;
  } else {
    return voltage_;
  }
}

const ::mjbots::moteus::Fault & JointMoteus::get_fault() const {
  if(query_type_.fault == ::mjbots::moteus::Resolution::kIgnore){
    LOG_WARN("Fault code resolution is set to kIgnore, while attempting to log register");
    return ::mjbots::moteus::Fault::kSuccess;
  } else {
    return fault_;
  }
}

[[nodiscard]] float JointMoteus::get_kp_scale() const {
  if(moteus_kp_ == 0){
    LOG_WARN("Moteus kp is set to 0, while attempting to send pd commands");
    return 0;
  }
  // Multiply by 2pi to convert kp from radians to revolutions
  // Divide by gear ratio to get servo kp rather than joint kp
  const float kp_scale = kp_/moteus_kp_ * 2 * M_PI/gear_ratio_;
  if(kp_scale > 1){
    LOG_WARN("kp_scale is greater than 1, will be limited to 1. Either use a lower kp or increase kp in the moteus");
    LOG_WARN("With the current moteus kp of %.3f, the max value of the joint kp is %.3f", moteus_kp_, kp_/kp_scale);
  }
  return kp_scale;
}

[[nodiscard]] float JointMoteus::get_kd_scale() const {
  if(moteus_kd_ == 0){
    LOG_WARN("Moteus kd is set to 0, while attempting to send pd commands");
    return 0;
  }
  const float kd_scale = kd_/moteus_kd_ * 2 * M_PI/gear_ratio_;
  if(kd_scale > 1){
    LOG_WARN("kd_scale is greater than 1, will be limited to 1. Either use a lower kd or increase kd in the moteus");
    LOG_WARN("With the current moteus kd of %.3f, the max value of the joint kd is %.3f", moteus_kp_, kd_/kd_scale);
  }
  return kd_scale;
}

[[nodiscard]] float JointMoteus::get_moteus_position_target() const {
  return (position_target_ + zero_offset_) * gear_ratio_/direction_/2/M_PI;
}

[[nodiscard]] float JointMoteus::get_moteus_velocity_target() const {
  return (velocity_target_) * gear_ratio_/direction_/2/M_PI;
}

} //namespace mjbots
} //namespace kodlab

