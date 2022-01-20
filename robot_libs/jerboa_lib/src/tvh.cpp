//
// Created by shane on 1/20/22.
//

#include <jerboa_lib/tvh.h>

#include <cmath>

void TVH::UpdateState(const std::shared_ptr<kodlab::mjbots::MjbotsRobotInterface>& robot) {
  tail_angle_ = robot->GetJointPositions()[0] / params_.tail_N;
  tail_speed_ = robot->GetJointVelocities()[0]/ params_.tail_N;
  leg_encoder_ = robot->GetJointPositions()[1];
  leg_encoder_speed_ = robot->GetJointVelocities()[1];
  legKinematics(leg_encoder_,leg_encoder_speed_);
}

void TVH::legKinematics(float LE, float dLE) {
  float theta = params_.encoder_offset + LE;
  leg_length_ = 2 * params_.femur_length * std::cos(theta)+params_.toe_length;
  leg_speed_ = - 2 * params_.femur_length * std::sin(theta) * dLE;
  leg_comp_ = params_.r0 - leg_length_;
}

void TVH::SetEffort(const std::shared_ptr<kodlab::mjbots::MjbotsRobotInterface>& robot, const float tail_torque) {
  robot->SetTorques({tail_torque/params_.tail_N});
}

float TVH::GetTailAngle() const {
  return tail_angle_;
}
float TVH::GetTailSpeed() const {
  return tail_speed_;
}
float TVH::GetLegCompression() const {
  return leg_comp_;
}
float TVH::GetLegLength() const {
  return leg_length_;
}
float TVH::GetLegSpeed() const {
  return leg_speed_;
}

