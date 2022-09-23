/**
 * @file limb_base.cpp
 * @author Kodlab - Zac Gong (zacgong@seas.upenn.edu)
 * @brief Class implementation for LimbBase.
 * @date 2022-08-18
 *
 * @copyright BSD 3-Clause License, Copyright (c) 2021 The Trustees of the
 * University of Pennsylvania. All Rights Reserved
 *
 */

#include "kodlab_mjbots_sdk/limb_base.h"

namespace kodlab {
    
LimbBase::LimbBase(const std::string &name,
                   const std::vector<std::shared_ptr<JointBase>> &joints,
                   const LimbConfig &config) {
  joints_ = joints;
  config_ = config;
  legDOFs_ = joints.size();
}

LimbBase::LimbBase(const std::vector<std::shared_ptr<JointBase>> &joints,
                   const LimbConfig &config) {
  joints_ = joints;
  config_ = config;
}

void LimbBase::Update(const std::vector<float> &pos_list,
                      const std::vector<float> &vel_list) {
  for (int i = 0; i < joints_.size(); i++) {
    // update position/velocity/torque vectors
    positions_[i] = joints_[i]->get_position();
    velocities_[i] = joints_[i]->get_velocity();
    torques_[i] = joints_[i]->get_servo_torque();
  }

  LimbBase::set_positions(pos_list);
  LimbBase::set_velocities(vel_list);
}

std::vector<float> LimbBase::get_positions() { return positions_; }

std::vector<float> LimbBase::get_velocities() { return velocities_; }

std::vector<float> LimbBase::get_torques() { return torques_; }

void LimbBase::set_positions(const std::vector<float> &positions) {
  for (int i = 0; i < joints_.size(); i++) {
    joints_[i]->set_position(positions[i]);
  }
}

void LimbBase::set_velocities(const std::vector<float> &velocities) {
  for (int i = 0; i < joints_.size(); i++) {
    joints_[i]->set_velocity(velocities[i]);
  }
}

}  // namespace kodlab
