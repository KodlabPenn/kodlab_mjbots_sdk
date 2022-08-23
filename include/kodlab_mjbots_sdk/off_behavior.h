/**
 * @file off_behavior.h
 * @author Ethan J. Musser (emusser@seas.upenn.edu)
 * @brief A simple behavior commanding zero torques on all joints in a derived
 *        `RobotBase`
 * @date 7/28/22
 * 
 * @copyright Copyright 2022 The Trustees of the University of Pennsylvania. All
 *            rights reserved.
 * 
 */

#pragma once

#include <vector>
#include "kodlab_mjbots_sdk/behavior.h"
#include "kodlab_mjbots_sdk/robot_base.h"

namespace kodlab
{

/**
 * @brief Off behavior commanding zero torques to all joints in a robot
 *        interface
 * @tparam Robot[optional] `RobotBase`-derived object
 */
template<class Robot = kodlab::RobotBase>
class OffBehavior : public Behavior<Robot>
{

public:
  /**
   * @brief Use Behavior constructor
   */
  using Behavior<Robot>::Behavior;

  /**
   * @brief Set joint torques to zero for a given robot interface
   * @note The default behavior for this function sets all joint torques to zero
   * @param robot robot interface
   */
  void Update() override
  {
    this->robot_->SetTorques(
        std::vector<float>(this->robot_->joints.size(), 0));
  }

};

} // kodlab
