/**
 * @file behavior_manager.h
 * @author Ethan J. Musser (emusser@seas.upenn.edu)
 * @brief Container for managing the `Behaviors` available to a robot and
 *        switching between them
 * @date 7/26/22
 * 
 * @copyright Copyright 2022 The Trustees of the University of Pennsylvania. All
 *            rights reserved.
 * 
 */

#pragma once

#include <iostream>
#include <memory>
#include "kodlab_mjbots_sdk/robot_base.h"
#include "kodlab_mjbots_sdk/behavior.h"
#include "kodlab_mjbots_sdk/off_behavior.h"
#include "kodlab_mjbots_sdk/log.h"

namespace kodlab
{

/**
 * @brief Manager for `Behavior` objects
 * @details The behavior manager class is a container for storing numerous
 *          behaviors for a given robot interface. It manages their selection
 *          and transitions between them.
 * @note By default, this class initializes its internal behaviors list with a
 *       `kodlab::Offbehavior`. This is done for safety purposes so that, when
 *       turned on, the robot does not command some undefined torques. This
 *       default can be overwritten with the `SetDefaultBehavior` member.
 * @tparam Robot[optional] derived `kodlab::RobotBase` class
 */
template<class Robot = kodlab::RobotBase>
class BehaviorManager
{
  static_assert(std::is_base_of<kodlab::RobotBase, Robot>::value,
                "Robot must have base kodlab::RobotBase.");

private:
  /**
   * @brief Robot that behaviors run on
   */
  std::shared_ptr<Robot> robot_;

  /**
   * @brief Behavior command that kills robot, default -1
   */
  static const int KILL_ROBOT_IDX = -1;

  /**
   * @brief Behaviors available to the robot
   * @note The first element of this vector is always `OFF_BEHAVIOR`.  Added
   *       behaviors are appended to the end.
   */
  std::vector<std::unique_ptr<Behavior<Robot>>> behaviors_;

  /**
   * @brief Names of behaviors in `behaviors_`
   */
  std::vector<std::string> names_;

  /**
   * @brief Index in `behaviors_` of the selected behavior
   */
  unsigned int selected_idx = 0;

  /**
   * @brief Index in `behaviors_` of the incoming behavior while transitioning
   *        behaviors
   */
  int next_idx_ = 0;

  /**
   * @brief Indicates whether a behavior switch is in progress
   */
  bool switching_behaviors_ = false;

public:
  /**
   * @brief Construct a behavior manager object
   * @param robot pointer to robot that the behavior is executing on
   */
  BehaviorManager(std::shared_ptr<Robot> robot) : robot_(robot) {}

  /**
   * @brief Destructor
   */
  virtual ~BehaviorManager() {}

  /**
   * @brief Set the current behavior
   * @param next_idx index in `behaviors_`
   */
  virtual void SetBehavior(const int &next_idx);

  /**
   * @brief Add a behavior to the behaviors list and initialize it
   * @tparam BehaviorType `kodlab::Behavior`-derived type
   * @tparam ConstructorArgs constructor arguments
   * @param args behavior constructor arguments
   */
  template<class BehaviorType, typename... ConstructorArgs>
  void AddBehavior(ConstructorArgs &&... args);

  /**
   * @brief Reset the behaviors list back to default
   */
  void ResetBehaviors();

  /**
   * @brief Update the current behavior
   * @param robot shared pointer to robot behavior is executing on
   */
  virtual void Update();

  /**
   * @brief Accessor for index of selected behavior
   * @return selected behavior index
   */
  [[nodiscard]] unsigned int get_selected_behavior() const
  {
    return selected_idx;
  }

  /**
   * @brief Accessor for names of behaviors in `behaviors_`
   * @param behavior_idx index of behavior
   * @return name of behavior at index `behavior_idx` in `behaviors_`
   */
  [[nodiscard]] std::string get_behavior_name(int behavior_idx) const
  {
    return behaviors_[behavior_idx]->get_name();
  }

  /**
   * @brief Accessor for name of selected behavior
   * @return selected behavior name
   */
  [[nodiscard]] std::string get_sel_behavior_name() const
  {
    return get_behavior_name(selected_idx);
  }

  /**
   * @brief Accessor for all behavior names in `behaviors_`
   * @return vector of names of behaviors
   */
  [[nodiscard]] std::vector<std::string> get_behavior_list() const
  {
    return names_;
  }

};

} // kodlab

