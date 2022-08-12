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
   * @brief Behavior command that turns robot off, always 0
   */
  static const int DEFAULT_IDX = 0;

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
  BehaviorManager(std::shared_ptr<Robot> robot) : robot_(robot)
  {
    AddBehavior<OffBehavior<Robot>>(robot_, "OFF");
  }

  /**
   * @brief Destructor
   */
  virtual ~BehaviorManager() {}

  /**
   * @brief Set the current behavior
   * @param next_idx index in `behaviors_`
   */
  virtual void SetBehavior(const int &next_idx)
  {
    if (next_idx == KILL_ROBOT_IDX)
    {
      kodlab::CTRL_C_DETECTED = true; // kill robot
      LOG_FATAL("Behavior %d received, killing robot.", KILL_ROBOT_IDX);
    } else if (next_idx == selected_idx)
    {
      LOG_INFO("Behavior %d is already active.", next_idx);
      return;
    }
    else if (next_idx < behaviors_.size() && next_idx >= 0)
    {
      next_idx_ = next_idx; // update next behavior
      if (!behaviors_[next_idx_]->is_initialized())
      {
        behaviors_[next_idx_]->Init();
      }
      behaviors_[selected_idx]->Stop(*behaviors_[next_idx_]); // stop current behavior
      switching_behaviors_ = true;  // indicate transition is in progress
    } else
    {
      LOG_WARN("Invalid behavior selected: %d", next_idx);
    }
  }

  /**
   * @brief Constructs and initializes a behavior
   * @tparam BehaviorType `kodlab::Behavior`-derived type
   * @tparam ConstructorArgs constructor arguments
   * @param args behavior constructor arguments
   * @return unique pointer to the behavior if initialization was successful,
   *         nullptr otherwise
   */
  template<class BehaviorType, typename... ConstructorArgs>
  std::unique_ptr<BehaviorType> InitBehavior(ConstructorArgs &&... args)
  {
    static_assert(std::is_base_of<kodlab::Behavior<Robot>, BehaviorType>::value,
                  "BehaviorType must be a `kodlab::Behavior`-derived type");
    auto b = std::make_unique<BehaviorType>(args...);
    if (b->Init())
    {
      b->set_initialized();
      return std::move(b);
    } else { return nullptr; }
  }

  /**
   * @brief Add a behavior to the behaviors list and initialize it
   * @tparam BehaviorType `kodlab::Behavior`-derived type
   * @tparam ConstructorArgs constructor arguments
   * @param args behavior constructor arguments
   */
  template<class BehaviorType, typename... ConstructorArgs>
  void AddBehavior(ConstructorArgs &&... args)
  {
    static_assert(std::is_base_of<kodlab::Behavior<Robot>, BehaviorType>::value,
                  "BehaviorType must be a `kodlab::Behavior`-derived type");
    auto b = InitBehavior<BehaviorType>(args...);
    if (b)
    {
      names_.push_back(b->get_name());
      behaviors_.emplace_back(std::move(b));
    } else
    {
      LOG_WARN(
          "%s behavior could not be initialized and will not be added to behaviors list.",
          b->get_name().c_str());
    }
  }

  /**
   * @brief Replaces the default behavior with a user-defined one
   * @note This is included to provide support for custom default robot
   *       behaviors. If this function is not called, the default behavior will
   *       be a `kodlab::OffBehavior`.
   * @tparam BehaviorType `kodlab::Behavior`-derived type
   * @tparam ConstructorArgs constructor arguments
   * @param args behavior constructor arguments
   */
  template<class BehaviorType, typename... ConstructorArgs>
  void SetDefaultBehavior(ConstructorArgs &&... args)
  {
    static_assert(std::is_base_of<kodlab::Behavior<Robot>, BehaviorType>::value,
                  "BehaviorType must be a `kodlab::Behavior`-derived type");
    auto b = InitBehavior<BehaviorType>(args...);
    if (b)
    {
      names_.front() = b->get_name();
      behaviors_.front() = std::move(b);
    } else
    {
      LOG_WARN(
          "%s behavior could not be initialized and will not be set as default behavior.",
          b->get_name().c_str());
    }
  }

  /**
   * @brief Reset the behaviors list back to default
   */
  void ResetBehaviors()
  {
    if (behaviors_.size() > 1)
    {
      behaviors_.erase(behaviors_.begin() + 1, behaviors_.end());
      names_.erase(names_.begin() + 1, names_.end());
    }
  }

  /**
   * @brief Update the current behavior
   * @param robot shared pointer to robot behavior is executing on
   */
  virtual void Update()
  {
    behaviors_[selected_idx]->ProcessInput();
    behaviors_[selected_idx]->Update();
    behaviors_[selected_idx]->ProcessOutput();
    if (switching_behaviors_
        && behaviors_[selected_idx]->ReadyToSwitch(*behaviors_[next_idx_]))
    {
      behaviors_[selected_idx]->set_inactive();  // mark previous behavior as inactive
      behaviors_[next_idx_]->Begin(*behaviors_[selected_idx]);  // begin new behavior
      behaviors_[next_idx_]->set_active();  // mark new behavior as active
      selected_idx = next_idx_;  // update selected behavior to next
      switching_behaviors_ = false;  // no longer transitioning behaviors
      std::fprintf(stdout,
                   "[INFO][BehaviorManager] Switched to behavior %d.\n",
                   selected_idx);
    }
  }

  /**
   * @brief Prints a list of behaviors currently in `behaviors_`
   * @param stream output stream
   */
  void PrintBehaviorList(FILE *stream = stdout) const
  {
    std::fprintf(stream, "+--------------------------------+\n");
    std::fprintf(stream, "| Control Loop Behaviors         |\n");
    std::fprintf(stream, "| ----------------------         |\n");
    for (unsigned int i = 0; i < names_.size(); i++)
    {
      std::fprintf(stream, "| %3d. %-25s |\n",
                   i, names_[i].c_str());
    }
    std::fprintf(stream, "+--------------------------------+\n");
  }

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
