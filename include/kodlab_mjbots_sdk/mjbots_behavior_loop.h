/**
 * @file mjbots_behavior_loop.h
 * @author Ethan J. Musser (emusser@seas.upenn.edu)
 * @brief An MjbotsControlLoop with added behavior support.
 * @date 7/26/22
 * 
 * @copyright 2022 The Trustees of the University of Pennsylvania. All rights reserved.
 * 
 */

#pragma once

#include "kodlab_mjbots_sdk/mjbots_control_loop.h"
#include "kodlab_mjbots_sdk/behavior_manager.h"
#include "kodlab_mjbots_sdk/behavior.h"
#include "kodlab_mjbots_sdk/off_behavior.h"
#include "kodlab_mjbots_sdk/log.h"

namespace kodlab::mjbots
{

/**
 * @brief
 * @tparam Log[optional] data type for logging
 * @tparam Input[optional] class for input data
 * @tparam Robot[optional] `RobotBase`-Derived class that contains state and
 *                         control calculations
 */
template<class Log = VoidLcm, class Input = VoidLcm,
    class Robot = kodlab::RobotBase>
class MjbotsBehaviorLoop : public MjbotsControlLoop<Log, Input, Robot>
{

protected:
  /**
   * @brief Updates robot state and sets torques based on active behavior in
   *        behavior manager
   */
  void Update() override
  {
    this->robot_->Update();
    behavior_mgr.Update();
  }

public:
  /**
   * @brief Construct an Mjbots behavior loop based on an options struct
   * @note Does not Start the controller.
   * @param joints vector of JointMoteus
   * @param options options defining the behavior
   */
  MjbotsBehaviorLoop(std::vector<kodlab::mjbots::JointMoteus> joints,
                     const ControlLoopOptions &options)
      : MjbotsControlLoop<Log, Input, Robot>(joints, options),
        behavior_mgr{this->robot_} {}

  /**
   * @brief Construct an Mjbots behavior loop based on an options struct
   * @note Does not Start the controller.
   * @param joints vector of JointMoteus shared pointers
   * @param options options defining the behavior
   */
  MjbotsBehaviorLoop(std::vector<std::shared_ptr<kodlab::mjbots::JointMoteus>> joints,
                     const ControlLoopOptions &options)
      : MjbotsControlLoop<Log, Input, Robot>(joints, options),
        behavior_mgr{this->robot_} {}

  /**
   * @brief Construct an Mjbots behavior loop based on an options struct
   * @note Does not Start the controller.
   * @param robot_in instance of a derived RobotBase
   * @param options options defining the behavior
   */
  MjbotsBehaviorLoop(std::shared_ptr<Robot> robot_in,
                     const ControlLoopOptions &options)
      : MjbotsControlLoop<Log, Input, Robot>(robot_in, options),
        behavior_mgr{this->robot_} {}

  /**
   * @brief Destructor
   */
  virtual ~MjbotsBehaviorLoop() {}

  /**
   * @brief Behavior manager handling robot behaviors and transitions between
   *        robot behaviors
   */
  BehaviorManager<Robot> behavior_mgr;

  /**
   * @brief Wrapper for behavior manager AddBehavior function
   * @tparam BehaviorType `kodlab::Behavior`-derived type
   * @tparam ConstructorArgs constructor arguments
   * @param args behavior constructor arguments
   */
  template<class BehaviorType, typename... ConstructorArgs>
  void AddBehavior(ConstructorArgs &&... args)
  {
    behavior_mgr.template AddBehavior<BehaviorType>(this->robot_, args...);
  }

};

} // kodlab::mjbots

