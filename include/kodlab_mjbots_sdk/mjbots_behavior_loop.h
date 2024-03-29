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

#include <type_traits>

#include "kodlab_mjbots_sdk/mjbots_control_loop.h"
#include "kodlab_mjbots_sdk/behavior_manager.h"
#include "kodlab_mjbots_sdk/behavior.h"
#include "kodlab_mjbots_sdk/off_behavior.h"
#include "kodlab_mjbots_sdk/io_behavior.h"
#include "kodlab_mjbots_sdk/log.h"
#include "kodlab_mjbots_sdk/type_traits.h"

namespace kodlab::mjbots {

/**
 * @brief MjbotsControlLoop with behavior suppport.
 * @tparam Log[optional] data type for logging
 * @tparam Input[optional] class for input data
 * @tparam Robot[optional] `RobotBase`-Derived class that contains state and
 *                         control calculations
 */
template<class Log = VoidLcm, class Input = VoidLcm,
    class Robot = kodlab::RobotBase>
class MjbotsBehaviorLoop : public MjbotsControlLoop<Log, Input, Robot> {

 protected:
  /**
   * @brief Behavior manager handling robot behaviors and transitions between
   *        robot behaviors
   */
  BehaviorManager<Robot> behavior_manager_;

  /**
   * @brief Updates robot state and sets torques based on active behavior in
   *        behavior manager
   * @note If a derived class chooses to override `Update`, ensure that the
   *       necessary internal updates are called. This is not recommended.
   */
  void Update() override {
    this->robot_->Update();
    behavior_manager_.Update();
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
        behavior_manager_{this->robot_} {}

  /**
   * @brief Construct an Mjbots behavior loop based on an options struct
   * @note Does not Start the controller.
   * @param joints vector of JointMoteus shared pointers
   * @param options options defining the behavior
   */
  MjbotsBehaviorLoop(std::vector<std::shared_ptr<kodlab::mjbots::JointMoteus>> joints,
                     const ControlLoopOptions &options)
      : MjbotsControlLoop<Log, Input, Robot>(joints, options),
        behavior_manager_{this->robot_} {}

  /**
   * @brief Construct an Mjbots behavior loop based on an options struct
   * @note Does not Start the controller.
   * @param robot_in instance of a derived RobotBase
   * @param options options defining the behavior
   */
  MjbotsBehaviorLoop(std::shared_ptr<Robot> robot_in,
                     const ControlLoopOptions &options)
      : MjbotsControlLoop<Log, Input, Robot>(robot_in, options),
        behavior_manager_{this->robot_} {}

  /**
   * @brief Destructor
   */
  virtual ~MjbotsBehaviorLoop() = default;

  /**
   * @brief Add a `Behavior` child to the behavior manager
   * @note Parameters `args` should omit the first parameter,
   * `robot`.
   * @tparam BehaviorType `kodlab::Behavior`-derived type
   * @tparam ConstructorArgs constructor arguments
   * @param constructor_args behavior constructor arguments subsequent to
   * `robot`
   */
  template<class BehaviorType, typename... ConstructorArgs>
  void AddBehavior(ConstructorArgs &&... constructor_args) {
    static_assert(kodlab::type_traits::is_base_of_template<Behavior,
                                                           BehaviorType>::value);
    behavior_manager_.template AddBehavior<BehaviorType>(this->robot_,
                                                         constructor_args...);
  }

  /**
   * @brief Add an `IOBehavior` child to the behavior manager
   * @note Parameters `ConstructorArgs` should omit the first three parameters:
   * `robot`, `subscriber`, and `publish_lcm`.
   * @tparam BehaviorType `kodlab::IOBehavior`-derived type
   * @tparam ConstructorArgs constructor arguments
   * @param constructor_args behavior constructor arguments subsequent to `lcm`
   */
  template<class BehaviorType, typename... ConstructorArgs>
  void AddIOBehavior(ConstructorArgs &&... constructor_args) {
    static_assert(kodlab::type_traits::is_base_of_template<IOBehavior,
                                                           BehaviorType>::value);
    behavior_manager_.template AddBehavior<BehaviorType>(this->robot_,
                                                         this->lcm_sub_,
                                                         this->lcm_,
                                                         constructor_args...);
  }

  /**
   * @brief Add a `Behavior` child as the default behavior in the behavior
   * manager
   * @tparam BehaviorType `kodlab::Behavior`-derived type
   * @tparam ConstructorArgs constructor arguments
   * @param constructor_args behavior constructor arguments subsequent to
   * `robot`
   */
  template<class BehaviorType, typename... ConstructorArgs>
  void SetDefaultBehavior(ConstructorArgs &&... constructor_args) {
    static_assert(kodlab::type_traits::is_base_of_template<Behavior,
                                                           BehaviorType>::value);
    behavior_manager_.template SetDefaultBehavior<BehaviorType>(this->robot_,
                                                                constructor_args...);
  }

  /**
   * @brief Add an `IOBehavior` child as the default behavior in the behavior
   * manager
   * @note Parameters `ConstructorArgs` should omit the first three parameters:
   * `robot`, `subscriber`, and `publish_lcm`.
   * @tparam BehaviorType `kodlab::IOBehavior`-derived type
   * @tparam ConstructorArgs constructor arguments
   * @param constructor_args behavior constructor arguments subsequent to `lcm`
   */
  template<class BehaviorType, typename... ConstructorArgs>
  void SetDefaultIOBehavior(ConstructorArgs &&... constructor_args) {
    static_assert(kodlab::type_traits::is_base_of_template<IOBehavior,
                                                           BehaviorType>::value);
    behavior_manager_.template SetDefaultBehavior<BehaviorType>(this->robot_,
                                                                this->lcm_sub_,
                                                                this->lcm_,
                                                                constructor_args...);
  }

  /**
   * @brief Set the current behavior in the behavior manager
   * @param next_idx index in the behavior manager behaviors list
   */
  void SetBehavior(int next_idx) {
    behavior_manager_.SetBehavior(next_idx);
  }

  /**
   * @brief Set the current behavior in the behavior manager
   * @param name name of behavior in behavior manager
   */
  void SetBehavior(const std::string &name) {
    behavior_manager_.SetBehavior(name);
  }


  /**
   * @brief Prints a list of behaviors currently stored in the behavior manager
   * @param stream output stream
   */
  void PrintBehaviorList(FILE *stream = stdout) {
    behavior_manager_.PrintBehaviorList();
  }

};

} // kodlab::mjbots

