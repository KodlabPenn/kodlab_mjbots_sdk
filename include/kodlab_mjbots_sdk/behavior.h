/**
 * @file behavior.h
 * @author Ethan J. Musser (emusser@seas.upenn.edu)
 * @brief Base class for implementing robot behaviors
 * @date 7/22/22
 * 
 * @copyright Copyright 2022 The Trustees of the University of Pennsylvania. All
 *            rights reserved.
 * 
 */

#pragma once

#include <memory>
#include <string>
#include <type_traits>
#include <utility>
#include "kodlab_mjbots_sdk/robot_base.h"
#include "kodlab_mjbots_sdk/log.h"
#include "VoidLcm.hpp"
#include "kodlab_mjbots_sdk/lcm_subscriber.h"
#include "lcm/lcm-cpp.hpp"

namespace kodlab
{

/**
 * @brief Abstract Behavior class 
 * 
 * @details This class allows the use of std::vector<*AbstractBehavior<Robot>>
 *          Behavior will be a class template that gets derived from to classes 
 *          with a couple different Logging t-params. Needs every function that 
 *          should be allowed to be called
 * 
 * @tparam Robot derived `kodlab::RobotBase` class
 */
template<class Robot>
class AbstractBehavior
{
  static_assert(std::is_base_of<kodlab::RobotBase, Robot>::value,
                "Robot must have base kodlab::RobotBase.");
public:
  virtual ~AbstractBehavior() = default;
  
  virtual void Begin(const AbstractBehavior<Robot> &prev_behavior)=0;
  virtual void Update()=0;
  virtual bool Init()=0;
  virtual void Stop(const AbstractBehavior<Robot> &next_behavior)=0;
  virtual bool Running()=0;
  virtual bool ReadyToSwitch(const AbstractBehavior<Robot> &next_behavior)=0;
  virtual void PublishLog(lcm::LCM* lcm_in)=0;
  virtual void PrepareLog()=0;
  // virtual bool ReadInput() =0;

};

/**
 * @brief Robot behavior object
 * @tparam Derived derived Behavior Class
 * @tparam Robot[Optional] derived `kodlab::RobotBase` class
 * @tparam LogClass[Optional] A LCMgen Class
 */
template<class Robot = kodlab::RobotBase, 
         class LogClass = VoidLcm, 
         class InputClass = VoidLcm>
class Behavior: public AbstractBehavior<Robot>
{
protected:
  /**
   * @brief Robot that this behavior runs on
   */
  std::shared_ptr<Robot> robot_;

  /**
   * @brief Behavior name, default is "OFF"
   */
  std::string name_;

  /**
   * @brief Initialization state of this behavior
   * @note Must be updated appropriately for integration with BehaviorManager
   */
  bool initialized_ = false;

  /**
   * @brief Active state of this behavior
   */
  bool active_ = false;

  /**
   * @brief Log data for this behavior
   */
  LogClass log_data_;

  /**
   * @brief Log channel name for this behavior
   * @todo THIS MAY NOT BE USEFUL IF WE WANT TO STAY ON ONE CHANNEL
   */
  std::string log_channel_name_="";

public:
  /**
   * @brief Construct a Behavior object
   * @param robot robot behavior is executing on
   * @param name[optional] behavior name
   */
  Behavior(std::shared_ptr<Robot> robot, std::string name = "")
      : robot_(robot), name_(std::move(name)) {}

  /**
   * @brief Destructor
   */
  virtual ~Behavior() {}

  /**
   * @brief Initialize this behavior for a given robot interface
   * @note Must set `initialized_` to true for integration with BehaviorManager
   * @details This is where initialization code, which runs when a behavior is
   *          first added to a behavior manager, should be put. This is where
   *          initialization that did not take place in the constructor should
   *          occur, e.g., setting up pointers.
   * @return true if initialization successful, false otherwise
   */
  virtual bool Init() override { return true; }

  /**
   * @brief Begin this behavior for a given robot interface
   * @details This code runs a single time when the behavior starts up. This is
   *          a good place to read in the initial robot state and set the
   *          appropriate variables to begin the behavior. The previously active
   *          behavior is passed as an argument, permitting different
   *          functionalities for different prior behaviors.
   * @param prev_behavior previously active behavior
   */
  virtual void Begin(const AbstractBehavior<Robot> &prev_behavior) override {}

  /**
   * @brief Update this behavior for a given robot interface
   * @note This function must be implemented in a derived class
   * @details This code runs after Begin, and is called on every control loop
   *          update. Use this to define the primary content of your behavior,
   *          e.g., to update and command new joint torques.  Additionally, this
   *          code can monitor if the behavior has terminated and update flags
   *          appropriately.
   * @param robot robot interface
   */
  // virtual void Update() = 0;

  /**
   * @brief Stop this behavior
   * @details This code is run when a behavior is directed to stop running by
   *          the behavior manager. This should trigger a shutdown process for
   *          the behavior, e.g. slowing the robot motion. The upcoming behavior
   *          is passed as an argument to permit different shutdown procedures
   *          for different on-deck behaviors.
   * @param next_behavior behavior to be transitioned to
   */
  virtual void Stop(const AbstractBehavior<Robot> &next_behavior) override {}

  /**
   * @brief Check for whether the behavior is running
   * @details This function identifies if the behavior is active. By default, it
   *          simply accesses the `active_` variable which is toggled by the
   *          behavior manager.
   * @return true if this behavior is active, false otherwise
   */
  virtual bool Running() override { return active_; }

  /**
   * @brief Check for whether this behavior is prepared to switch to a new
   *        behavior
   * @details This function indicates to the behavior manager whether the
   *          behavior is prepared to terminate and transition to the next
   *          behavior. In its simplest form, this function accesses the
   *          internal behavior active flag. This function can be used as a mode
   *          switch guard.
   * @param next_behavior behavior being transitioned to
   * @return true if behavior is ready to switch, false otherwise
   */
  virtual bool ReadyToSwitch(const AbstractBehavior<Robot> &next_behavior) override
  {
    return true;
  }

  /**
   * @brief Publish log_data_ to lcm_in
   * @param lcm_in pointer to lcm object
   */
  virtual void PublishLog(lcm::LCM* lcm_in) override 
    {
      lcm_in->publish(log_channel_name_,&log_data_);
    };

  /**
   * @brief Virtual function for Log Preparation
   */
  virtual void PrepareLog(){};
  
  /**
   * @brief Sets this behavior's robot
   * @param robot robot that this behavior operates on
   */
  void set_robot(std::shared_ptr<Robot> robot) { robot_ = robot; }

  /**
   * @brief Sets this behavior's name
   * @param name behavior name
   */
  void set_name(const std::string &name) { name_ = name; }

  /**
   * @brief Accessor for this behavior's name
   * @return behavior name
   */
  [[nodiscard]] std::string get_name() const { return name_; }

  /**
   * @brief Accessor for initialization state
   * @return true if initialized, false otherwise
   */
  [[nodiscard]] bool is_initialized() const { return initialized_; }

  /**
   * @brief Sets behavior flag to active
   */
  void set_active() { active_ = true; }

  /**
   * @brief Sets behavior flag to inactive
   */
  void set_inactive() { active_ = false; }

  /**
   * @brief Sets initialized flag to true
   */
  void set_initialized() { initialized_ = true; }
};

} // namespace kodlab

