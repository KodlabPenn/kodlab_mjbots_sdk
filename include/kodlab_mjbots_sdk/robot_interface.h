// BSD 3-Clause License
// Copyright (c) 2021 The Trustees of the University of Pennsylvania. All Rights Reserved
// Authors:
// Shane Rozen-Levy <srozen01@seas.upenn.edu>


#pragma once

#include <vector>
#include <map>
#include <future>
#include <memory>
#include <optional>
#include "kodlab_mjbots_sdk/moteus_protocol.h"
#include "kodlab_mjbots_sdk/joint_moteus.h"
#include "kodlab_mjbots_sdk/soft_start.h"

namespace kodlab {


/*!
 * @brief struct for setting the realtime params for the robot
 */
struct RealtimeParams {
  int can_cpu = 3;   /// Which cpu the can should be on
  int main_cpu = 2;  /// Which cpu the main loop should be on
  int main_rtp = 97; /// The realtime priority of the main thread
  int can_rtp = 98;  /// The realtime priority of the can thread
  int lcm_rtp = 90;
  int lcm_cpu = 0;
};

/*!
 * @brief Base class for mjbot hardware interface and simulation interface
 */
class RobotInterface  {
 public:
  
  /**
   * @brief Send and recieve initial communications effectively starting the robot
   * 
   */
  virtual void Init()=0;

  /*!
   * @brief Checks to make sure the response is ready and then adds the response to the data members in the robot interface
   * should be called after send command.
   * WARNING this is a blocking function call
   */
  virtual void ProcessReply()=0;

  /*!
   * @brief initiates a cycle of communication with the pi3hat. Sends torques and requests responses.
   * WARNING this is a non blocking function call, to get the response use process reply.
   * WARNING you must call ProcessReply after calling SendCommand before sending the next command
   */
  virtual void SendCommand()=0;
  
  /*!
   * @brief sets the moteus message to be stop, Run this followed by send command to stop the motors
   */
  virtual void SetModeStop()=0;

  /*!
   * @brief Stops the robot by setting and sending stop commands
   */
  virtual void Stop()=0;

  /*!
   * @brief shuts down the can thread
   */
  virtual void Shutdown()=0;

  /*!
   * @brief Set xml model path for simulation. Do nothing by default
   */
  virtual void SetModelPath(std::string path) {} 

  /*!
   * @brief Set control frequency, which is also simulation frequency as well for now, for simulation. Do nothing by default
   */
  virtual void SetFrequency(int freq) {}
  
  /*!
   * @brief Set initial robot's state for simulation. Do nothing by default
   */
  virtual void SetInitialState(std::vector<double> initial_pos,std::vector<double> initial_vel) {}
 protected:
  
  std::vector< std::shared_ptr<kodlab::mjbots::JointMoteus>> joints; /// Vector of shared pointers to joints for the robot, shares state information
  int num_joints_ = 0;                               /// Number of joints
  u_int64_t cycle_count_ = 0;                        /// Number of cycles/commands sent
  std::map<int, int> servo_bus_map_;       /// map from servo id to servo bus
  

};
} // namespace kodlab::mjbots
