// BSD 3-Clause License
// Copyright (c) 2021 The Trustees of the University of Pennsylvania. All Rights Reserved
// Authors:
// Shane Rozen-Levy <srozen01@seas.upenn.edu>


#pragma once

#include <vector>
#include <map>
#include <future>
#include "kodlab_mjbots_sdk/moteus_protocol.h"
#include "kodlab_mjbots_sdk/pi3hat_moteus_interface.h"
#include "kodlab_mjbots_sdk/soft_start.h"

namespace kodlab::mjbots {
/*!
 * @brief Motor struct used for defining a motor in the robot
 */
struct Motor {
  /*!
   * @brief constructor to set id and can_bus
   * @param id_ the can_id of the motor, must be unique
   * @param can_bus_ the can bus the motor is on
   */
  Motor(int id_, int can_bus_) : can_bus(can_bus_), id(id_) {}

  /*!
   * @brief constructor to set id, can_bus, offset, and direction
   * @param id_ the can_id of the motor, must be unique
   * @param can_bus_ the can bus the motor is on
   * @param direction_ the direction of the motor, should be 1 or -1
   * @param offset_ the offset of the motor in radians
   */
  Motor(int id_, int can_bus_, int direction_, float offset_) :
      can_bus(can_bus_), id(id_), direction(direction_), offset(offset_) {}
  int can_bus;       /// The can bus the motor is on
  int id;            /// Motor can id, must be unique
  int direction = 1; /// direction of the motor, should be 1 or -1
  float offset = 0; /// Offset of the motor in radians
};

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

class MjbotsRobotInterface {
 public:
  /*!
   * @brief constructs an mjbots_robot_interface to communicate with a collection of moeteusses
   * @param motor_list a list of motors defining the motors in the robot
   * @param realtime_params the realtime parameters defining cpu and realtime priority
   * @param max_torque the maximum torque to allow per motor
   * @param soft_start_duration how long in dt to spend ramping the torque
   */
  MjbotsRobotInterface(const std::vector<Motor> &motor_list,
                       const RealtimeParams &realtime_params,
                       float max_torque = 20,
                       int soft_start_duration = 1);

  /*!
   * @brief Checks to make sure the response is ready and then adds the response to the data members in the robot interface
   * should be called after send command.
   * WARNING this is a blocking function call
   */
  void ProcessReply();

  /*!
   * @brief initiates a cycle of communication with the pi3hat. Sends torques and requests responses.
   * WARNING this is a non blocking function call, to get the response use process reply.
   * WARNING you must call ProcessReply after calling SendCommand before sending the next command
   */
  void SendCommand();

  /*!
   * @brief setter for torque command
   * @param torques[in] vector of torques
   */
  void SetTorques(std::vector<float> torques);

  /*!
   * @brief sets the moteus message to be stop, Run this followed by send command to stop the motors
   */
  void SetModeStop();

  /*!
   * @brief shuts down the can thread
   */
  void Shutdown();

  /*!
   * @brief accessor for joint positions, takes into account direction and offset
   * @return the joint positions
   */
  std::vector<float> GetJointPositions();

  /*!
   * @brief accessor for joint velocities, takes into account direction
   * @return the joint velocities
   */
  std::vector<float> GetJointVelocities();

  /*!
   * @brief accessor for the torque md, takes into account direction
   * @return the torque cmd
   */
  std::vector<float> GetJointTorqueCmd();

  /*!
   * @brief accessor for the joint modes
   * @return the joint modes
   */
  std::vector<::mjbots::moteus::Mode> GetJointModes();

 private:
  int num_servos_;                         /// The number of motors in the robot
  std::vector<int> servo_id_list_;         /// Vector of the servo id
  std::vector<int> servo_bus_list_;        /// Vector of the servo bus
  std::map<int, int> servo_bus_map_;       /// map from servo id to servo bus
  std::vector<float> positions_;           /// Vector of the motor positions
  std::vector<float> velocities_;          /// Vector of the motor velocities
  std::vector<float> torque_cmd_;          /// Vector of the torque command sent to motors
  std::vector<float> offsets_;             /// Offset of the motor position
  std::vector<int> directions_;            /// Direction of motors
  std::shared_ptr<bool> timeout_ = std::make_shared<bool>(false);                   /// True if communication has timed out
  std::vector<::mjbots::moteus::Mode> modes_;/// Vector of the motor modes
  u_int64_t cycle_count_ = 0;               /// How many cycles have happened, used for soft Start

  std::vector<::mjbots::moteus::Pi3HatMoteusInterface::ServoCommand> commands_;  /// Vector of servo commands
  std::vector<::mjbots::moteus::Pi3HatMoteusInterface::ServoReply> replies_;     /// Vector of replies
  std::shared_ptr<::mjbots::moteus::Pi3HatMoteusInterface> moteus_interface_;    /// pi3hat interface
  ::mjbots::moteus::Pi3HatMoteusInterface::Data moteus_data_;                    /// Data
  std::future<::mjbots::moteus::Pi3HatMoteusInterface::Output> can_result_;      /// future can result, used to check if
  /// response is ready
  SoftStart soft_start_;                                                      /// Soft Start object

  /*!
   * @brief initialize the command with resolutions
   */
  void InitializeCommand();

  /*!
   * @brief sets command to torque mode
   */
  void PrepareTorqueCommand();

  /*!
   * @brief gets reply the corresponds to id
   * @param replies vector of replies
   * @param id servo id
   * @return the result from servo of id id
   */
  static ::mjbots::moteus::QueryResult Get(const std::vector<::mjbots::moteus::Pi3HatMoteusInterface::ServoReply> &replies,
                                         int id);
};
} // namespace kodlab::mjbots
