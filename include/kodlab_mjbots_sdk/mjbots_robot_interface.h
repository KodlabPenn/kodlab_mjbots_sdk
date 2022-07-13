// BSD 3-Clause License
// Copyright (c) 2021 The Trustees of the University of Pennsylvania. All Rights Reserved
// Authors:
// Shane Rozen-Levy <srozen01@seas.upenn.edu>


#pragma once

#include <vector>
#include <map>
#include <future>
#include <memory>
#include "kodlab_mjbots_sdk/robot_interface.h"
#include "kodlab_mjbots_sdk/moteus_protocol.h"
#include "kodlab_mjbots_sdk/joint_moteus.h"
#include "kodlab_mjbots_sdk/pi3hat_moteus_interface.h"
#include "kodlab_mjbots_sdk/soft_start.h"

namespace kodlab::mjbots {


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

class MjbotsRobotInterface : public kodlab::RobotInterface {
 public:

  /*!
   * @brief constructs an mjbots_robot_interface to communicate with a collection of moeteusses
   * @param joint_list a list of joints (or shared pointers to joints) defining the motors in the robot
   * @param realtime_params the realtime parameters defining cpu and realtime priority
   * @param soft_start_duration how long in dt to spend ramping the torque
   * @param robot_max_torque the maximum torque to allow per motor in the robot
   * @param imu_mounting_deg Orientation of the imu on the pi3hat. Assumes gravity points in the +z direction
   * @param imu_rate_hz Frequency of the imu updates from the pi3hat
   */
  MjbotsRobotInterface(const std::vector<JointMoteus> &joint_list,
                       const RealtimeParams &realtime_params,
                       int soft_start_duration = 1,
                       float robot_max_torque = 100,
                       ::mjbots::pi3hat::Euler imu_mounting_deg = ::mjbots::pi3hat::Euler(),
                       int imu_rate_hz = 1000);
  /*!
   * \overload
   */
  MjbotsRobotInterface(std::vector<std::shared_ptr<JointMoteus>> joint_list,
                       const RealtimeParams &realtime_params,
                       int soft_start_duration = 1,
                       float robot_max_torque = 100,
                       ::mjbots::pi3hat::Euler imu_mounting_deg = ::mjbots::pi3hat::Euler(),
                       int imu_rate_hz = 1000);

  /**
   * @brief Send and recieve initial communications effectively starting the robot
   * 
   */

  void Init() override;

  /*!
   * @brief Checks to make sure the response is ready and then adds the response to the data members in the robot interface
   * should be called after send command.
   * WARNING this is a blocking function call
   */
  void ProcessReply() override;

  /*!
   * @brief initiates a cycle of communication with the pi3hat. Sends torques and requests responses.
   * WARNING this is a non blocking function call, to get the response use process reply.
   * WARNING you must call ProcessReply after calling SendCommand before sending the next command
   */
  void SendCommand() override;
  
  /*!
   * @brief sets the moteus message to be stop, Run this followed by send command to stop the motors
   */
  void SetModeStop();

  /*!
   * @brief Stops the robot by setting and sending stop commands
   */
  void Stop() override;

  /*!
   * @brief shuts down the can thread
   */
  void Shutdown() override;

  /*!
   * @brief accessor for the joint modes
   * @return the joint modes
   */
  std::vector<::mjbots::moteus::Mode> GetJointModes();

  /*!
   * @brief accessor for the attitude of the robot
   * @return the attitude object for the robot
   */
  ::mjbots::pi3hat::Attitude GetAttitude();

  //TODO ATTITUDE UPDATE TO KODLAB

 private:
  std::vector< std::shared_ptr<JointMoteus>> joints_moteus_; /// Vector of shared pointers to joints for the robot, shares state information

  std::map<int, int> servo_bus_map_;       /// map from servo id to servo bus

  std::vector<std::reference_wrapper<const ::mjbots::moteus::Mode>> modes_; /// Vector of current moteus modes (references to the members of joints_)
  
  std::shared_ptr<bool> timeout_ = std::make_shared<bool>(false);                   /// True if communication has timed out

  std::vector<::mjbots::moteus::Pi3HatMoteusInterface::ServoCommand> commands_;  /// Vector of servo commands
  std::vector<::mjbots::moteus::Pi3HatMoteusInterface::ServoReply> replies_;     /// Vector of replies
  std::shared_ptr<::mjbots::moteus::Pi3HatMoteusInterface> moteus_interface_;    /// pi3hat interface
  ::mjbots::moteus::Pi3HatMoteusInterface::Data moteus_data_;                    /// Data
  std::future<::mjbots::moteus::Pi3HatMoteusInterface::Output> can_result_;      /// future can result, used to check if
                                                                                 /// response is ready
  ::mjbots::pi3hat::Attitude attitude_;                                          /// Robot attitude

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
