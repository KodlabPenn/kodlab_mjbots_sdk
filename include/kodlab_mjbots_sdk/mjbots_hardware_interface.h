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
#include "kodlab_mjbots_sdk/pi3hat_moteus_interface.h"
#include "kodlab_mjbots_sdk/soft_start.h"
#include "kodlab_mjbots_sdk/imu_data.h"

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

/*!
 * @brief Object allowing interaction with the Mjbots Moteus motor controller
 *        hardware
 */
class MjbotsHardwareInterface  {
 public:

  /*!
   * @brief constructs an mjbots_robot_interface to communicate with a collection of moeteusses
   * @param joint_list a vector of shared pointers to joints defining the motors in the robot
   * @param realtime_params the realtime parameters defining cpu and realtime priority
   * @param imu_mounting_deg Orientation of the imu on the pi3hat. Assumes gravity points in the +z direction
   * @param imu_rate_hz Frequency of the imu updates from the pi3hat
   * @param imu_data_ptr Shared pointer to imu_data to use or nullptr if it should make its own
   * @param imu_world_offset_deg [Optional] IMU orientation offset. Useful for re-orienting gravity, etc.
   * @param dry_run if true, sends zero-torques to Moteus controllers
   * @param print_torques if true, prints torque commands
   * @param send_pd_commands if true, packets to the moteus include pd gains and setpoints
   * @param send_current if true, packets to the moteus include q_current and d_current registers
   * @param send_voltage if true, packets to the moteus include voltage register
   * @param send_temperature if true, packets to the moteus include board temperature register
   * @param send_fault if true, packets to the moteus include fault code register
   * 
   */
  MjbotsHardwareInterface(std::vector<std::shared_ptr<JointMoteus>> joint_list,
                       const RealtimeParams &realtime_params,
                       ::mjbots::pi3hat::Euler imu_mounting_deg = ::mjbots::pi3hat::Euler(),
                       int imu_rate_hz = 1000,
                       std::shared_ptr<::kodlab::IMUData<float>> imu_data_ptr = nullptr,
                       std::optional<::mjbots::pi3hat::Euler > imu_world_offset_deg = std::nullopt,
                       bool dry_run = false,
                       bool print_torques = false,
                       bool send_pd_commands = false,
                       bool send_current = false,
                       bool send_voltage = false,
                       bool send_temperature = false,
                       bool send_fault = false
                       );

  /**
   * @brief Send and recieve initial communications effectively starting the robot
   * 
   */
  void Init();

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
   * @brief sets the moteus message to be stop, Run this followed by send command to stop the motors
   */
  void SetModeStop();

  /*!
   * @brief Stops the robot by setting and sending stop commands
   */
  void Stop();

  /*!
   * @brief shuts down the can thread
   */
  void Shutdown();

  /*!
    * @brief Get a vector of shared_ptrs to joints 
    * @return a vector of shared pointers to the desired joints
    */
  std::vector<std::shared_ptr<JointMoteus>> GetJoints();

  /*!
   * @brief accessor for the joint modes
   * @return the joint modes
   */
  std::vector<::mjbots::moteus::Mode> GetJointModes();
  
  /*!
   * @brief accessor for the IMU data of the robot
   * @return const reference to the IMU data object for the robot
   */
  const ::kodlab::IMUData<float>& GetIMUData();

  /*!
   * @brief accessor for the IMU data of the robot
   * @return const IMU data shared pointer for the robot
   */
  const std::shared_ptr<::kodlab::IMUData<float>> GetIMUDataSharedPtr();        
  
  /*!
  * @brief Setter for the robot's IMU data pointer. Releases the previously owned IMU data object
  *
  * @param imu_data_ptr a shared pointer to kodlab::IMUData
  */
  void SetIMUDataSharedPtr(std::shared_ptr<::kodlab::IMUData<float>> imu_data_ptr){imu_data_ = imu_data_ptr;}

 private:
  std::vector< std::shared_ptr<JointMoteus>> joints; /// Vector of shared pointers to joints for the robot, shares state information
  int num_joints_ = 0;                               /// Number of joints
  u_int64_t cycle_count_ = 0;                        /// Number of cycles/commands sent
  bool dry_run_;                                     ///< dry run active flag
  bool print_torques_;                               ///< print torques active flag
  bool send_pd_commands_;                            ///< Include pd gains and setpoints in the moteus packet
  bool send_current_;                                ///< Include q_current and d_current registers in the moteus packet
  bool send_voltage_;                                ///< Include voltage register in the moteus packet
  bool send_temperature_;                            ///< Include board temperature register in the moteus packet
  bool send_fault_;                                  ///< Include fault code register in the moteus packet

  std::map<int, int> servo_bus_map_;       /// map from servo id to servo bus

  std::vector<std::reference_wrapper<const ::mjbots::moteus::Mode>> modes_; /// Vector of current moteus modes (references to the members of joints_)

  std::shared_ptr<bool> timeout_ = std::make_shared<bool>(false);                /// True if communication has timed out

  std::vector<::mjbots::moteus::Pi3HatMoteusInterface::ServoCommand> commands_;  /// Vector of servo commands
  std::vector<::mjbots::moteus::Pi3HatMoteusInterface::ServoReply> replies_;     /// Vector of replies
  std::shared_ptr<::mjbots::moteus::Pi3HatMoteusInterface> moteus_interface_;    /// pi3hat interface
  ::mjbots::moteus::Pi3HatMoteusInterface::Data moteus_data_;                    /// Data
  std::future<::mjbots::moteus::Pi3HatMoteusInterface::Output> can_result_;      /// future can result, used to check if response is ready
  std::shared_ptr<::kodlab::IMUData<float>> imu_data_;                           /// Robot IMU data

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
