//
// Created by shane on 11/3/21.
//

#pragma once

#include <vector>
#include <map>
#include <future>
#include "kodlab_mjbots_sdk/moteus_protocol.h"
#include "kodlab_mjbots_sdk/pi3hat_moteus_interface.h"
#include "kodlab_mjbots_sdk/soft_start.h"

/*!
 * @brief Motor struct used for defining a motor in the robot
 */
struct Motor{
  /*!
   * @brief constructor to set id and can_bus
   * @param id_ the can_id of the motor, must be unique
   * @param can_bus_ the can bus the motor is on
   */
  Motor(int id_, int can_bus_): can_bus(can_bus_), id(id_){}

  /*!
   * @brief constructor to set id, can_bus, offset, and direction
   * @param id_ the can_id of the motor, must be unique
   * @param can_bus_ the can bus the motor is on
   * @param direction_ the direction of the motor, should be 1 or -1
   * @param offset_ the offset of the motor in radians
   */
  Motor(int id_, int can_bus_, int direction_, float offset_):
        can_bus(can_bus_), id(id_), direction(direction_), offset(offset_){}
  int can_bus;       /// The can bus the motor is on
  int id;            /// Motor can id, must be unique
  int direction = 1; /// direction of the motor, should be 1 or -1
  float offset  = 0; /// Offset of the motor in radians
};

/*!
 * @brief struct for setting the realtime params for the robot
 */
struct Realtime_Params{
  int can_cpu = 3;   /// Which cpu the can should be on
  int main_cpu = 2;  /// Which cpu the main loop should be on
  int main_rtp = 97; /// The realtime priority of the main thread
  int can_rtp = 98;  /// The realtime priority of the can thread
};


class Mjbots_Robot_Interface {
 public:
  /*!
   * @brief constructs an mjbots_robot_interface to communicate with a collection of moeteusses
   * @param motor_list a list of motors defining the motors in the robot
   * @param realtime_params the realtime parameters defining cpu and realtime priority
   * @param max_torque the maximum torque to allow per motor
   * @param soft_start_duration how long in dt to spend ramping the torque
   */
  Mjbots_Robot_Interface(const std::vector<Motor>& motor_list,
                         const Realtime_Params& realtime_params,
                         float max_torque = 20,
                         int soft_start_duration = 1);

  /*!
   * @brief Checks to make sure the response is ready and then adds the response to the data members in the robot interface
   * should be called after send command.
   * WARNING this is a blocking function call
   */
  void process_reply();

  /*!
   * @brief initiates a cycle of communication with the pi3hat. Sends torques and requests responses.
   * WARNING this is a non blocking function call, to get the response use process reply.
   * WARNING you must call process_reply after calling send_command before sending the next command
   */
  void send_command();

  /*!
   * @brief setter for torque command
   * @param torques[in] vector of torques
   */
  void set_torques(std::vector<float> torques);

  /*!
   * @brief sets the moteus message to be stop, run this followed by send command to stop the motors
   */
  void set_mode_stop();

  /*!
   * @brief shuts down the can thread
   */
  void shutdown();

  /*!
   * @brief accessor for joint positions, takes into account direction and offset
   * @return the joint positions
   */
  std::vector<float> get_joint_positions();

  /*!
   * @brief accessor for joint velocities, takes into account direction
   * @return the joint velocities
   */
  std::vector<float> get_joint_velocities();

  /*!
   * @brief accessor for the torque md, takes into account direction
   * @return the torque cmd
   */
  std::vector<float> get_joint_torque_cmd();

  /*!
   * @brief accessor for the measured torque by the motors, takes into account direction
   * @return the measured torque
   */
  std::vector<float> get_joint_torque_measured();

  /*!
   * @brief accessor for the joint modes
   * @return the joint modes
   */
  std::vector<mjbots::moteus::Mode> get_joint_modes();

 private:
  int m_num_servos;                         /// The number of motors in the robot
  std::vector<int> m_servo_id_list;         /// Vector of the servo id
  std::vector<int> m_servo_bus_list;        /// Vector of the servo bus
  std::map<int, int> m_servo_bus_map;       /// map from servo id to servo bus
  std::vector<float> m_positions;           /// Vector of the motor positions
  std::vector<float> m_velocities;          /// Vector of the motor velocities
  std::vector<float> m_torque_cmd;          /// Vector of the torque command sent to motors
  std::vector<float> m_torque_measured;     /// Vector of the torque measured by motors
  std::vector<float> m_offsets;             /// Offset of the motor position
  std::vector<int> m_directions;            /// Direction of motors
  bool m_timeout = false;                   /// True if communication has timed out
  std::vector<mjbots::moteus::Mode> m_modes;/// Vector of the motor modes
  long int m_cycle_count = 0;               /// How many cycles have happened, used for soft start

  std::vector<mjbots::moteus::Pi3HatMoteusInterface::ServoCommand> m_commands;  /// Vector of servo commands
  std::vector<mjbots::moteus::Pi3HatMoteusInterface::ServoReply> m_replies;     /// Vector of replies
  std::unique_ptr<mjbots::moteus::Pi3HatMoteusInterface> m_moteus_interface;    /// pi3hat interface
  mjbots::moteus::Pi3HatMoteusInterface::Data m_moteus_data;                    /// Data
  std::future<mjbots::moteus::Pi3HatMoteusInterface::Output> m_can_result;      /// future can result, used to check if
                                                                                /// response is ready
  Soft_Start m_soft_start;                                                      /// Soft start object

  /*!
   * @brief initialize the command with resolutions
   */
  void initialize_command();

  /*!
   * @brief sets command to torque mode
   */
  void prepare_torque_command();

  /*!
   * @brief gets reply the corresponds to id
   * @param replies vector of replies
   * @param id servo id
   * @return the result from servo of id id
   */
  static mjbots::moteus::QueryResult Get(const std::vector<mjbots::moteus::Pi3HatMoteusInterface::ServoReply>& replies, int id);
};

