//
// Created by shane on 11/17/21.
//

#pragma once
#include "kodlab_mjbots_sdk/abstract_realtime_object.h"
#include "kodlab_mjbots_sdk/mjbots_robot.h"
#include "lcm/lcm-cpp.hpp"
#include "real_time_tools/timer.hpp"
#include "real_time_tools/hard_spinner.hpp"
#include <type_traits>

/*!
 * @brief options struct for creating a mjbots behavior
 */
struct Behavior_Options{
  std::vector<Motor> motor_list_;    /// List of motors in robot
  Realtime_Params realtime_params_;  /// Set of parameters for robot's realtimeness

  float max_torque = 20;             /// Maximum torque in Nm
  int soft_start_duration = 1000;    /// Duration of the soft start in cycles
  int frequency = 1000;              /// Frequency of the control loop in Hz
  std::string channel_name;          /// LCM channel name for logging data. Leave empty to not log
};

/*!
 * @brief mjbots_behavior class is an parent class to be used to create mjbots_behavior. It supports 1 controller and
 *        logging. The child class must implement calc_torques and prepare_log (if logging). The robot data is stored in
 *        the robot object. The behavior runs in its own thread. To start the thread run start()
 * @tparam log_type[optional] data type for logging
 */
template<class log_type = void>
class Mjbots_Behavior: public Abstract_Realtime_Object{
 public:
  /*!
   * @brief constructs and mjbots behavior based on the options struct. Does not start the controller.
   * @param options contains options defining the behavior
   */
  Mjbots_Behavior(const Behavior_Options &options);

 protected:

  /*!
   * @brief runs the controller at frequency and logs the data
   */
  void run() override;

  /*!
   * @brief function to be implemented by child. Must set torques in the robot class
   */
  virtual void calc_torques() = 0;

  /*!
   * @brief adds data to m_log_data if logging is being used. To be implemented by child class
   */
  virtual void prepare_log(){};

  /*!
   * @brief adds logging information to log. Log must have entries timestamp, margin, and message_duration
   * @param t time in ms since start
   * @param margin amount of time controller slept in ms
   * @param message_duration how long it took for the message to controllers to be sent and returned in ms
   */
  void add_timing_log(float t, float margin, float message_duration);

  /*!
   * @brief publishes log to lcm
   */
  void publish_log();

  std::shared_ptr<Mjbots_Robot> m_robot;   /// ptr to the robot object, if unique causes many issues, also should be
                                           /// initialized inside thread
  int m_frequency;                         /// frequency of the controller in Hz
  int m_num_motors;                        /// Number of motors
  Behavior_Options m_options;              /// Options struct
  bool m_logging = false;                  /// Boolean to determine if logging is in use
  std::string m_channel_name;              /// Channel name to publish logs to, leave empty if not publishing
  lcm::LCM m_lcm;                          /// LCM object
  log_type m_log_data;                     /// object containing log data
};


/******************************************Implementation**************************************************************/

template<class log_type>
Mjbots_Behavior<log_type>::Mjbots_Behavior(const Behavior_Options &options) :
    Abstract_Realtime_Object(options.realtime_params_.main_rtp, options.realtime_params_.can_cpu) {
  // Extract useful values from options
  m_options = options;
  m_cpu = options.realtime_params_.can_cpu;
  m_realtime_priority = options.realtime_params_.main_rtp;
  m_frequency = options.frequency;
  m_num_motors = options.motor_list_.size();
  // Setup logging info and confirm template is provided if logging
  m_channel_name = options.channel_name;
  m_logging = !m_channel_name.empty();
  if(m_logging && std::is_same<log_type, void>()){
    std::cout<<"Warning, log_type is void, but logging is enabled"<<std::endl;
  }
}


template<class log_type>
void Mjbots_Behavior<log_type>::add_timing_log(float t, float margin, float message_duration) {
  if(m_logging){
    m_log_data.timestamp = t;
    m_log_data.margin = margin;
    m_log_data.message_duration = message_duration;
  }
}

template<class log_type>
void Mjbots_Behavior<log_type>::publish_log() {
  if(m_logging)
    m_lcm.publish(m_channel_name, &m_log_data);
}

template<class log_type>
void Mjbots_Behavior<log_type>::run() {

  // Create robot object
  m_robot = std::make_shared<Mjbots_Robot>(Mjbots_Robot(m_options.motor_list_, m_options.realtime_params_,
                                                        m_options.max_torque, m_options.soft_start_duration));

  float prev_msg_duration = 0;

  // Create spinner to time loop
  real_time_tools::HardSpinner spinner;
  spinner.set_frequency(m_frequency);

  // Create additional timers for other timing information
  real_time_tools::Timer dt_timer;
  dt_timer.tic();
  real_time_tools::Timer message_duration_timer;

  while (!CTRL_C_DETECTED) {
    // sleep the correct amount
    float sleep_duration = spinner.predict_sleeping_time();
    spinner.spin();

    // Calculate torques
    calc_torques();
    // Prepare log
    prepare_log();
    add_timing_log(dt_timer.tac() * 1000, sleep_duration * 1000, prev_msg_duration);
    message_duration_timer.tic();

    // Initiate communications cycle
    m_robot->send_command();
    // Publish log
    publish_log();
    // Wait until reply from motors is ready and then add reply to robot
    m_robot->process_reply();
    prev_msg_duration = message_duration_timer.tac() * 1000;
  }

  // Might be related to use of new
  std::cout<<"\nCTRL C Detected. Sending stop command and then segaulting" << std::endl;
  std::cout<<"TODO: Don't segfault" << std::endl;

  // Send a few stop commands
  m_robot->process_reply();
  m_robot->set_mode_stop();
  m_robot->send_command();
  m_robot->process_reply();
  m_robot->set_mode_stop();
  m_robot->send_command();
  m_robot->process_reply();

  // try to shutdown, but fail
  m_robot->shutdown();
}
