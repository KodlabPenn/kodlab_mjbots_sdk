// BSD 3-Clause License
// Copyright (c) 2021 The Trustees of the University of Pennsylvania. All Rights Reserved
// Authors:
// Shane Rozen-Levy <srozen01@seas.upenn.edu>


#pragma once
#include "kodlab_mjbots_sdk/abstract_realtime_object.h"
#include "kodlab_mjbots_sdk/mjbots_robot_interface.h"
#include "kodlab_mjbots_sdk/abstract_lcm_subscriber.h"
#include "lcm/lcm-cpp.hpp"
#include "real_time_tools/timer.hpp"
#include "real_time_tools/hard_spinner.hpp"
#include <type_traits>
#include "void_lcm.hpp"
/*!
 * @brief options struct for creating a mjbots behavior
 */
struct Control_Loop_Options{
  std::vector<Motor> m_motor_list;    /// List of motors in robot
  Realtime_Params m_realtime_params;  /// Set of parameters for robot's realtimeness

  float m_max_torque = 20;             /// Maximum torque in Nm
  int m_soft_start_duration = 1000;    /// Duration of the soft start in cycles
  int m_frequency = 1000;              /// Frequency of the control loop in Hz
  std::string m_log_channel_name;          /// LCM channel name for logging data. Leave empty to not log
  std::string m_input_channel_name;          /// LCM channel name for input data. Leave empty to not use input
};

/*!
 * @brief mjbots_control_loop class is an parent class to be used to create a control loop. It supports 1 controller and
 *        logging. The child class must implement calc_torques and prepare_log (if logging). The robot data is stored in
 *        the robot object. The behavior runs in its own thread. To start the thread run start()
 * @tparam log_type[optional] data type for logging
 */
template<class log_type = void_lcm, class input_type = void_lcm>
class Mjbots_Control_Loop: public Abstract_Realtime_Object{
 public:
  /*!
   * @brief constructs and mjbots behavior based on the options struct. Does not start the controller.
   * @param options contains options defining the behavior
   */
  Mjbots_Control_Loop(const Control_Loop_Options &options);

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

  void safe_process_input();

  virtual void process_input(){};

  std::shared_ptr<Mjbots_Robot_Interface> m_robot;   /// ptr to the robot object, if unique causes many issues, also should be
                                                     /// initialized inside thread
  int m_frequency;                         /// frequency of the controller in Hz
  int m_num_motors;                        /// Number of motors
  Control_Loop_Options m_options;              /// Options struct
  bool m_logging = false;                  /// Boolean to determine if logging is in use
  bool m_input = false;
  std::string m_logging_channel_name;              /// Channel name to publish logs to, leave empty if not publishing
  lcm::LCM m_lcm;                          /// LCM object
  log_type m_log_data;                     /// object containing log data
  Lcm_Subscriber<input_type> m_lcm_sub;
};


/******************************************Implementation**************************************************************/

template<class log_type, class input_type>
Mjbots_Control_Loop<log_type, input_type>::Mjbots_Control_Loop(const Control_Loop_Options &options) :
    Abstract_Realtime_Object(options.m_realtime_params.main_rtp, options.m_realtime_params.can_cpu),
    m_lcm_sub(options.m_realtime_params.lcm_rtp, options.m_realtime_params.lcm_cpu, options.m_input_channel_name){
  // Extract useful values from options
  m_options = options;
  m_cpu = options.m_realtime_params.main_cpu;
  m_realtime_priority = options.m_realtime_params.main_rtp;
  m_frequency = options.m_frequency;
  m_num_motors = options.m_motor_list.size();
  // Setup logging info and confirm template is provided if logging
  m_logging_channel_name = options.m_log_channel_name;
  m_logging = !m_logging_channel_name.empty();
  if(m_logging && std::is_same<log_type, void_lcm>()){
    std::cout<<"Warning, log_type is default, but logging is enabled"<<std::endl;
    m_logging = false;
  }

  m_input = !options.m_input_channel_name.empty();
  if(m_input && std::is_same<input_type, void_lcm>()){
    std::cout<<"Warning, input_type is default, but input is enabled"<<std::endl;
    m_input = false;
  }
}


template<class log_type, class input_type>
void Mjbots_Control_Loop<log_type, input_type>::add_timing_log(float t, float margin, float message_duration) {
  if(m_logging){
    m_log_data.timestamp = t;
    m_log_data.margin = margin;
    m_log_data.message_duration = message_duration;
  }
}

template<class log_type, class input_type>
void Mjbots_Control_Loop<log_type, input_type>::publish_log() {
  if(m_logging)
    m_lcm.publish(m_logging_channel_name, &m_log_data);
}

template<class log_type, class input_type>
void Mjbots_Control_Loop<log_type, input_type>::run() {
  enable_ctrl_c();

  // Create robot object
  m_robot = std::make_shared<Mjbots_Robot_Interface>(Mjbots_Robot_Interface(m_options.m_motor_list, m_options.m_realtime_params,
                                                                            m_options.m_max_torque, m_options.m_soft_start_duration));

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
    message_duration_timer.tic();

    // Initiate communications cycle
    m_robot->send_command();
    // Publish log
    add_timing_log(dt_timer.tac() * 1000, sleep_duration * 1000, prev_msg_duration);
    publish_log();
    // handle new inputs if available
    safe_process_input();
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
  m_robot->send_command();
  m_robot->process_reply();

  // try to shutdown, but fail
  m_robot->shutdown();
  if(m_input){
    m_lcm_sub.join();
  }
}

template<class log_type, class input_type>
void Mjbots_Control_Loop<log_type, input_type>::safe_process_input() {
  if(m_input){
    if (m_lcm_sub.m_mutex.try_lock()){
      if(m_lcm_sub.m_new_message){
        process_input();
      }
      m_lcm_sub.m_new_message = false;
      m_lcm_sub.m_mutex.unlock();
    }
  }
}
