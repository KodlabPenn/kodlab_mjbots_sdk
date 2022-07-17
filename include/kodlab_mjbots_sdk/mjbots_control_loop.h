// BSD 3-Clause License
// Copyright (c) 2021 The Trustees of the University of Pennsylvania. All Rights Reserved
// Authors:
// Shane Rozen-Levy <srozen01@seas.upenn.edu>


#pragma once
#include <type_traits>
#include "kodlab_mjbots_sdk/abstract_realtime_object.h"
#include "kodlab_mjbots_sdk/robot_interface.h"
#include "kodlab_mjbots_sdk/mjbots_robot_interface.h"
#include "kodlab_mjbots_sdk/lcm_subscriber.h"
#include "lcm/lcm-cpp.hpp"
#include "real_time_tools/timer.hpp"
#include "real_time_tools/hard_spinner.hpp"
#include "VoidLcm.hpp"
#include "common_header.h"

namespace kodlab::mjbots {
/*!
 * @brief options struct for creating a mjbots behavior
 */
struct ControlLoopOptions {
  RealtimeParams realtime_params;  /// Set of parameters for robot's realtimeness

  float max_torque = 20;             /// Maximum torque in Nm
  int soft_start_duration = 1000;    /// Duration of the soft Start in cycles
  int frequency = 1000;              /// Frequency of the control loop in Hz
  std::string log_channel_name;         /// LCM channel name for logging data. Leave empty to not log
  std::string input_channel_name;       /// LCM channel name for input data. Leave empty to not use input
  bool parallelize_control_loop = false;   /// If true the communication with the moteus will happen in parallel with the
                                           /// torque update loop. If false the communication will happen in series. True
                                           /// results in a 1 dt delay in your controller, but is necessary for robots with
                                           /// more motors or more complicated update loops
  ::mjbots::pi3hat::Euler imu_mounting_deg; /// Orientation of the imu on the pi3hat. Assumes gravity points in the +z direction
  int attitude_rate_hz = 1000;              /// Frequency of the imu updates from the pi3hat. Options are limited to 1000
                                            /// 400, 200, 100.
};

/*!
 * @brief mjbots_control_loop class is an parent class to be used to create a control loop. It supports 1 controller and
 *        logging. The child class must implement CalcTorques and PrepareLog (if logging). The robot data is stored in
 *        the robot object. The behavior runs in its own thread. To Start the thread Run Start()
 * @tparam LogClass[optional] data type for logging
 * @tparam InputClass[optional] class for input data 
 */
template<class LogClass = VoidLcm, class InputClass = VoidLcm, class RobotClass = kodlab::RobotInterface>
class MjbotsControlLoop : public AbstractRealtimeObject {
 static_assert(std::is_base_of<kodlab::RobotInterface,RobotClass>::value);
 public:
  /*!
   * @brief constructs and mjbots behavior based on the options struct. Does not Start the controller.
   * @param joints a std::vector of JointMoteus's (or std::shared_ptr to the same)
   * @param options contains options defining the behavior
   */
  MjbotsControlLoop(std::vector<kodlab::mjbots::JointMoteus> joints, const ControlLoopOptions &options);
  /*!
   * \overload 
   */
  MjbotsControlLoop(std::vector<std::shared_ptr<kodlab::mjbots::JointMoteus>> joints, const ControlLoopOptions &options);
  /*!
   * \overload 
   */
  MjbotsControlLoop(std::shared_ptr<RobotClass>robot_in, const ControlLoopOptions &options);

 protected:

  /*!
   * @brief runs the controller at frequency and logs the data
   */
  void Run() override;

  /*!
   * @brief function to be implemented by child. Must set torques in the robot class
   */
  virtual void CalcTorques() {robot_->Update();}

  /*!
   * @brief adds data to m_log_data if logging is being used. To be implemented by child class
   */
  virtual void PrepareLog() {};

  /*!
   * @brief adds logging information to log. Log must have entries timestamp, margin, and message_duration
   * @param t time in ms since Start
   * @param margin amount of time controller slept in ms
   * @param message_duration how long it took for the message to controllers to be sent and returned in ms
   */
  void AddTimingLog(float t, float margin, float message_duration);

  /*!
   * @brief publishes log to lcm
   */
  void PublishLog();

  /*!
   * @brief handles mutex for processing input
   */
  void SafeProcessInput();

  /*!
   * @brief virtual class to be implemented when logging. Process data in m_lcm_sub.m_data; This function is threadsafe
   * and won't run if the LCM thread holds the mutex
   */
  virtual void ProcessInput() {};

  /*!
   * @brief Construct a new Setup Options object
   * 
   * @param options 
   */
  void SetupOptions(const ControlLoopOptions &options);

  std::shared_ptr<RobotClass> robot_;   /// ptr to the robot object
  std::shared_ptr<kodlab::mjbots::MjbotsRobotInterface> mjbots_interface_;   ///ptr to mjbots_interface object, if unique causes issues, also should be initialized inside thread
  int frequency_;                         /// frequency of the controller in Hz
  int num_motors_;                        /// Number of motors
  ControlLoopOptions options_;              /// Options struct
  bool logging_ = false;                  /// Boolean to determine if logging is in use
  bool input_ = false;
  std::string logging_channel_name_;              /// Channel name to publish logs to, leave empty if not publishing
  lcm::LCM lcm_;                          /// LCM object
  LogClass log_data_;                     /// object containing log data
  LcmSubscriber<InputClass> lcm_sub_;    /// LCM subscriber object
  float time_now_ = 0;                       /// Time since start in micro seconds
};

/******************************************Implementation**************************************************************/

template<class log_type, class input_type, class robot_type>
MjbotsControlLoop<log_type, input_type, robot_type>::MjbotsControlLoop(std::vector<kodlab::mjbots::JointMoteus> joints, const ControlLoopOptions &options)
  : MjbotsControlLoop<log_type, input_type,robot_type>( make_share_vector(joints), options){}

template<class log_type, class input_type, class robot_type>
MjbotsControlLoop<log_type, input_type, robot_type>::MjbotsControlLoop(std::vector<std::shared_ptr<kodlab::mjbots::JointMoteus>> joint_ptrs, const ControlLoopOptions &options)
  : AbstractRealtimeObject(options.realtime_params.main_rtp, options.realtime_params.can_cpu),
    lcm_sub_(options.realtime_params.lcm_rtp, options.realtime_params.lcm_cpu, options.input_channel_name) 
{
  robot_ = std::make_shared<robot_type>(  joint_ptrs,
                                          options.soft_start_duration,
                                          options.max_torque);
  
  mjbots_interface_ = std::make_shared<kodlab::mjbots::MjbotsRobotInterface>( joint_ptrs,
                                          options.realtime_params,
                                          options.imu_mounting_deg,
                                          options.attitude_rate_hz);
  num_motors_ = robot_->joints.size();
  SetupOptions(options);
}

template<class log_type, class input_type, class robot_type>
MjbotsControlLoop<log_type, input_type, robot_type>::MjbotsControlLoop(std::shared_ptr<robot_type>robot_in, const ControlLoopOptions &options)
  : AbstractRealtimeObject(options.realtime_params.main_rtp, options.realtime_params.can_cpu),
    lcm_sub_(options.realtime_params.lcm_rtp, options.realtime_params.lcm_cpu, options.input_channel_name) {
  // Create robot object
  robot_ = robot_in;
  mjbots_interface_ = std::make_shared<kodlab::mjbots::MjbotsRobotInterface>( robot_->joints,
                                          options.realtime_params,
                                          options.imu_mounting_deg,
                                          options.attitude_rate_hz);
  num_motors_ = robot_->joints.size();
  SetupOptions(options);
}

template<class log_type, class input_type, class robot_type>
void MjbotsControlLoop<log_type, input_type, robot_type>::SetupOptions(const ControlLoopOptions &options){
  // Extract useful values from options
  options_ = options;
  cpu_ = options.realtime_params.main_cpu;
  realtime_priority_ = options.realtime_params.main_rtp;
  frequency_ = options.frequency;
  // Setup logging info and confirm template is provided if logging
  logging_channel_name_ = options.log_channel_name;
  logging_ = !logging_channel_name_.empty();
  if (logging_ && std::is_same<log_type, VoidLcm>()) {
    std::cout << "Warning, log_type is default, but logging is enabled" << std::endl;
    logging_ = false;
  }

  input_ = !options.input_channel_name.empty();
  if (input_ && std::is_same<input_type, VoidLcm>()) {
    std::cout << "Warning, input_type is default, but input is enabled" << std::endl;
    input_ = false;
  }
}


template<class log_type, class input_type, class robot_type>
void MjbotsControlLoop<log_type, input_type, robot_type>::AddTimingLog(float t, float margin, float message_duration) {
  if (logging_) {
    log_data_.timestamp = t;
    log_data_.margin = margin;
    log_data_.message_duration = message_duration;
  }
}

template<class log_type, class input_type, class robot_type>
void MjbotsControlLoop<log_type, input_type, robot_type>::PublishLog() {
  if (logging_)
    lcm_.publish(logging_channel_name_, &log_data_);
}

template<class log_type, class input_type, class robot_type>
void MjbotsControlLoop<log_type, input_type, robot_type>::Run() {
  EnableCtrlC();

  mjbots_interface_->Init();
  robot_->Init();

  float prev_msg_duration = 0;

  // Create spinner to time loop
  real_time_tools::HardSpinner spinner;
  spinner.set_frequency(frequency_);

  // Create additional timers for other timing information
  real_time_tools::Timer dt_timer;
  real_time_tools::Timer message_duration_timer;

  spinner.initialize();
  spinner.spin();
  // If parallelizing loop send a command to process once the loop starts
  if(options_.parallelize_control_loop)
    mjbots_interface_->SendCommand();
  spinner.spin();
  dt_timer.tic();
  std::cout<<"Starting main loop"<<std::endl;
  while (!CTRL_C_DETECTED) {
    // Sleep and get current time
    float sleep_duration = spinner.predict_sleeping_time_micro();
    spinner.spin();
    time_now_ = dt_timer.tac();

    // If parallel mode, process the previous reply, then send command to keep cycle time up on pi3hat loop
    if(options_.parallelize_control_loop){
      mjbots_interface_->ProcessReply();
      prev_msg_duration = message_duration_timer.tac();
      message_duration_timer.tic();
      mjbots_interface_->SendCommand();
    }

    // Calculate torques and log
    CalcTorques();      //TODO should we give full control to the robot_ instead of feeding update thorugh?
    // robot_->Update();
    PrepareLog();
    AddTimingLog(time_now_, sleep_duration, prev_msg_duration);

    // If messages were not sent earlier, send the command and process the reply
    if(!options_.parallelize_control_loop){
      message_duration_timer.tic();
      mjbots_interface_->SendCommand();
      // Publishing log can happen now as well
      PublishLog();
      // Process input since that can be parallelized here
      SafeProcessInput();
      mjbots_interface_->ProcessReply();
      prev_msg_duration = message_duration_timer.tac();
    }
    else{
      // Publish log for parallel loop
      PublishLog();
      // Process input for parallel loop
      SafeProcessInput();
    }
  }

  // Might be related to use of new
  std::cout << "\nCTRL C Detected. Sending stop command and then segaulting" << std::endl;
  std::cout << "TODO: Don't segfault" << std::endl;

  // Send a few stop commands
  mjbots_interface_->Stop();

  // try to Shutdown, but fail
  mjbots_interface_->Shutdown();
  if (input_) {
    lcm_sub_.Join();
  }
}

template<class log_type, class input_type, class robot_type>
void MjbotsControlLoop<log_type, input_type, robot_type>::SafeProcessInput() {
  // Check to make sure using input
  if (input_) {
    // Try to unlock mutex, if you can't don't worry and try next time
    if (lcm_sub_.mutex_.try_lock()) {
      // If new message process
      if (lcm_sub_.new_message_) {
        ProcessInput();
      }
      // Set new message to false and unlock mutex
      lcm_sub_.new_message_ = false;
      lcm_sub_.mutex_.unlock();
    }
  }
}
} // namespace kodlab::mjbots
