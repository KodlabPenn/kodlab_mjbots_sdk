// BSD 3-Clause License
// Copyright (c) 2021 The Trustees of the University of Pennsylvania. All Rights Reserved
// Authors:
// Shane Rozen-Levy <srozen01@seas.upenn.edu>
// J. Diego Caporale <jdcap@seas.upenn.edu>


#pragma once
#include <type_traits>
#include "kodlab_mjbots_sdk/joint_moteus.h"
#include "kodlab_mjbots_sdk/abstract_realtime_object.h"
#include "kodlab_mjbots_sdk/robot_base.h"
#include "kodlab_mjbots_sdk/mjbots_hardware_interface.h"
#include "kodlab_mjbots_sdk/lcm_subscriber.h"
#include "kodlab_mjbots_sdk/lcm_message_handler.h"
#include "kodlab_mjbots_sdk/lcm_publisher.h"
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
  float soft_start_duration_ms = 1000;    /// Duration of the soft Start in ms
  int frequency = 1000;              /// Frequency of the control loop in Hz
  std::string log_channel_name;         /// LCM channel name for logging data. Leave empty to not log
  std::string input_channel_name;       /// LCM channel name for input data. Leave empty to not use input
  bool parallelize_control_loop = false;   /// If true the communication with the moteus will happen in parallel with the
                                           /// torque update loop. If false the communication will happen in series. True
                                           /// results in a 1 dt delay in your controller, but is necessary for robots with
                                           /// more motors or more complicated update loops
  ::mjbots::pi3hat::Euler imu_mounting_deg; /// Orientation of the imu on the pi3hat. Assumes gravity points in the +z direction
  ::mjbots::pi3hat::Euler imu_world_offset_deg; /// IMU orientation offset. Useful for re-orienting gravity, etc.
  int attitude_rate_hz = 1000;              /// Frequency of the imu updates from the pi3hat. Options are limited to 1000
                                            /// 400, 200, 100.
  bool dry_run = false;  ///< If true, torques will be printed to console and not sent to the joints
};

/*!
 * @brief A parent class used to create a control loop. It supports one
 *        controller, a `RobotBase` child class, logging, and inputs. The
 *        `MjbotsControlLoop` child class must implement `Update`,
 *        `PrepareLog` if logging, and `ProcessInput` if receiving inputs. The
 *        robot data is stored in the `RobotClass` object. The behavior runs in
 *        its own thread. To Start the thread, run `Start()`.
 * @tparam LogClass[optional] data type for logging
 * @tparam InputClass[optional] class for input data 
 * @tparam RobotClass[optional] RobotInterfaceDerived class that contains state and control calculations 
 */
template<class LogClass = VoidLcm, class InputClass = VoidLcm, class RobotClass = kodlab::RobotBase>
class MjbotsControlLoop : public AbstractRealtimeObject {
 static_assert(std::is_base_of<kodlab::RobotBase, RobotClass>::value);
 public:
  /*!
   * @brief constructs an mjbots control loop based on the options struct. Does not Start the controller.
   * @param joints a std::vector of JointMoteus's
   * @param options contains options defining the behavior
   */
  MjbotsControlLoop(std::vector<kodlab::mjbots::JointMoteus> joints, const ControlLoopOptions &options);
  /*!
   * @brief constructs an mjbots control loop based on the options struct. Does not Start the controller.
   * @param joints a std::vector of std::shared_ptrs of JointMoteus
   * @param options contains options defining the behavior
   * \overload 
   */
  MjbotsControlLoop(std::vector<std::shared_ptr<kodlab::mjbots::JointMoteus>> joints, const ControlLoopOptions &options);
  /*!
   * @brief constructs an mjbots control loop based on the options struct. Does not Start the controller.
   * @param robot_in an instance of a derived RobotBase
   * @param options contains options defining the behavior
   * \overload 
   */
  MjbotsControlLoop(std::shared_ptr<RobotClass>robot_in, const ControlLoopOptions &options);

 protected:

  /*!
   * @brief Initialization function run when the control loop is started
   * @details This function is called at the beginning of the `Run` function,
   *          and can be used for initialization in child implementations.
   * @note This function is called in the `Run` function directly after `robot_`
   *       and `mjbots_interface_` are initialized.  Any initialization desired
   *       before this point should be written into the derived class
   *       constructors.
   * @note This function is included to provide a user-overridable
   *       initialization function.  It is not called in the constructor due to
   *       issues with virtual methods in base class constructors.
   */
  virtual void Init() {}

  /*!
   * @brief runs the controller at frequency and logs the data
   */
  void Run() override;

  /*!
   * @brief function to be implemented by child. Must set torques in the robot class
   */
  virtual void Update() = 0;

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
   * @brief virtual method to be implemented when logging. Process data in
   * `input_sub_.data`; This function is threadsafe and won't run if the LCM
   * thread holds the mutex
   * @param input_data input data to be processed
   */
  virtual void ProcessInput(const InputClass &input_data) {};

  /*!
   * @brief Construct a new Setup Options object
   * 
   * @param options 
   */
  void SetupOptions(const ControlLoopOptions &options);

  std::shared_ptr<RobotClass> robot_;     /// ptr to the robot object
  std::shared_ptr<kodlab::mjbots::MjbotsHardwareInterface> mjbots_interface_;   ///ptr to mjbots_interface object, if unique causes issues, also should be initialized inside thread
  int frequency_;                         /// frequency of the controller in Hz
  int num_joints_;                        /// Number of motors
  ControlLoopOptions options_;            /// Options struct
  bool logging_ = false;                  /// Boolean to determine if logging is in use
  bool input_ = false;                    /// Boolean to determine if input is in use
  std::shared_ptr<lcm::LCM> lcm_;         /// LCM object shared pointer
  LcmPublisher<LogClass> log_pub_;        /// log LCM publisher
  std::shared_ptr<LogClass> log_data_;    /// LCM log message data shared pointer
  std::shared_ptr<LcmSubscriber> lcm_sub_;     /// LCM subscriber object
  LcmMessageHandler<InputClass> input_sub_;    /// LCM input subscription
  float time_now_ = 0;                    /// Time since start in micro seconds
};

/******************************************Implementation**************************************************************/

template<class log_type, class input_type, class robot_type>
MjbotsControlLoop<log_type, input_type, robot_type>::MjbotsControlLoop(std::vector<kodlab::mjbots::JointMoteus> joints, const ControlLoopOptions &options)
  : MjbotsControlLoop<log_type, input_type,robot_type>( make_share_vector(joints), options){}

template<class log_type, class input_type, class robot_type>
MjbotsControlLoop<log_type, input_type, robot_type>::MjbotsControlLoop(
    std::vector<std::shared_ptr<kodlab::mjbots::JointMoteus>> joint_ptrs, const ControlLoopOptions &options)
  : MjbotsControlLoop(std::make_shared<robot_type>(joint_ptrs, options.max_torque, options.soft_start_duration_ms),options) {}

template<class log_type, class input_type, class robot_type>
MjbotsControlLoop<log_type, input_type, robot_type>::MjbotsControlLoop(std::shared_ptr<robot_type>robot_in, const ControlLoopOptions &options)
  : AbstractRealtimeObject(options.realtime_params.main_rtp, options.realtime_params.can_cpu),
    lcm_(std::make_shared<lcm::LCM>()),
    log_pub_(lcm_, options.log_channel_name),
    log_data_(log_pub_.get_message_shared_ptr()),
    lcm_sub_(std::make_shared<LcmSubscriber>(options.realtime_params.lcm_rtp, options.realtime_params.lcm_cpu)) {
  // Copy robot pointer
  robot_ = robot_in;
  
  // Cast joints to JointMoteus (currently JointBase)
  std::vector<std::shared_ptr<kodlab::mjbots::JointMoteus>> joints_moteus;
  for (auto &j : robot_->joints) {
    joints_moteus.push_back(
        std::dynamic_pointer_cast<kodlab::mjbots::JointMoteus>(j));
  }
  // Initialize mjbots_interface
  mjbots_interface_ = std::make_shared<kodlab::mjbots::MjbotsHardwareInterface>(
      std::move(joints_moteus), options.realtime_params,
      options.imu_mounting_deg, options.attitude_rate_hz,
      robot_->GetIMUDataSharedPtr(), options.imu_world_offset_deg,
      options.dry_run);
  num_joints_ = robot_->joints.size();
  SetupOptions(options);
}

template<class log_type, class input_type, class robot_type>
void MjbotsControlLoop<log_type, input_type, robot_type>::SetupOptions(const ControlLoopOptions &options){
  // Extract useful values from options
  options_ = options;
  cpu_ = options.realtime_params.main_cpu;
  realtime_priority_ = options.realtime_params.main_rtp;
  frequency_ = options.frequency;
  // Setup logging info and confirm template is provided if logging is named
  logging_ = !log_pub_.get_channel().empty();
  if (logging_ && std::is_same<log_type, VoidLcm>()) {
    std::cout << "Warning, log_type is default, but logging is enabled" << std::endl;
    logging_ = false;
  }
  //Add input subscriber if not VoidLcm
  input_ = !options.input_channel_name.empty();
  if (input_) {
    if (std::is_same<input_type, VoidLcm>()) {
      std::cout << "Warning, input_type is default, but input is enabled"
                << std::endl;
      input_ = false;
    } else {
      // Add control loop input subscriber
      lcm_sub_->AddSubscription<input_type>(options.input_channel_name,
                                            input_sub_);
    }
  }
}


template<class log_type, class input_type, class robot_type>
void MjbotsControlLoop<log_type, input_type, robot_type>::AddTimingLog(float t, float margin, float message_duration) {
  if (logging_) {
    log_data_->timestamp = t;
    log_data_->margin = margin;
    log_data_->message_duration = message_duration;
  }
}

template<class log_type, class input_type, class robot_type>
void MjbotsControlLoop<log_type, input_type, robot_type>::PublishLog() {
  if (logging_)
    log_pub_.Publish();
}

template<class log_type, class input_type, class robot_type>
void MjbotsControlLoop<log_type, input_type, robot_type>::Run() {
  EnableCtrlC();

  mjbots_interface_->Init();
  robot_->Init();
  lcm_sub_->Init();
  Init();

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
    Update();      //TODO should we give full control to the robot_ instead of feeding update thorugh?
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
    lcm_sub_->Join();
  }
}

template<class log_type, class input_type, class robot_type>
void MjbotsControlLoop<log_type, input_type, robot_type>::SafeProcessInput() {
  // Check to make sure using input
  if (input_) {
    // Retrieve new data if available, std::nullopt_t otherwise
    auto data_in = input_sub_.GetDataIfNew();
    // If new data is present, pass as input
    if (data_in) {
      // Pass new data as input
      ProcessInput(data_in.value());
    }
  }
}
} // namespace kodlab::mjbots
