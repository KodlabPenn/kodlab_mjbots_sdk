//
// Created by shane on 12/8/21.
//

#pragma once
#include <real_time_tools/hard_spinner.hpp>
#include <lcm/lcm-cpp.hpp>
#include <real_time_tools/timer.hpp>
#include "kodlab_mjbots_sdk/mjbots_behavior.h"

template <class logging_data_type>
class mjbots_behavior_logging : public mjbots_behavior{
 public:
  mjbots_behavior_logging(const Behavior_Options& options, const std::string channel_name);

 protected:
  virtual void prepare_log(logging_data_type& log_data) = 0;

  void run() override;

  std::string m_channel_name;

};
template<class logging_data_type>
mjbots_behavior_logging<logging_data_type>::mjbots_behavior_logging(const Behavior_Options &options,
                                                                    const std::string channel_name):
                                                                    mjbots_behavior(options),
                                                                    m_channel_name(channel_name) {

}

template<class logging_data_type>
void mjbots_behavior_logging<logging_data_type>::run() {
  lcm::LCM lcm;
  logging_data_type log_data{};


  real_time_tools::HardSpinner spinner;
  spinner.set_frequency(m_frequency);
  real_time_tools::Timer dt_timer;
  dt_timer.tic();
  real_time_tools::Timer message_duration_timer;

  while (!CTRL_C_DETECTED) {
    double sleep_duration = spinner.predict_sleeping_time();
    spinner.spin();
    calc_torques();
    prepare_log(log_data);

    message_duration_timer.tic();
    m_robot->send_command();
    log_data.timestamp = dt_timer.tac() * 1000;
    log_data.mean_margin = sleep_duration * 1000;
    lcm.publish(m_channel_name, &log_data);

    m_robot->process_reply();

    log_data.message_duration = message_duration_timer.tac() * 1000;
  }

  // Might be related to use of new
  std::cout<<"\nCTRL C Detected. Sending stop command and then segaulting" << std::endl;
  std::cout<<"TODO: Don't segfault" << std::endl;

  m_robot->process_reply();
  m_robot->set_mode_stop();
  m_robot->send_command();
  m_robot->process_reply();
  m_robot->set_mode_stop();
  m_robot->send_command();
  m_robot->process_reply();
  m_robot->shutdown();

}
