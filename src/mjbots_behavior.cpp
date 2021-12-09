//
// Created by shane on 12/8/21.
//
#include "kodlab_mjbots_sdk/mjbots_behavior.h"
#include "real_time_tools/hard_spinner.hpp"
#include "real_time_tools/timer.hpp"

mjbots_behavior::mjbots_behavior(const Behavior_Options& options) {
  m_options = options;
  m_cpu = options.realtime_params_.can_cpu;
  m_realtime_priority = options.realtime_params_.main_rtp;
  m_frequency = options.frequency;
  m_num_motors = options.motor_list_.size();
  m_robot = std::make_unique<Realtime_Robot>(Realtime_Robot(options.motor_list_, options.realtime_params_,
                                                          options.max_torque, options.soft_start_duration));

  start();
}



void mjbots_behavior::run() {
  real_time_tools::HardSpinner spinner;
  spinner.set_frequency(m_frequency);

  while (!CTRL_C_DETECTED) {
    spinner.spin();
    calc_torques();
    m_robot->send_command();
    m_robot->process_reply();
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
