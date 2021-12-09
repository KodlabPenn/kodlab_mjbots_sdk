//
// Created by shane on 12/8/21.
//
#include "kodlab_mjbots_sdk/mjbots_behavior.h"
#include "real_time_tools/hard_spinner.hpp"
#include "real_time_tools/timer.hpp"

mjbots_behavior::mjbots_behavior(const Behavior_Options &options) :
                                abstract_realtime_object(options.realtime_params_.main_rtp, options.realtime_params_.can_cpu) {
  m_options = options;
  m_cpu = options.realtime_params_.can_cpu;
  m_realtime_priority = options.realtime_params_.main_rtp;
  m_frequency = options.frequency;
  m_num_motors = options.motor_list_.size();
  m_robot = std::make_unique<Realtime_Robot>(Realtime_Robot(options.motor_list_, options.realtime_params_,
                                                          options.max_torque, options.soft_start_duration));
}

void mjbots_behavior::run() {
  float prev_msg_duration = 0;

  real_time_tools::HardSpinner spinner;
  spinner.set_frequency(m_frequency);
  real_time_tools::Timer dt_timer;
  dt_timer.tic();
  real_time_tools::Timer message_duration_timer;

  while (!CTRL_C_DETECTED) {
    float sleep_duration = spinner.predict_sleeping_time();
    spinner.spin();
    calc_torques();
    prepare_log();
    add_timing_log(dt_timer.tac() * 1000, sleep_duration * 1000, prev_msg_duration);
    message_duration_timer.tic();
    m_robot->send_command();
    publish_log();
    m_robot->process_reply();
    prev_msg_duration = message_duration_timer.tac() * 1000;
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