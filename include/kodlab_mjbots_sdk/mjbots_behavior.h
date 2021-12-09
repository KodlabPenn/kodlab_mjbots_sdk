//
// Created by shane on 11/17/21.
//

#pragma once
#include "kodlab_mjbots_sdk/abstract_realtime_object.h"
#include "kodlab_mjbots_sdk/realtime_robot.h"
#include "lcm/lcm-cpp.hpp"
#include "real_time_tools/timer.hpp"
#include "real_time_tools/hard_spinner.hpp"

struct Behavior_Options{
  std::vector<Motor> motor_list_;
  Realtime_Params realtime_params_;

  float max_torque = 20;
  int soft_start_duration = 1000;
  int frequency = 1000;
  std::string channel_name;
};

template<class log_type = void>
class mjbots_behavior: public abstract_realtime_object{
 public:
  mjbots_behavior(const Behavior_Options &options);

 protected:

  void run() override;
  virtual void calc_torques() = 0;
  virtual void prepare_log(){};
  void add_timing_log(float t, float margin, float message_duration);
  void publish_log();

  std::unique_ptr<Realtime_Robot> m_robot;
  int m_frequency;
  int m_num_motors;
  Behavior_Options m_options;
  bool m_logging = false;
  std::string m_channel_name;
  lcm::LCM m_lcm;
  log_type m_log_data;
};

template<class log_type>
mjbots_behavior<log_type>::mjbots_behavior(const Behavior_Options &options) :
    abstract_realtime_object(options.realtime_params_.main_rtp, options.realtime_params_.can_cpu) {
  m_options = options;
  m_cpu = options.realtime_params_.can_cpu;
  m_realtime_priority = options.realtime_params_.main_rtp;
  m_frequency = options.frequency;
  m_num_motors = options.motor_list_.size();
  m_robot = std::make_unique<Realtime_Robot>(Realtime_Robot(options.motor_list_, options.realtime_params_,
                                                            options.max_torque, options.soft_start_duration));
  m_channel_name = options.channel_name;
  m_logging = !m_channel_name.empty();
  std::cout<<"robot joint pos size = "<<m_robot->get_joint_positions().size()<<std::endl;
}


template<class log_type>
void mjbots_behavior<log_type>::add_timing_log(float t, float margin, float message_duration) {
  if(m_logging){
    m_log_data.timestamp = t;
    m_log_data.margin = margin;
    m_log_data.message_duration = message_duration;
  }
}

template<class log_type>
void mjbots_behavior<log_type>::publish_log() {
  if(m_logging)
    m_lcm.publish(m_channel_name, &m_log_data);
}

template<class log_type>
void mjbots_behavior<log_type>::run() {
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
