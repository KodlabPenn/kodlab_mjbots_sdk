//
// Created by shane on 11/17/21.
//

#pragma once
#include "kodlab_mjbots_sdk/abstract_realtime_object.h"
#include "kodlab_mjbots_sdk/realtime_robot.h"
#include "lcm/lcm-cpp.hpp"

struct Behavior_Options{
  std::vector<Motor> motor_list_;
  Realtime_Params realtime_params_;

  float max_torque = 20;
  int soft_start_duration = 1000;
  int frequency = 1000;
};

class mjbots_behavior: public abstract_realtime_object{
 public:
  mjbots_behavior(const Behavior_Options &options);

 protected:

  void run() override;
  virtual void calc_torques() = 0;
  virtual void prepare_log(){};
  virtual void add_timing_log(float t, float margin, float message_duration){};
  virtual void publish_log(){};

  std::unique_ptr<Realtime_Robot> m_robot;
  int m_frequency;
  int m_num_motors;
  Behavior_Options m_options;
};


template <class logging_data_type>
class mjbots_behavior_logging : public mjbots_behavior{
 public:
  mjbots_behavior_logging(const Behavior_Options& options, const std::string channel_name);

 protected:
  void prepare_log() override = 0;
  void add_timing_log(float t, float margin, float message_duration) override;
  void publish_log() override;
  std::string m_channel_name;
  lcm::LCM m_lcm;
  logging_data_type m_log_data;
};

template<class logging_data_type>
mjbots_behavior_logging<logging_data_type>::mjbots_behavior_logging(const Behavior_Options &options,
                                                                    const std::string channel_name):
    mjbots_behavior(options),
    m_channel_name(channel_name) {}

template<class logging_data_type>
void mjbots_behavior_logging<logging_data_type>::add_timing_log(float t, float margin, float message_duration) {
  m_log_data.timestamp = t;
  m_log_data.margin = margin;
  m_log_data.message_duration = message_duration;
}

template<class logging_data_type>
void mjbots_behavior_logging<logging_data_type>::publish_log() {
  m_lcm.publish(m_channel_name, &m_log_data);
}
