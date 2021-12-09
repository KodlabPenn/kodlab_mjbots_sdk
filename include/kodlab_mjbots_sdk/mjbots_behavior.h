//
// Created by shane on 11/17/21.
//

#pragma once
#include "kodlab_mjbots_sdk/abstract_realtime_object.h"
#include "kodlab_mjbots_sdk/realtime_robot.h"

struct Behavior_Options{
  std::vector<Motor> motor_list_;
  Realtime_Params realtime_params_;

  float max_torque = 20;
  int soft_start_duration = 1000;
  int frequency = 1000;
};

class mjbots_behavior: public abstract_realtime_object{
 public:
  mjbots_behavior(const Behavior_Options& options);

 protected:

  void run() override;
  virtual void calc_torques() = 0;

  std::unique_ptr<Realtime_Robot> m_robot;
  int m_frequency;
  int m_num_motors;
  Behavior_Options m_options;
};


