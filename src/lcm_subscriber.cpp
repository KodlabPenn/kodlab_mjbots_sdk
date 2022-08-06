/**
 * @file lcm_subscriber.cpp
 * @author Ethan Musser (emusser@seas.upenn.edu)
 * @author J. Diego Caporale (jdcap@seas.upenn.edu)
 * @author Shane Rozen-Levy (srozen01@seas.upenn.edu)
 * @brief Provides LCM subscriber object for subscribing to onw or multiple LCM
 * channels.
 * @date 8/6/22
 * 
 * @copyright (c) Copyright 2022 The Trustees of the University of Pennsylvania.
 * All rights reserved. BSD 3-Clause License.
 * 
 */

#include <string>
#include "kodlab_mjbots_sdk/lcm_subscriber.h"
#include "lcm/lcm-cpp.hpp"

namespace kodlab {

LcmSubscriber::LcmSubscriber(int realtime_priority, int cpu) {
  cpu_ = cpu;
  realtime_priority_ = realtime_priority;
  Start();
}

[[nodiscard]] const lcm::Subscription *LcmSubscriber::get_subscription(const std::string &channel_name) const {
  return subs_.at(channel_name);
}

void LcmSubscriber::Run() {
  while (!CTRL_C_DETECTED) {
    lcm_.handleTimeout(1000);
  }
}

} // kodlab


