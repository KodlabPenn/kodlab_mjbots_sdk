/**
 * @file lcm_subscriber.cpp
 * @author Ethan Musser (emusser@seas.upenn.edu)
 * @author J. Diego Caporale (jdcap@seas.upenn.edu)
 * @brief Source for LCM subscriber object.
 * @date 7/7/22
 *
 * @copyright Copyright 2022 The Trustees of the University of Pennsylvania. All
 * rights reserved. BSD 3-Clause License
 *
 */

#include "kodlab_mjbots_sdk/lcm_subscriber.h"

namespace kodlab{

LcmSubscriber::LcmSubscriber(int realtime_priority, int cpu) {
  cpu_ = cpu;
  realtime_priority_ = realtime_priority;
}

void LcmSubscriber::Init() {
  Start();
}

int LcmSubscriber::RemoveSubscription(const std::string &channel_name) {
  auto it = subs_.find(channel_name);
  if (it != subs_.end()) {
    int success = lcm_.unsubscribe(subs_[channel_name]);
    subs_.erase(it);
    return success;
  } else {
    LOG_WARN("Channel \"%s\" is not in subscription list.",
             channel_name.c_str());
  }
  return -1;
}

[[nodiscard]]  const lcm::Subscription *LcmSubscriber::get_subscription(const std::string &channel_name) const {
  if (subs_.count(channel_name) == 1) {
    return subs_.at(channel_name);
  } else {
    LOG_ERROR("Channel \"%s\" is not in subscription list.",
             channel_name.c_str());
    return nullptr;
  }
}

void LcmSubscriber::Run() {
  while (!CtrlCDetected()) {
    lcm_.handleTimeout(1000);
  }
}
} // namespace kodlab
