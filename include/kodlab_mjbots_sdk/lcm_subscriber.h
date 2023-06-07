/**
 * @file lcm_subscriber.h
 * @author Ethan Musser (emusser@seas.upenn.edu)
 * @author J. Diego Caporale (jdcap@seas.upenn.edu)
 * @author Shane Rozen-Levy (srozen01@seas.upenn.edu)
 * @brief Provides LCM subscriber object for subscribing to one or multiple LCM
 * channels.
 * @date 7/7/22
 *
 * @copyright Copyright 2022 The Trustees of the University of Pennsylvania. All
 * rights reserved. BSD 3-Clause License
 *
 */

#pragma once

#include <map>
#include <mutex>
#include <string>
#include "lcm/lcm-cpp.hpp"
#include "real_time_tools/thread.hpp"
#include "kodlab_mjbots_sdk/abstract_realtime_object.h"
#include "kodlab_mjbots_sdk/lcm_message_handler.h"
#include "kodlab_mjbots_sdk/log.h"

namespace kodlab {

/**
 * @brief LCM subscriber capable of subscribing to multiple channels
 * @warning Running multiple `LcmSubscriber` objects on a single CPU can result
 * in concurrency issues.  The authors do not endorse this usage, do so at your
 * own risk.
 * @todo Move implementation to source file once `CTRL_C_DETECTED` global
 *       variable made available to TU.
 */
class LcmSubscriber : public AbstractRealtimeObject {
 public:

  /*!
   * @brief Constructor an lcm subscriber
   * @param realtime_priority realtime priority in range [1, 99]
   * @param cpu cpu for this process
   */
  LcmSubscriber(int realtime_priority, int cpu);

  /**
   * @brief Initialize the LCM subscriber thread.
   */
  void Init();

  /**
   * @brief Add a new LCM channel to the the LCM object's subscriptions
   * @tparam Message LCM message type for channel
   * @param channel_name channel name
   * @param handler `LcmMessageHandler`-inherited message handler object
   * @return generated `lcm::Subscription` object
   */
  template<class Message>
  lcm::Subscription *AddSubscription(std::string channel_name,
                                     LcmMessageHandler<Message> &handler) {
    auto sub = lcm_.subscribe(channel_name,
                              &LcmMessageHandler<Message>::HandleMessage,
                              &handler);
    subs_.emplace(channel_name, sub);
    return sub;
  }

  /**
   * @brief Remove a subscription by channel name
   * @note Warns and continues if channel does not exist.
   * @param channel_name channel name of subscription to be removed
   * @return 0 if unsubscribe successful, -1 if `channel_name` is not a valid
   * subscription
   */
  int RemoveSubscription(const std::string &channel_name);

  /**
   * @brief Accessor for `lcm::Subscription` objects
   * @param channel_name channel name
   * @return `lcm::Subscription` object on corresponding channel if subscription
   * exists, `nullptr` otherwise
   */
  [[nodiscard]] const lcm::Subscription *get_subscription(const std::string &channel_name) const;

 private:

  /**
   * @brief LCM object
   */
  lcm::LCM lcm_;

  /**
   * @brief Map of channel names to corresponding `lcm::Subscription` objects
   */
  std::map<std::string, lcm::Subscription *> subs_;

  /**
   * @brief Waits for LCM messages and exits when ctrl+c detected
   */
  void Run() override;

};


////////////////////////////////////////////////////////////////////////////////
// Implementation                                                             //
////////////////////////////////////////////////////////////////////////////////

inline LcmSubscriber::LcmSubscriber(int realtime_priority, int cpu) {
  cpu_ = cpu;
  realtime_priority_ = realtime_priority;
}

inline void LcmSubscriber::Init() {
  Start();
}

inline int LcmSubscriber::RemoveSubscription(const std::string &channel_name) {
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

[[nodiscard]] inline const lcm::Subscription *LcmSubscriber::get_subscription(const std::string &channel_name) const {
  if (subs_.count(channel_name) == 1) {
    return subs_.at(channel_name);
  } else {
    LOG_ERROR("Channel \"%s\" is not in subscription list.",
             channel_name.c_str());
    return nullptr;
  }
}

inline void LcmSubscriber::Run() {
  while (!CtrlCDetected()) {
    lcm_.handleTimeout(1000);
  }
}

} // kodlab

