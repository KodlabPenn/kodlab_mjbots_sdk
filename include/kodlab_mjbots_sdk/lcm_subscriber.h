/**
 * @file lcm_subscriber.h
 * @author Ethan Musser (emusser@seas.upenn.edu)
 * @author J. Diego Caporale (jdcap@seas.upenn.edu)
 * @author Shane Rozen-Levy (srozen01@seas.upenn.edu)
 * @brief Provides LCM subscriber and message handler objects.
 * @date 7/7/22
 *
 * @copyright Copyright 2022 The Trustees of the University of Pennsylvania. All
 * rights reserved. BSD 3-Clause License
 *
 */

#pragma once

#include <map>
#include <string>
#include "lcm/lcm-cpp.hpp"
#include "real_time_tools/thread.hpp"
#include "kodlab_mjbots_sdk/abstract_realtime_object.h"

namespace kodlab
{

/**
 * @brief Handler class for incoming LCM messages.
 * @tparam Message LCM message type
 */
template<class Message>
struct LcmMessageHandler
{
  /**
   * @brief Flag indicating that a new message is available.
   */
  bool new_message = false;

  /**
   * @brief Data for the most-recently received message.
   */
  Message data;

  /**
   * @brief Mutex for the message handler.
   */
  std::mutex mutex_;

  /**
   * @brief Constructs an `LcmMessageHandler` and ensures the mutex is unlocked.
   */
  LcmMessageHandler()
  {
    mutex_.unlock();  // ensures mutex is unlocked
  }

  /**
   * @brief Callback function for when an LCM message is received.
   * @details This function serves as a callback for incoming LCM messages on
   * `channel`.  It copies the decoded message data to `data`, and sets
   * `new_message` to `true`.
   * @param rbuf LCM recieve buffer containing raw bytes and timestamp
   * @param channel channel name
   * @param msg pointer to the incoming message data
   */
  void HandleMessage(const lcm::ReceiveBuffer *rbuf,
                     const std::string &channel,
                     const Message *msg)
  {
    mutex_.lock();
    data = *msg;
    new_message = true;
    mutex_.unlock();
  }
};

/**
 * @brief LCM subscriber capable of subscribing to multiple channels
 * @todo Implement subscription removal (i.e., unsubscribing)
 */
class LcmSubscriber : public AbstractRealtimeObject
{
public:

  /*!
   * @brief Constructor an lcm subscriber
   * @param realtime_priority realtime priority in range [1, 99]
   * @param cpu cpu for this process
   */
  LcmSubscriber(int realtime_priority, int cpu)
  {
    cpu_ = cpu;
    realtime_priority_ = realtime_priority;
    Start();
  }

  /**
   * @brief Add a new LCM channel to the the LCM object's subscriptions
   * @tparam Message LCM message type for channel
   * @param channel_name channel name
   * @param handler `LcmMessageHandler`-inherited message handler object
   * @return generated `lcm::Subscription` object
   */
  template<class Message>
  lcm::Subscription *AddSubscription(std::string channel_name,
                                     LcmMessageHandler<Message> &handler)
  {
    auto sub = lcm_.subscribe(channel_name,
                              &LcmMessageHandler<Message>::HandleMessage,
                              &handler);
    subs_.emplace(channel_name, sub);
    return sub;
  }

  /**
   * @brief Accessor for `lcm::Subscription` objects
   * @param channel_name channel name
   * @return `lcm::Subscription` object on corresponding channel
   */
  [[nodiscard]] const lcm::Subscription *get_subscription(const std::string &channel_name) const
  {
    return subs_.at(channel_name);
  }

protected:

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
  void Run() override
  {
    while (!CTRL_C_DETECTED)
    {
      lcm_.handleTimeout(1000);
    }
  }

};

} // kodlab

