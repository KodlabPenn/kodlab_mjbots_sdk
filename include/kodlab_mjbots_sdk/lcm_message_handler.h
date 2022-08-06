/**
 * @file lcm_message_handler.h
 * @author Ethan Musser (emusser@seas.upenn.edu)
 * @brief Provides LCM message handler object to be derived or composed into
 * classes requiring LCM input via the `LcmSubscriber` object.
 * @date 8/6/22
 * 
 * @copyright Copyright 2022 The Trustees of the University of Pennsylvania. All
 * rights reserved.
 * 
 */

#pragma once

#include <string>
#include <mutex>
#include "lcm/lcm-cpp.hpp"
#include "real_time_tools/thread.hpp"

namespace kodlab {

/**
 * @brief Handler class for incoming LCM messages.
 * @details This class is intended to be derived from or composed into objects
 * requiring LCM input.  This class and its children should be compatible with
 * the `LcmSubscriber` object, and can implemented with the
 * `LcmSubscriber::AddSubscription` method in particular.
 * @tparam Message LCM message type
 */
template<class Message>
struct LcmMessageHandler {
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
  LcmMessageHandler() {
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
                     const Message *msg) {
    mutex_.lock();
    data = *msg;
    new_message = true;
    mutex_.unlock();
  }

};

} // kodlab
