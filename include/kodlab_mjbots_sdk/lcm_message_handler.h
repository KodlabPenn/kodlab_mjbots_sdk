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
#include <optional>
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
class LcmMessageHandler {

 public:
  /**
   * @brief Constructs an `LcmMessageHandler` and ensures the mutex is unlocked.
   */
  LcmMessageHandler() {
    mutex_.unlock();  // ensures mutex is unlocked
  }

  /**
   * @brief Virtual destructor for clean inherited class destruction
   */
  virtual ~LcmMessageHandler() = default;

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
    data_ = *msg;
    new_message_ = true;
    mutex_.unlock();
  }

  /**
   * @brief Retrieve new data, if available.
   * @details Retrieve a `std::optional` object containing new data if the
   * internal `new_message_` flag evaluates to `true`, or `std::nullopt_t`
   * otherwise. If new data is retrieved, the new message flag is reset to
   * `false`.
   * @return `std::optional` containing message data if new message available,
   * `std::nullopt_t` otherwise
   */
  std::optional<Message> GetDataIfNew() {
    if (new_message_ && mutex_.try_lock()) {
      Message data_out = data_;
      new_message_ = false;
      mutex_.unlock();
      return data_out;
    }
    return {};
  }

  /**
   * @brief Check if new message is available.
   * @return `true` if new message available, `false` otherwise
   */
  [[nodiscard]] bool is_new_message() const { return new_message_; }

 private:

  /**
   * @brief Flag indicating that a new message is available.
   */
  bool new_message_ = false;

  /**
   * @brief Data for the most-recently received message.
   */
  Message data_;

  /**
   * @brief Mutex for the message handler.
   */
  std::mutex mutex_;

};

} // kodlab
