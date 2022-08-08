/**
 * @file lcm_publisher.h
 * @author Ethan J. Musser (emusser@seas.upenn.edu)
 * @brief Provides `LcmPublisher` class which handles storage and publishing of
 * LCM messages.
 * @date 8/7/22
 * 
 * @copyright (c) Copyright 2022 The Trustees of the University of Pennsylvania.
 * All rights reserved.
 * 
 */

#pragma once

#include <memory>
#include <string>

#include "lcm/lcm-cpp.hpp"

namespace kodlab {

/**
 * @brief LCM message data container and publisher
 * @note One way to use the `lcm::LCM` object is to have a single object which
 * publishes multiple messages on multiple channels. To support this usage, the
 * internal `lcm::LCM` object is stored as a shared pointer, which is set during
 * construction and accessible later on. this pointer can be passed around and
 * used in multiple `LcmPublisher` objects.
 * @tparam Message LCM message type
 */
template<class Message>
class LcmPublisher {

 public:

  /**
   * @brief Construct an LCM publisher object without an `lcm::LCM` object or
   * channel.
   */
  LcmPublisher() = default;

  /**
   * @brief Construct an LCM publisher object with a shared LCM object.
   * @param lcm `lcm::LCM` object shared pointer
   * @param channel channel name for publishing
   */
  LcmPublisher(std::shared_ptr<lcm::LCM> lcm, const std::string &channel)
      : lcm_(lcm), channel_(channel), message_(std::make_shared<Message>()) {}

  /**
   * @brief Construct an LCM publisher object with an owned LCM object.
   * @param lcm `lcm::LCM` object rvalue (must transfer ownership)
   * @param channel channel name for publishing
   */
  LcmPublisher(lcm::LCM &&lcm, const std::string &channel)
      : LcmPublisher(std::make_shared<lcm::LCM>(std::move(lcm)), channel) {}

  /**
   * @brief Publish the data stored in `message_` on LCM channel `channel_`.
   */
  void Publish() {
    lcm_->publish<Message>(channel_, message_.get());
  }

  /**
   * @brief Retrieve shared pointer to the LCM object used for publishing.
   * @return shared pointer to `lcm::LCM` object
   */
  [[nodiscard]] std::shared_ptr<lcm::LCM> get_lcm() const { return lcm_; }

  /**
   * @brief Set the object used for publishing as a new, shared LCM object.
   * @param lcm shared pointer to `lcm::LCM` object
   */
  void set_lcm(std::shared_ptr<lcm::LCM> lcm) { lcm_ = lcm; }

  /**
   * @brief Set the object used for publishing as a new, owned LCM object.
   * @param lcm `lcm::LCM` object rvalue (must transfer ownership)
   */
  void set_lcm(lcm::LCM &&lcm) {
    lcm_ = std::make_shared<lcm::LCM>(std::move(lcm));
  }

  /**
   * @brief Retrieve the LCM publishing channel name.
   * @return name of LCM channel used for publishing
   */
  [[nodiscard]] const std::string get_channel() const { return channel_; }

  /**
   * @brief Set the LCM channel used for publishing.
   * @param channel name of LCM channel to be used for publishing
   */
  void set_channel(const std::string &channel) { channel_ = channel; }

  /**
   * @brief Retrieve a shared pointer to the internally stored message data.
   * @return shared pointer to internal message data
   */
  [[nodiscard]] const std::shared_ptr<Message> get_message_shared_ptr() const {
    return message_;
  }

  /**
   * @brief Retrieve a raw pointer to the internally stored message data.
   * @return raw pointer to internal message data
   */
  [[nodiscard]] const Message *get_message_ptr() const {
    return message_.get();
  }

  /**
   * @brief Retrieve read-only version of internally stored message data.
   * @return read-only copy of internal message data
   */
  [[nodiscard]] const Message get_message_data() const { return *message_; }

 private:

  /**
   * @brief LCM object used for publishing.
   * @note This object can be shared by multiple `LcmPublishers`.
   */
  std::shared_ptr<lcm::LCM> lcm_;

  /**
   * @brief LCM publishing channel name.
   */
  std::string channel_;

  /**
   * @brief Internal message that is published on calls to `Publish`.
   */
  std::shared_ptr<Message> message_;

};

} // kodlab
