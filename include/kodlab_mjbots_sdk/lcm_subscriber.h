// BSD 3-Clause License
// Copyright (c) 2021 The Trustees of the University of Pennsylvania. All Rights Reserved
// Authors:
// Shane Rozen-Levy <srozen01@seas.upenn.edu>

#pragma once
#include <lcm/lcm-cpp.hpp>
#include "real_time_tools/thread.hpp"
#include "kodlab_mjbots_sdk/abstract_realtime_object.h"

/*!
 * @brief An template class for an lcm subscriber. Subscribes to just one msg of type msg_type.
 * @tparam MessageClass the lcm msg type
 */
template <class MessageClass>
class LcmSubscriber: public AbstractRealtimeObject{
 public:
  /*!
   * @brief constructor for lcm subscriber
   * @param realtime_priority the realtime priority, max is 99
   * @param cpu the cpu for this process
   * @param channel_name the string for the channel name
   */
  LcmSubscriber(int realtime_priority, int cpu, std::string channel_name);

  std::mutex mutex_;         /// Mutex for if m_data
  bool new_message_ = false; /// True if there is new data, false if data is old, used to prevent user for checking too often
  MessageClass data_;            /// A copy of the most recent lcm data
 protected:
  /*!
   * @brief callback function when msg is received. Copies the message to m_data
   * @param rbuf unknown, but not used
   * @param chan channel name
   * @param msg ptr to the incoming message data
   */
  void HandleMsg(const lcm::ReceiveBuffer* rbuf,
                 const std::string& chan,
                 const MessageClass* msg);

  /*!
   * @brief subscribes to the lcm channel using handle message and handles ctrl c detection
   */
  void Run() override;

  std::string channel_name_;
  lcm::LCM lcm_;
};

/************************************Implementation********************************************************************/
template<class msg_type>
void LcmSubscriber<msg_type>::Run() {
  lcm_.subscribe(channel_name_, &LcmSubscriber::HandleMsg, this);
  mutex_.unlock(); // Ensures mutex is unlocked
  std::cout << "Subscribing to " << channel_name_ << std::endl;
  while (!CTRL_C_DETECTED){
    lcm_.handleTimeout(1000);
  }
}

template<class msg_type>
LcmSubscriber<msg_type>::LcmSubscriber(int realtime_priority, int cpu, std::string channel_name):
  channel_name_(channel_name){
  cpu_ = cpu;
  realtime_priority_ = realtime_priority;
  if(!channel_name_.empty())
    Start();
}

template<class msg_type>
void LcmSubscriber<msg_type>::HandleMsg(const lcm::ReceiveBuffer *rbuf,
                                        const std::string &chan,
                                        const msg_type *msg) {
  // Lock mutex
  mutex_.lock();
  // Copy data
  data_ = *msg;
  // Let user know new message
  new_message_ = true;
  // Unlock mutex
  mutex_.unlock();
}