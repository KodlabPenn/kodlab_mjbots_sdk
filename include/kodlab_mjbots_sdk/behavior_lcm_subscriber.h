// BSD 3-Clause License
// Copyright (c) 2021 The Trustees of the University of Pennsylvania. All Rights Reserved
// Authors:
// Shane Rozen-Levy <srozen01@seas.upenn.edu>

#pragma once
#include <string>
#include <map>
#include "lcm/lcm-cpp.hpp"
#include "real_time_tools/thread.hpp"
#include "kodlab_mjbots_sdk/abstract_realtime_object.h"

namespace kodlab {
/*!
 * @brief Class for a behavior lcm subscriber. subscribes to all behavior messages
 */
class BehaviorLcmSubscriber : public AbstractRealtimeObject {
 public:
  /*!
   * @brief constructor for behavior lcm subscriber
   * @param realtime_priority the realtime priority, max is 99
   * @param cpu the cpu for this process
   */
  BehaviorLcmSubscriber(int realtime_priority, int cpu);

  std::mutex mutex_;         /// Mutex for any data changing
  bool new_message_ =
      false; /// True if there is new data, false if data is old, used to prevent user for checking too often
 protected:
  /*!
   * @brief callback function when msg is received. Copies the message to m_data
   * @param rbuf unknown, but not used
   * @param chan channel name
   * @param msg ptr to the incoming message data
   * @tparam MessageClass the class of the data
   */
  
  template<class MessageClass>
  void HandleMsg(const lcm::ReceiveBuffer *rbuf,
                 const std::string &chan,
                 const MessageClass *msg);

  /**
   * @brief Adds a subscriber to the lcm object and to the useful mpas.
   * 
   * @tparam MessageClass the lcmgen data struct type
   * @param input_channel_name unique channel name
   * @param behavior_input_data shared pointer to behavior lcmgen data struct
   */
  template<class MessageClass>
  void AddSubscriber(std::string input_channel_name, std::shared_ptr<MessageClass> behavior_input_data);

  /*!
   * @brief subscribes to the lcm channel using handle message and handles ctrl c detection
   */
  void Run() override;

  lcm::LCM lcm_;
  //This will work, but is likely dangerous? Very C, not C++
  std::map<std::string, std::shared_ptr<void>> NameToDataPtr; // Map between the channel names and the location of their data
  std::map<std::string, lcm::Subscription*> NameToSubscriptionPtr; // Map between the channel names and the Subscription*
};

/************************************Implementation********************************************************************/

void BehaviorLcmSubscriber::Run() 
{
  mutex_.unlock(); // Ensures mutex is unlocked
  while (!CTRL_C_DETECTED) {
    lcm_.handleTimeout(10);
  }
}
template<class MessageClass>
void BehaviorLcmSubscriber::AddSubscriber(std::string input_channel_name, 
                                          std::shared_ptr<MessageClass> behavior_input_data)
{
  std::cout << "Subscribing to " << input_channel_name << std::endl;
  lcm::Subscription* sub_ptr = lcm_.subscribe(input_channel_name, &BehaviorLcmSubscriber::HandleMsg<MessageClass>, this);
  NameToSubscriptionPtr.emplace (input_channel_name,sub_ptr);
  NameToDataPtr.emplace (input_channel_name,behavior_input_data);

}

BehaviorLcmSubscriber::BehaviorLcmSubscriber(int realtime_priority, int cpu)
{
  cpu_ = cpu;
  realtime_priority_ = realtime_priority;
  Start();
}

template<class msg_type>
void BehaviorLcmSubscriber::HandleMsg(const lcm::ReceiveBuffer *rbuf,
                                        const std::string &chan,
                                        const msg_type *msg) 
{
  // Lock mutex
  mutex_.lock();
  // Copy data
  *static_cast<msg_type*>(NameToDataPtr[chan]) = *msg;
  // Let user know new message
  new_message_ = true;
  // Unlock mutex
  mutex_.unlock();
}
} // namespace kodlab