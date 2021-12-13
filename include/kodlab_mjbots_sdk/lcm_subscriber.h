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
 * @tparam msg_type the lcm msg type
 */
template <class msg_type>
class Lcm_Subscriber: public Abstract_Realtime_Object{
 public:
  /*!
   * @brief constructor for lcm subscriber
   * @param realtime_priority the realtime priority, max is 99
   * @param cpu the cpu for this process
   * @param channel_name the string for the channel name
   */
  Lcm_Subscriber(int realtime_priority, int cpu, std::string channel_name);

  std::mutex m_mutex;         /// Mutex for if m_data
  bool m_new_message = false; /// True if there is new data, false if data is old, used to prevent user for checking too often
  msg_type m_data;            /// A copy of the most recent lcm data
 protected:
  /*!
   * @brief callback function when msg is received. Copies the message to m_data
   * @param rbuf unknown, but not used
   * @param chan channel name
   * @param msg ptr to the incoming message data
   */
  void handle_msg(const lcm::ReceiveBuffer* rbuf,
                          const std::string& chan,
                          const msg_type* msg);

  /*!
   * @brief subscribes to the lcm channel using handle message and handles ctrl c detection
   */
  void run() override;

  std::string m_channel_name;
  lcm::LCM m_lcm;
};

/************************************Implementation********************************************************************/
template<class msg_type>
void Lcm_Subscriber<msg_type>::run() {
  m_lcm.subscribe(m_channel_name, &Lcm_Subscriber::handle_msg, this);
  m_mutex.unlock(); // Ensures mutex is unlocked
  std::cout<<"Subscribing to "<<m_channel_name<<std::endl;
  while (!CTRL_C_DETECTED){
    m_lcm.handleTimeout(1000);
  }
}

template<class msg_type>
Lcm_Subscriber<msg_type>::Lcm_Subscriber(int realtime_priority, int cpu, std::string channel_name):
  m_channel_name(channel_name){
  m_cpu = cpu;
  m_realtime_priority = realtime_priority;
  if(!m_channel_name.empty())
    start();
}

template<class msg_type>
void Lcm_Subscriber<msg_type>::handle_msg(const lcm::ReceiveBuffer *rbuf,
                                          const std::string &chan,
                                          const msg_type *msg) {
  // Lock mutex
  m_mutex.lock();
  // Copy data
  m_data = *msg;
  // Let user know new message
  m_new_message = true;
  // Unlock mutex
  m_mutex.unlock();
}