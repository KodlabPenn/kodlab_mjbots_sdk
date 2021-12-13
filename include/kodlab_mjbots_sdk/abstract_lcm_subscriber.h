// BSD 3-Clause License
// Copyright (c) 2021 The Trustees of the University of Pennsylvania. All Rights Reserved
// Authors:
// Shane Rozen-Levy <srozen01@seas.upenn.edu>

#pragma once
#include <lcm/lcm-cpp.hpp>
#include "real_time_tools/thread.hpp"
#include "kodlab_mjbots_sdk/abstract_realtime_object.h"

/*!
 * @brief An abstract template class for an lcm subscriber. Subscribes to just one msg of type msg_type. To use create
 * child class and implement handle_msg to take process the incoming msg
 * @tparam msg_type the lcm msg type
 */
template <class msg_type>
class Lcm_Subscriber: public Abstract_Realtime_Object{
 public:
  /*!
   * @brief constructor for abstract lcm subscriber
   * @param realtime_priority the realtime priority, max is 99
   * @param cpu the cpu for this process
   * @param channel_name the string for the channel name
   */
  Lcm_Subscriber(int realtime_priority, int cpu, std::string channel_name);

  std::mutex m_mutex;
  bool m_new_message;
  msg_type m_data;
 protected:
  /*!
   * @brief callback function when msg is received. To be overriden by child class
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
  std::cout<<"Subscribing to "<<m_channel_name<<std::endl;
  while (!CTRL_C_DETECTED){
    m_lcm.handleTimeout(100000);
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
  m_mutex.lock();
  m_data = *msg;
  m_new_message = true;
  m_mutex.unlock();
}