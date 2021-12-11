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
class Abstract_Lcm_Subscriber: public Abstract_Realtime_Object{
 public:
  /*!
   * @brief constructor for abstract lcm subscriber
   * @param realtime_priority the realtime priority, max is 99
   * @param cpu the cpu for this process
   * @param channel_name the string for the channel name
   */
  Abstract_Lcm_Subscriber(int realtime_priority, int cpu, std::string channel_name);

 protected:
  /*!
   * @brief callback function when msg is received. To be overriden by child class
   * @param rbuf unknown, but not used
   * @param chan channel name
   * @param msg ptr to the incoming message data
   */
  virtual void handle_msg(const lcm::ReceiveBuffer* rbuf,
                          const std::string& chan,
                          const msg_type* msg) = 0;

  /*!
   * @brief subscribes to the lcm channel using handle message and handles ctrl c detection
   */
  void run() override;

  std::string m_channel_name;
  lcm::LCM m_lcm;
};

/************************************Implementation********************************************************************/
template<class msg_type>
void Abstract_Lcm_Subscriber<msg_type>::run() {
  m_lcm.subscribe(m_channel_name, &Abstract_Lcm_Subscriber::handle_msg, this);
  while (!CTRL_C_DETECTED){
    m_lcm.handleTimeout(100000);
  }
}

template<class msg_type>
Abstract_Lcm_Subscriber<msg_type>::Abstract_Lcm_Subscriber(int realtime_priority, int cpu, std::string channel_name):
  m_channel_name(channel_name){
  m_cpu = cpu;
  m_realtime_priority = realtime_priority;
  start();
}


