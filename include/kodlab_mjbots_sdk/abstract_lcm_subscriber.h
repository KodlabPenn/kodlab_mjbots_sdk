//
// Created by shane on 11/17/21.
//

#pragma once
#include <lcm/lcm-cpp.hpp>
#include "real_time_tools/thread.hpp"
#include "kodlab_mjbots_sdk/abstract_realtime_object.h"

template <class msg_type>
class abstract_lcm_subscriber: public abstract_realtime_object{
 public:
  /*!
   * @brief constructor for abstract lcm subscriber
   * @param realtime_priority the realtime priority, max is 99
   * @param cpu the cpu for this process
   * @param channel_name the string for the channel name
   */
  abstract_lcm_subscriber(int realtime_priority, int cpu, std::string channel_name);

 protected:
//  void* static_run(void *abstract_void_ptr);
  virtual void handle_msg(const lcm::ReceiveBuffer* rbuf,
                          const std::string& chan,
                          const msg_type* msg) = 0;

  void run() override;

  std::string m_channel_name;
  lcm::LCM m_lcm;
};

template<class msg_type>
void abstract_lcm_subscriber<msg_type>::run() {
  std::cout<< m_channel_name<<std::endl;
  m_lcm.subscribe(m_channel_name, &abstract_lcm_subscriber::handle_msg, this);
  std::cout<<"subscribe successful"<<std::endl;

  while (!CTRL_C_DETECTED){
    m_lcm.handleTimeout(100000);
  }
}

//template<class msg_type>
//void *abstract_lcm_subscriber<msg_type>::static_run(void *abstract_void_ptr) {
//  std::cout<<"foo22"<<std::endl;
//  abstract_lcm_subscriber<msg_type>* ptr =
//      (static_cast<abstract_lcm_subscriber*>(abstract_void_ptr));
//  ptr->set_up_cpu_run();
//  return nullptr;
//}


template<class msg_type>
abstract_lcm_subscriber<msg_type>::abstract_lcm_subscriber(int realtime_priority, int cpu, std::string channel_name):
m_channel_name(channel_name){
  m_cpu = cpu;
  m_realtime_priority = realtime_priority;
  m_thread.reset(new real_time_tools::RealTimeThread());
  m_thread->parameters_.cpu_dma_latency_ = -1;
  m_thread->parameters_.priority_ = m_realtime_priority;
  std::cout<<"Startng thread"<<std::endl;
  m_thread->create_realtime_thread(static_run, this);
  std::cout<<"huh"<<std::endl;
}


