//
// Created by shane on 11/17/21.
//

#pragma once
#include <lcm/lcm-cpp.hpp>
#include "real_time_tools/thread.hpp"

template <class msg_type>
class abstract_lcm_subscriber {
 public:
  abstract_lcm_subscriber(int realtime_priority, int cpu);

  void start();

  void join();

 private:
  virtual void handle_msg(const lcm::ReceiveBuffer* rbuf,
                          const std::string& chan,
                          const msg_type* msg) = 0;

  static void* static_run(void* abstract_lcm_subscriber_void_ptr);

  void run();

  int m_realtime_priority = 90;
  int m_cpu = 4;
  real_time_tools::RealTimeThread m_thread;
  lcm::LCM m_lcm;
};

template<class msg_type>
void abstract_lcm_subscriber<msg_type>::run() {
  std::vector<int> cpu = {m_cpu};
  real_time_tools::fix_current_process_to_cpu(cpu, ::getpid());
  while (!CTRL_C_DETECTED){
    m_lcm.handleTimeout(1000);
  }
}

template<class msg_type>
void *abstract_lcm_subscriber<msg_type>::static_run(void *abstract_lcm_subscriber_void_ptr) {
  abstract_lcm_subscriber<msg_type>* subscriber =
      (static_cast<abstract_lcm_subscriber*>(abstract_lcm_subscriber_void_ptr));
  subscriber->run();
  return nullptr;
}

template<class msg_type>
void abstract_lcm_subscriber<msg_type>::start() {
  m_thread.parameters_.cpu_dma_latency_ = -1;
  m_thread.parameters_.priority_ = m_realtime_priority;
  m_thread.create_realtime_thread(static_run, this);

}
template<class msg_type>
void abstract_lcm_subscriber<msg_type>::join() {
  m_thread.join();
}

template<class msg_type>
abstract_lcm_subscriber<msg_type>::abstract_lcm_subscriber(int realtime_priority, int cpu):
                                                                                          m_realtime_priority(realtime_priority),
                                                                                          m_cpu(cpu){}


