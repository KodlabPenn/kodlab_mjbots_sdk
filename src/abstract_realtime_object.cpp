//
// Created by shane on 11/18/21.
//

#include <unistd.h>
#include <real_time_tools/process_manager.hpp>
#include "kodlab_mjbots_sdk/abstract_realtime_object.h"
#include <iostream>
abstract_realtime_object::abstract_realtime_object(int realtime_priority, int cpu):m_realtime_priority(realtime_priority),
                                                                                   m_cpu(cpu){
}

void abstract_realtime_object::join() {
  m_thread->join();
}

void *abstract_realtime_object::static_run(void *abstract_void_ptr) {
  abstract_realtime_object* ptr =
      (static_cast<abstract_realtime_object*>(abstract_void_ptr));
  ptr->set_up_cpu_run();
  return nullptr;
}

void abstract_realtime_object::set_up_cpu_run() {
  if(m_cpu > 0){
    std::vector<int> cpu = {m_cpu};
    real_time_tools::fix_current_process_to_cpu(cpu, ::getpid());
  }
  run();
}

void abstract_realtime_object::start() {
  m_thread.reset(new real_time_tools::RealTimeThread());
  m_thread->parameters_.cpu_dma_latency_ = -1;
  m_thread->parameters_.priority_ = m_realtime_priority;
  m_thread->create_realtime_thread(static_run, this);
}

abstract_realtime_object::abstract_realtime_object() {

}
