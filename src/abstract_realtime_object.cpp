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
  m_thread.join();
}

void *abstract_realtime_object::static_run(void *abstract_void_ptr) {
  abstract_realtime_object* ptr =
      (static_cast<abstract_realtime_object*>(abstract_void_ptr));
  ptr->run();
  return nullptr;
}

void abstract_realtime_object::start() {
  m_thread.parameters_.cpu_dma_latency_ = -1;
  m_thread.parameters_.priority_ = m_realtime_priority;
  if (m_cpu>=0){
    m_thread.parameters_.cpu_id_ = {m_cpu};
  }
  m_thread.create_realtime_thread(static_run, this);
}