//
// Created by shane on 11/18/21.
//

#include <unistd.h>
#include <real_time_tools/process_manager.hpp>
#include "kodlab_mjbots_sdk/abstract_realtime_object.h"
#include <iostream>
Abstract_Realtime_Object::Abstract_Realtime_Object(int realtime_priority, int cpu): m_realtime_priority(realtime_priority),
                                                                                    m_cpu(cpu){
}

void Abstract_Realtime_Object::join() {
  m_thread.join();
}

void *Abstract_Realtime_Object::static_run(void *abstract_void_ptr) {
  Abstract_Realtime_Object* ptr =
      (static_cast<Abstract_Realtime_Object*>(abstract_void_ptr));
  sleep(1);
  ptr->run();
  return nullptr;
}

void Abstract_Realtime_Object::start() {
  // Setup realtime thread and then start
  m_thread.parameters_.cpu_dma_latency_ = -1;
  m_thread.parameters_.priority_ = m_realtime_priority;
  if (m_cpu>=0){
    m_thread.parameters_.cpu_id_ = {m_cpu};
  }
  m_thread.parameters_.block_memory_=true;
  m_thread.create_realtime_thread(static_run, this);
}