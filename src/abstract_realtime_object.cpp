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
  std::cout<<"foo"<<std::endl;
  abstract_realtime_object* ptr =
      (static_cast<abstract_realtime_object*>(abstract_void_ptr));
  ptr->set_up_cpu_run();
  std::cout<<"foo"<<std::endl;
  return nullptr;
}

void abstract_realtime_object::set_up_cpu_run() {
  std::vector<int> cpu = {m_cpu};
  real_time_tools::fix_current_process_to_cpu(cpu, ::getpid());

  std::cout<<"run"<<std::endl;
  run();
}

void abstract_realtime_object::start() {
  m_thread.parameters_.cpu_dma_latency_ = -1;
  m_thread.parameters_.priority_ = m_realtime_priority;
  std::cout<<"Startng thread"<<std::endl;
  m_thread.create_realtime_thread(static_run, this);
}
