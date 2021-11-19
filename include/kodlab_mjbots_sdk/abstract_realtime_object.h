//
// Created by shane on 11/18/21.
//

#pragma once
#include "real_time_tools/thread.hpp"
#include "kodlab_mjbots_sdk/common_header.h"
class abstract_realtime_object {
 public:

  abstract_realtime_object(int realtime_priority, int cpu);

  /*!
   * @brief joins the realtime thread
   */
  void join();

 protected:
  static void* static_run(void* abstract_void_ptr);

  virtual void run() = 0;

  void set_up_cpu_run();

  /*!
   * @brief starts the realtime thread
   */
  void start();


  int m_realtime_priority;
  int m_cpu;
  real_time_tools::RealTimeThread m_thread;


};

