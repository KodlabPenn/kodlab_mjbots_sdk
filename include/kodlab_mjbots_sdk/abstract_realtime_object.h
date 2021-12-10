//
// Created by shane on 11/18/21.
//

#pragma once
#include "real_time_tools/thread.hpp"
#include "kodlab_mjbots_sdk/common_header.h"

/*!
 * @brief template class for using a realtime thread with an object.
 * WARNING: unique ptr do not play nicely with the abstract realtime object due to the way the abstract realtime object
 * is setup. Instead consider using a shared_ptr and initialize the shared ptr from within the thread.
 * WARNING: Don't call start from a constructor since inheritance is shaky during a constructor.
 */
class Abstract_Realtime_Object {
 public:
  Abstract_Realtime_Object(int realtime_priority, int cpu);

  /*!
   * @brief joins the realtime thread
   */
  void join();

  /*!
   * @brief starts the realtime thread by calling static_run
   */
  void start();

 protected:
  /*!
   * @brief a static function which runs the run function of the passed in void ptr
   * @param abstract_void_ptr ptr to an abstract_realtime_object child class
   * @return nulptr
   */
  static void* static_run(void* abstract_void_ptr);

  /*!
   * @brief pure virtual function which contains the thread behavior. Must be implemented by child class
   */
  virtual void run() = 0;

  int m_realtime_priority;                  /// Thread realtime priority
  int m_cpu;                                /// CPU thread should run on, less than 0 is any thread
  real_time_tools::RealTimeThread m_thread; /// realtime thread object
};

