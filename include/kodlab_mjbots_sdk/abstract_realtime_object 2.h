// BSD 3-Clause License
// Copyright (c) 2021 The Trustees of the University of Pennsylvania. All Rights Reserved
// Authors:
// Shane Rozen-Levy <srozen01@seas.upenn.edu>


#pragma once
#include "real_time_tools/thread.hpp"
#include "kodlab_mjbots_sdk/common_header.h"

namespace kodlab {
/*!
 * @brief template class for using a realtime thread with an object.
 * WARNING: unique ptr do not play nicely with the abstract realtime object due to the way the abstract realtime object
 * is setup. Instead consider using a shared_ptr and initialize the shared ptr from within the thread.
 * WARNING: Don't call Start from a constructor since inheritance is shaky during a constructor.
 */
class AbstractRealtimeObject {
 public:
  AbstractRealtimeObject();

  AbstractRealtimeObject(int realtime_priority, int cpu);

  /*!
   * @brief Destroy the AbstractRealtimeObject. Virtual destructor for proper
   * derived pointer destruction.
   */
  virtual ~AbstractRealtimeObject(){};

  /*!
   * @brief joins the realtime thread
   */
  void Join();

  /*!
   * @brief starts the realtime thread by calling StaticRun
   */
  void Start();

 protected:
  /*!
   * @brief a static function which runs the Run function of the passed in void ptr
   * @param abstract_void_ptr ptr to an abstract_realtime_object child class
   * @return nulptr
   */
  static void *StaticRun(void *abstract_void_ptr);

  /*!
   * @brief pure virtual function which contains the thread behavior. Must be implemented by child class
   */
  virtual void Run() = 0;

  int realtime_priority_ = 1;               /// Thread realtime priority
  int cpu_ = -1;                            /// CPU thread should Run on, less than 0 is any thread
  std::shared_ptr<real_time_tools::RealTimeThread> thread_ = nullptr;  /// realtime thread object
};
}  // namespace kodlab
