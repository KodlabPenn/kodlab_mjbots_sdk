// BSD 3-Clause License
// Copyright (c) 2021 The Trustees of the University of Pennsylvania. All Rights Reserved
// Authors:
// Shane Rozen-Levy <srozen01@seas.upenn.edu>


#include "real_time_tools/timer.hpp"
#include "kodlab_mjbots_sdk/abstract_realtime_object.h"
namespace kodlab {
AbstractRealtimeObject::AbstractRealtimeObject(int realtime_priority, int cpu) : realtime_priority_(realtime_priority),
                                                                                 cpu_(cpu) {
}

void AbstractRealtimeObject::Join() {
  thread_.join();
}

void *AbstractRealtimeObject::StaticRun(void *abstract_void_ptr) {
  AbstractRealtimeObject *ptr =
      (static_cast<AbstractRealtimeObject *>(abstract_void_ptr));
  real_time_tools::Timer::sleep_ms(10);
  ptr->Run();
  return nullptr;
}

void AbstractRealtimeObject::Start() {
  // Setup realtime thread and then Start
  thread_.parameters_.cpu_dma_latency_ = -1;
  thread_.parameters_.priority_ = realtime_priority_;
  if (cpu_ >= 0) {
    thread_.parameters_.cpu_id_ = {cpu_};
  }
  thread_.parameters_.block_memory_ = true;
  // Create realtime_thread takes a static function and a void*. In order to get the Run function inside the control loop
  // we call the StaticRun function which takes in a void ptr to a Abstract_Realtime_Object and then runs the Run function
  thread_.create_realtime_thread(StaticRun, this);
}
AbstractRealtimeObject::AbstractRealtimeObject() {

}
} // namespace kodlab