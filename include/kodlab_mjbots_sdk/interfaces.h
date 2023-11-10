#pragma once 

#define INTERFACE_INCLUDED

#include "kodlab_mjbots_sdk/robot_interface.h"

#ifdef SIMULATION
  #include "kodlab_mjbots_sdk/mjbots_simulation_interface.h"
  typedef kodlab::mjbots::MjbotsSimulationInterface INTERFACE_TYPE;
#else 
  #include "kodlab_mjbots_sdk/mjbots_hardware_interface.h"
  typedef kodlab::mjbots::MjbotsHardwareInterface INTERFACE_TYPE;
#endif


#undef INTERFACE_INCLUDED
