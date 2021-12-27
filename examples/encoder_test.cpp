// BSD 3-Clause License
// Copyright (c) 2021 The Trustees of the University of Pennsylvania. All Rights Reserved
// Authors:
// Shane Rozen-Levy <srozen01@seas.upenn.edu>

/* Basic example script demonstrating how to use the mjbots_control_loop to 2 motors. The functions to implement are
 * CalcTorques and PrepareLog. In this example we send a torque cmd of all zeros and log the motor information.
 */

#include "kodlab_mjbots_sdk/pi3hat.h"
#include <iostream>

int main(int argc, char **argv) {
  mjbots::pi3hat::Pi3Hat hat({});
  while (true){
    std::cout<<hat.readEncoder()<<std::endl;
    sleep(1);
  }
  return 0;
}
