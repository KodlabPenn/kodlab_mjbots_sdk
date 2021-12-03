
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/// @file
///
/// This is a simple application that demonstrates how to efficiently
/// monitor and control multiple moteus servos at a high rate using
/// the pi3hat.
///
/// It is contained in a single file for the purposes of
/// demonstration.  A real application should likely be implemented in
/// multiple translation units or structured for longer term
/// maintenance.

#include <sys/mman.h>

#include <algorithm>
#include <iostream>
#include <future>
#include <limits>
#include <map>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>
#include <math.h>
#include "real_time_tools/spinner.hpp"
#include "real_time_tools/hard_spinner.hpp"

#include "real_time_tools/thread.hpp"
#include "real_time_tools/timer.hpp"
#include <lcm/lcm-cpp.hpp>
#include <real_time_tools/realtime_check.hpp>
#include <real_time_tools/process_manager.hpp>
#include "many_motor_log.hpp"
#include "kodlab_mjbots_sdk/realtime_robot.h"
#include "kodlab_mjbots_sdk/cartesian_leg.h"
#include "kodlab_mjbots_sdk/abstract_lcm_subscriber.h"

using namespace mjbots;

using MoteusInterface = moteus::Pi3HatMoteusInterface;

/// This holds the user-defined control logic.
class SampleController {
 public:
  SampleController() {
    robot = std::make_unique<Realtime_Robot>(Realtime_Robot(servo_id_list.size(),
                                                            servo_id_list,
                                                            bus_map,
                                                            3,
                                                            {0.0, 0.0, 0.0, 0, 0, 0, 0, 0, 0, 0},
                                                            {1, 1, 1, 1, 1, 1, 1, 1, 1, 1},
                                                            12,
                                                            5000));
  }

  void calc_torques() {
    std::vector<float> torques = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

    robot->set_torques(torques);
  }

  void process_reply() {
    robot->process_reply();
  }

  void send_command() {
    robot->send_command();
  }

  void prepare_log(many_motor_log &my_data) {
    for (int servo = 0; servo < servo_id_list.size(); servo++) {
      my_data.positions[servo] = robot->get_joint_positions()[servo];
      my_data.velocities[servo] = robot->get_joint_velocities()[servo];
      my_data.modes[servo] = static_cast<int>(robot->get_joint_modes()[servo]);
      my_data.torques[servo] = robot->get_joint_torque_cmd()[servo];
    }
  }

  void *Run() {
    int cycle_count = 0;
    std::vector<int> cpu = {2};
    real_time_tools::fix_current_process_to_cpu(cpu, ::getpid());

    lcm::LCM lcm;
    //leg_gain_subscriber.start();

    many_motor_log my_data{};
    real_time_tools::HardSpinner spinner;
    spinner.set_frequency(500);
    real_time_tools::Timer dt_timer;
    dt_timer.tic();

    real_time_tools::Timer message_duration_timer;

    // We will run at a fixed cycle time.
    while (!CTRL_C_DETECTED) {

      cycle_count ++;

      double sleep_duration = spinner.predict_sleeping_time();
      spinner.spin();
      calc_torques();

      prepare_log(my_data);

      message_duration_timer.tic();
      send_command();
      process_reply();

      my_data.mean_margin = message_duration_timer.tac() * 1000;
      my_data.timestamp = dt_timer.tac() * 1000;
      lcm.publish("motor_data", &my_data);

    }
    std::cout<<"\nCTRL C Detected. Sending stop command and then segaulting" << std::endl;
    std::cout<<"TODO: Don't segfault" << std::endl;

    process_reply();
    robot->set_mode_stop();
    send_command();
    process_reply();
    robot->shutdown();
    return nullptr;
  }


 private:
  std::unique_ptr<Realtime_Robot> robot;
  const std::vector<int> servo_id_list = {10,11,12, 16, 17, 18, 19, 20, 21, 22};
  const std::vector<int> bus_map = {1,1,1, 3, 3, 3, 4, 4, 4, 4};

};

static void* Run(void* controller_void_ptr){
  SampleController* controller =
      (static_cast<SampleController*>(controller_void_ptr));

  controller->Run();
  return nullptr;
}

int main(int argc, char **argv) {

  real_time_tools::RealTimeThread thread;
  thread.parameters_.cpu_dma_latency_ = -1;
  thread.parameters_.priority_ = 97;

  enable_ctrl_c();
  SampleController sample_controller;

  thread.create_realtime_thread(Run, &sample_controller);
  thread.join();

  return 0;
}
