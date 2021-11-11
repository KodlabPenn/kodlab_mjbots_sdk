
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

#include <iostream>
#include <future>
#include <limits>
#include <map>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>
#include <sys/types.h>
#include <unistd.h>
#include "real_time_tools/spinner.hpp"
#include "real_time_tools/hard_spinner.hpp"

#include "real_time_tools/thread.hpp"
#include "real_time_tools/timer.hpp"
#include <lcm/lcm-cpp.hpp>
#include <real_time_tools/realtime_check.hpp>
#include <real_time_tools/process_manager.hpp>
#include "motor_log.hpp"
#include "kodlab_mjbots_sdk/realtime_robot.h"
using namespace mjbots;

using MoteusInterface = moteus::Pi3HatMoteusInterface;

struct Arguments {
  Arguments(const std::vector<std::string> &args) {
    for (size_t i = 0; i < args.size(); i++) {
      const auto &arg = args[i];
      if (arg == "-h" || arg == "--help") {
        help = true;
      } else if (arg == "--primary-id") {
        primary_id = std::stoull(args.at(++i));
      } else if (arg == "--primary-bus") {
        primary_bus = std::stoull(args.at(++i));
      } else if (arg == "--secondary-id") {
        secondary_id = std::stoull(args.at(++i));
      } else if (arg == "--secondary-bus") {
        secondary_bus = std::stoull(args.at(++i));
      } else {
        throw std::runtime_error("Unknown argument: " + arg);
      }
    }
  }

  bool help = false;
  int main_cpu = 2;
  int can_cpu = 1;
  int primary_id = 1;
  int primary_bus = 1;
  int secondary_id = 2;
  int secondary_bus = 1;
};

void DisplayUsage() {
  std::cout << "Usage: moteus_control_example [options]\n";
  std::cout << "\n";
  std::cout << "  -h, --help           display this usage message\n";
  std::cout << "  --main-cpu CPU       run main thread on a fixed CPU [default: 1]\n";
  std::cout << "  --can-cpu CPU        run CAN thread on a fixed CPU [default: 2]\n";
  std::cout << "  --primary-id ID      servo ID of primary, undriven servo\n";
  std::cout << "  --primary-bus BUS    bus of primary servo\n";
  std::cout << "  --secondary-id ID    servo ID of secondary, driven servo\n";
  std::cout << "  --secondary-bus BUS  bus of secondary servo\n";
}


/// This holds the user-defined control logic.
class SampleController {
 public:
  SampleController(const Arguments& arguments): arguments_(arguments) {
    if (arguments_.primary_id == arguments_.secondary_id) {
      throw std::runtime_error("The servos must have unique IDs");
    }
    robot.reset(new Realtime_Robot(2,
                                   {arguments_.primary_id, arguments_.secondary_id},
                                   {arguments_.primary_bus, arguments_.secondary_bus},
                                   arguments_.can_cpu));
  }

  void calc_torques() {
    robot->set_torques({0.0, 0.0});
  }

  void process_reply() {
    robot->process_reply();
  }

  void send_command() {
    robot->send_command();
  }

  void prepare_log(motor_log &my_data) {
    for (int servo = 0; servo < 2; servo++) {
      my_data.positions[servo] = robot->get_joint_positions()[servo];
      my_data.velocities[servo] = robot->get_joint_velocities()[servo];
      my_data.modes[servo] = static_cast<int>(robot->get_joint_modes()[servo]);
      my_data.torques[servo] = robot->get_joint_torque_cmd()[servo];
    }
  }

  void *Run() {
    int cycle_count = 0;
    std::vector<int> cpu = {arguments_.main_cpu};
    real_time_tools::fix_current_process_to_cpu(cpu, ::getpid());

    lcm::LCM lcm;
    motor_log my_data{};

    real_time_tools::HardSpinner spinner;
    spinner.set_frequency(1000);
    real_time_tools::Timer dt_timer;
    dt_timer.tic();

    // We will run at a fixed cycle time.
    while (!CTRL_C_DETECTED) {
      {
        cycle_count ++;

        double sleep_duration = spinner.predict_sleeping_time();
        spinner.spin();

        process_reply();
        calc_torques();


        send_command();

        prepare_log(my_data);

        my_data.mean_margin = sleep_duration * 1000;
        my_data.timestamp = dt_timer.tac() * 1000;
        lcm.publish("EXAMPLE", &my_data);
      }
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
  const Arguments arguments_;
  std::unique_ptr<Realtime_Robot> robot;
};

static void* Run(void* controller_void_ptr){
  SampleController* controller =
      (static_cast<SampleController*>(controller_void_ptr));

  controller->Run();
  return nullptr;
}

int main(int argc, char **argv) {
  Arguments args({argv + 1, argv + argc});

  enable_ctrl_c();
  SampleController sample_controller{args};

  real_time_tools::RealTimeThread thread;
  thread.parameters_.cpu_dma_latency_ = -1;
  thread.parameters_.priority_ = 97;
  thread.create_realtime_thread(Run, &sample_controller);
  thread.join();

  return 0;
}
