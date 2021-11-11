
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

#include "real_time_tools/spinner.hpp"
#include "real_time_tools/thread.hpp"
#include "real_time_tools/timer.hpp"
#include "kodlab_mjbots_sdk/moteus_protocol.h"
#include "kodlab_mjbots_sdk/pi3hat_moteus_interface.h"
#include <lcm/lcm-cpp.hpp>
#include "motor_log.hpp"
#include "kodlab_mjbots_sdk/realtime_robot.h"
using namespace mjbots;

using MoteusInterface = moteus::Pi3HatMoteusInterface;

namespace {
struct Arguments {
  Arguments(const std::vector<std::string>& args) {
    for (size_t i = 0; i < args.size(); i++) {
      const auto& arg = args[i];
      if (arg == "-h" || arg == "--help") {
        help = true;
      } else if (arg == "--main-cpu") {
        main_cpu = std::stoull(args.at(++i));
      } else if (arg == "--can-cpu") {
        can_cpu = std::stoull(args.at(++i));
      } else if (arg == "--period-s") {
        period_s = std::stod(args.at(++i));
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
  int main_cpu = 1;
  int can_cpu = 2;
  double period_s = 0.001;
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
  std::cout << "  --period-s S         period to run control\n";
  std::cout << "  --primary-id ID      servo ID of primary, undriven servo\n";
  std::cout << "  --primary-bus BUS    bus of primary servo\n";
  std::cout << "  --secondary-id ID    servo ID of secondary, driven servo\n";
  std::cout << "  --secondary-bus BUS  bus of secondary servo\n";
}

void LockMemory() {
  // We lock all memory so that we don't end up having to page in
  // something later which can take time.
  {
    const int r = ::mlockall(MCL_CURRENT | MCL_FUTURE);
    if (r < 0) {
      throw std::runtime_error("Error locking memory");
    }
  }
}

std::pair<double, double> MinMaxVoltage(
    const std::vector<MoteusInterface::ServoReply>& r) {
  double rmin = std::numeric_limits<double>::infinity();
  double rmax = -std::numeric_limits<double>::infinity();

  for (const auto& i : r) {
    if (i.result.voltage > rmax) { rmax = i.result.voltage; }
    if (i.result.voltage < rmin) { rmin = i.result.voltage; }
  }

  return std::make_pair(rmin, rmax);
}

/// This holds the user-defined control logic.
class SampleController {
 public:
  SampleController(const Arguments& arguments) : arguments_(arguments){
    if (arguments_.primary_id == arguments_.secondary_id) {
      throw std::runtime_error("The servos must have unique IDs");
    }
    std::cout<<"Preparing to create robot object"<<std::endl;
    robot = std::make_unique<Realtime_Robot>(Realtime_Robot(2, {arguments.primary_id, arguments.secondary_id},
                                                            {arguments.primary_bus, arguments.secondary_bus}, arguments.can_cpu));
    std::cout<<"Created robot object"<<std::endl;
  }

  void Run() {
    robot->set_torques({0.15, 0.15});
  }

  void process_reply(){
    robot->process_reply();
  }

  void send_command(){
    robot->send_command();
  }

  void prepare_log(motor_log& my_data){
    for(int servo =0; servo< 2; servo ++){
      my_data.positions[servo]=robot->get_joint_positions()[servo];
      my_data.velocities[servo]=robot->get_joint_velocities()[servo];
      my_data.modes[servo]=static_cast<int>(robot->get_joint_modes()[servo]);
      my_data.torques[servo] = robot->get_joint_torque_cmd()[servo];
    }
  }

  void shutdown(){
    robot.reset();
  }
 private:
  const Arguments arguments_;
  std::unique_ptr<Realtime_Robot> robot;
};

template <typename Controller>
void Run(const Arguments& args, Controller* controller) {
  if (args.help) {
    DisplayUsage();
    return;
  }

  //Setup realtime for behavior thread
  moteus::ConfigureRealtime(args.main_cpu);

  double sleep_time = 0.0;

  lcm::LCM lcm;
  motor_log my_data{};

  const auto period =
      std::chrono::microseconds(static_cast<int64_t>(args.period_s * 1e6));
  const auto start = std::chrono::steady_clock::now();

  auto next_cycle = start + period;

  // We will run at a fixed cycle time.
  while (!CTRL_C_DETECTED) {
    {
      // Sleep the correct amount
      {
        const auto pre_sleep = std::chrono::steady_clock::now();
        std::this_thread::sleep_until(next_cycle);
        const auto post_sleep = std::chrono::steady_clock::now();
        std::chrono::duration<double> elapsed = post_sleep - pre_sleep;
        sleep_time = elapsed.count();
      }
      next_cycle += period;

      const auto now = std::chrono::steady_clock::now();

      // Make sure we have not slept too long
      int skip_count = 0;
      while (now > next_cycle) {
        skip_count++;
        next_cycle += period;
      }
      if (skip_count) {
        std::cout << "\nSkipped " << skip_count << " cycles\n";
      }


      //  ensure results are updated before running loop

      controller->process_reply();
      controller->Run();

      controller->prepare_log(my_data);

      controller->send_command();


      my_data.mean_margin = sleep_time * 1000;
      std::chrono::duration<double, std::milli> elapsed = now-start;
      my_data.timestamp = elapsed.count();
      lcm.publish("EXAMPLE", &my_data);
    }
  }
  controller->shutdown();
}
}

int main(int argc, char** argv) {
  Arguments args({argv + 1, argv + argc});

  // Lock memory for the whole process.
  LockMemory();
  //enable_ctrl_c();
  std::cout<<"Finished basic setup"<<std::endl;
  SampleController sample_controller{args};
  std::cout<<"Finished creating controller"<<std::endl;
  Run(args, &sample_controller);

  return 0;
}
