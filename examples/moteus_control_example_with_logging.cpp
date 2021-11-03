// Copyright 2020 Josh Pieper, jjp@pobox.com.
//
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

#include <chrono>
#include <iomanip>
#include <iostream>
#include <future>
#include <limits>
#include <map>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include "kodlab_mjbots_sdk/moteus_protocol.h"
#include "kodlab_mjbots_sdk/pi3hat_moteus_interface.h"
#include <lcm/lcm-cpp.hpp>
#include "motor_log.hpp"
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
  int secondary_bus = 2;
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
  SampleController(const Arguments& arguments) : arguments_(arguments) {
    if (arguments_.primary_id == arguments_.secondary_id) {
      throw std::runtime_error("The servos must have unique IDs");
    }
  }

  /// This is called before any control begins, and must return the
  /// set of servos that are used, along with which bus each is
  /// attached to.
  std::map<int, int> servo_bus_map() const {
    return {
        { arguments_.primary_id, arguments_.primary_bus },
        { arguments_.secondary_id, arguments_.secondary_bus },
    };
  }

  /// This is also called before any control begins.  @p commands will
  /// be pre-populated with an entry for each servo returned by
  /// 'servo_bus_map'.  It can be used to perform one-time
  /// initialization like setting the resolution of commands and
  /// queries.
  void Initialize(std::vector<MoteusInterface::ServoCommand>* commands) {
    moteus::PositionResolution res; // This is just for the command
    res.position = moteus::Resolution::kInt16;
    res.velocity = moteus::Resolution::kInt16;
    res.feedforward_torque = moteus::Resolution::kInt16;
    res.kp_scale = moteus::Resolution::kInt16;
    res.kd_scale = moteus::Resolution::kInt16;
    res.maximum_torque = moteus::Resolution::kIgnore;
    res.stop_position = moteus::Resolution::kIgnore;
    res.watchdog_timeout = moteus::Resolution::kInt8;
    for (auto& cmd : *commands) {
      cmd.resolution = res;
      cmd.position.watchdog_timeout = 0.3;
    }
  }

  moteus::QueryResult Get(const std::vector<MoteusInterface::ServoReply>& replies, int id) {
    for (const auto& item : replies) {
      if (item.id == id) { return item.result; }
    }
    return {};
  }

  /// This is run at each control cycle.  @p status is the most recent
  /// status of all servos (note that it is possible for a given
  /// servo's result to be omitted on some frames).
  ///
  /// @p output should hold the desired output.  It will be
  /// pre-populated with the result of the last command cycle, (or
  /// Initialize to begin with).
  void Run(const std::vector<MoteusInterface::ServoReply>& status,
           std::vector<MoteusInterface::ServoCommand>* output) {
    cycle_count_++;

    // This is where your control loop would go.

    if (cycle_count_ < 5) {
      for (auto& cmd : *output) {
        // We start everything with a stopped command to clear faults.
        cmd.mode = moteus::Mode::kStopped;
      }
    } else {
      // Then we make the secondary servo mirror the primary servo.
      const auto primary = Get(status, arguments_.primary_id);
      double primary_pos = primary.position;
      if (!std::isnan(primary_pos) && std::isnan(primary_initial_)) {
        primary_initial_ = primary_pos;
      }
      double secondary_pos = Get(status, arguments_.secondary_id).position;
      if (!std::isnan(secondary_pos) && std::isnan(secondary_initial_)) {
        secondary_initial_ = secondary_pos;
      }
      if (!std::isnan(primary_initial_) && !std::isnan(secondary_initial_)) {
        // We have everything we need to start commanding.
        auto& primary_out = output->at(0);  // We constructed this, so we know the order.
        primary_out.mode = moteus::Mode::kPosition;
//        secondary_out.position.position = secondary_initial_ + (primary_pos - primary_initial_);
//        secondary_out.position.velocity = primary.velocity;
        primary_out.position.feedforward_torque = 0.01;
        primary_out.position.kp_scale = 0.0;
        primary_out.position.kd_scale = 0.0;

        auto& secondary_out = output->at(1);  // We constructed this, so we know the order.
        secondary_out.mode = moteus::Mode::kPosition;
//        secondary_out.position.position = secondary_initial_ + (primary_pos - primary_initial_);
//        secondary_out.position.velocity = primary.velocity;
        secondary_out.position.feedforward_torque = 0.2;
        secondary_out.position.kp_scale = 0.0;
        secondary_out.position.kd_scale = 0.0;

      }
    }
  }

 private:
  const Arguments arguments_;
  uint64_t cycle_count_ = 0;
  double primary_initial_ = std::numeric_limits<double>::quiet_NaN();
  double secondary_initial_ = std::numeric_limits<double>::quiet_NaN();
};

template <typename Controller>
[[noreturn]] void Run(const Arguments& args, Controller* controller) {
  if (args.help) {
    DisplayUsage();
    return;
  }

  //Setup realtime for behavior thread
  moteus::ConfigureRealtime(args.main_cpu);

  // Setup moteus options and initialize moteus interface
  MoteusInterface::Options moteus_options;
  moteus_options.cpu = args.can_cpu;
  moteus_options.servo_bus_map = controller->servo_bus_map();
  MoteusInterface moteus_interface{moteus_options};

  const int idArray[2] = {args.primary_id,args.secondary_id};
  // Create and initialize parts of the command
  std::vector<MoteusInterface::ServoCommand> commands;
  for (const auto& pair : moteus_options.servo_bus_map) {
    commands.push_back({});
    commands.back().id = pair.first; //id
  }
  controller->Initialize(&commands);// resolution

  // Create replies object
  std::vector<MoteusInterface::ServoReply> replies{commands.size()};
  std::vector<MoteusInterface::ServoReply> saved_replies;

  // moteus data has pointers to replies and commands
  MoteusInterface::Data moteus_data;
  moteus_data.commands = { commands.data(), commands.size() };
  moteus_data.replies = { replies.data(), replies.size() };

  std::future<MoteusInterface::Output> can_result;


  double sleep_time = 0.0;

  lcm::LCM lcm;
  motor_log my_data{};

  // Prepolute stopped command
  for (auto& cmd : commands) {
    // We start everything with a stopped command to clear faults.
    cmd.mode = moteus::Mode::kStopped;
  }
  auto promise_out = std::make_shared<std::promise<MoteusInterface::Output>>();
  moteus_interface.Cycle(
      moteus_data,
      [promise_out](const MoteusInterface::Output& output) {
        // This is called from an arbitrary thread, so we just set
        // the promise value here.
        promise_out->set_value(output);
      });
  can_result = promise_out->get_future();

  const auto period =
      std::chrono::microseconds(static_cast<int64_t>(args.period_s * 1e6));
  const auto start = std::chrono::steady_clock::now();

  auto next_cycle = start + period;

  // We will run at a fixed cycle time.
  while (true) {
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
      can_result.wait();
      controller->Run(replies, &commands);

      // Copy reply for logging
      {
        const auto current_values = can_result.get();
        // We copy out the results we just got out.
        const auto rx_count = current_values.query_result_size;
        saved_replies.resize(rx_count);
        std::copy(replies.begin(), replies.begin() + rx_count,
                  saved_replies.begin());
      }

      auto promise = std::make_shared<std::promise<MoteusInterface::Output>>();
      // Then we can immediately ask them to be used again.
      moteus_interface.Cycle(
          moteus_data,
          [promise](const MoteusInterface::Output& output) {
            // This is called from an arbitrary thread, so we just set
            // the promise value here.
            promise->set_value(output);
          });
      can_result = promise->get_future();

      for(int motor =0; motor< 2; motor ++){
        const auto motor_reply = controller->Get(saved_replies, idArray[motor]);
        my_data.positions[motor]=motor_reply.position;
        my_data.velocities[motor]=motor_reply.velocity;
        my_data.modes[motor]=static_cast<int>(motor_reply.mode);
        my_data.torques[motor] = commands[motor].position.feedforward_torque;
      }
      my_data.mean_margin = sleep_time * 1000;
      std::chrono::duration<double, std::milli> elapsed = now-start;
      my_data.timestamp = elapsed.count();
      lcm.publish("EXAMPLE", &my_data);
    }
  }
}
}

int main(int argc, char** argv) {
  Arguments args({argv + 1, argv + argc});

  // Lock memory for the whole process.
  LockMemory();

  SampleController sample_controller{args};
  Run(args, &sample_controller);

  return 0;
}
