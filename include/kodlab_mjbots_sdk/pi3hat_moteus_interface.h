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

// Modified by Shane Rozen-Levy srozen01@seas.upenn.edu to use realtime library

#pragma once

#include <condition_variable>
#include <functional>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <thread>
#include <vector>
#include <iostream>
#include "real_time_tools/thread.hpp"
#include <real_time_tools/process_manager.hpp>
#include "kodlab_mjbots_sdk/pi3hat.h"
#include <sys/types.h>
#include <unistd.h>
#include "kodlab_mjbots_sdk/common_header.h"
#include "kodlab_mjbots_sdk/abstract_realtime_object.h"
namespace mjbots {
namespace moteus {

/// This class represents the interface to the moteus controllers.
/// Internally it uses a background thread to operate the pi3hat,
/// enabling the main thread to perform work while servo communication
/// is taking place.
class Pi3HatMoteusInterface : public kodlab::AbstractRealtimeObject{
 public:
  struct Options {
    int cpu = -1;
    int realtime_priority = -1;

    // If a servo is not present, it is assumed to be on bus 1.
    std::map<int, int> servo_bus_map;
  };

  Pi3HatMoteusInterface(const Options& options)
      : options_(options){
    realtime_priority_ = options.realtime_priority;
    cpu_ = options.cpu;
    Start();
  }

  ~Pi3HatMoteusInterface() {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      done_ = true;
      condition_.notify_one();
    }
    Join();
  }

  void shutdown() {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      done_ = true;
      condition_.notify_one();
    }
    Join();
  }

  struct ServoCommand {
    int id = 0;

    moteus::Mode mode = moteus::Mode::kStopped;

    // For mode = kPosition or kZeroVelocity
    moteus::PositionCommand position;
    moteus::PositionResolution resolution;

    moteus::QueryCommand query;
  };

  struct ServoReply {
    int id = 0;
    moteus::QueryResult result;
  };

  // This describes what you would like to do in a given control cycle
  // in terms of sending commands or querying data.
  struct Data {
    pi3hat::Span<ServoCommand> commands;

    pi3hat::Span<ServoReply> replies;

    std::shared_ptr<bool> timeout;
    std::shared_ptr<float> send_duration;
    std::shared_ptr<float> reply_duration;
    std::shared_ptr<float> cycle_duration;
    std::shared_ptr<float> child_cycle_duration;
  };

  struct Output {
    size_t query_result_size = 0;
  };

  using CallbackFunction = std::function<void (const Output&)>;

  /// When called, this will schedule a cycle of communication with
  /// the servos.  The callback will be invoked from an arbitrary
  /// thread when the communication cycle has completed.
  ///
  /// All memory pointed to by @p data must remain valid until the
  /// callback is invoked.
  void Cycle(const Data& data, CallbackFunction callback) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (active_) {
      throw std::logic_error(
          "Cycle cannot be called until the previous has completed");
    }

    callback_ = std::move(callback);
    active_ = true;
    data_ = data;
    condition_.notify_all();
  }

 private:

  void Run( ) override{
    pi3hat::Pi3Hat::Configuration config;
    pi3hat_ = std::make_shared<pi3hat::Pi3Hat>(config);
    while (true) {
      {
        std::unique_lock<std::mutex> lock(mutex_);
        if (!active_) {
          condition_.wait(lock);
          if (done_) { return;}

          if (!active_) { continue; }
        }
      }

      auto output = CHILD_Cycle();
      CallbackFunction callback_copy;
      {
        std::unique_lock<std::mutex> lock(mutex_);
        active_ = false;
        std::swap(callback_copy, callback_);
      }
      callback_copy(output);
    }
  }

  Output CHILD_Cycle() {
    child_cycle_timer.tic();
    tx_can_.resize(data_.commands.size());
    int out_idx = 0;
    for (const auto& cmd : data_.commands) {
      const auto& query = cmd.query;

      auto& can = tx_can_[out_idx++];

      can.expect_reply = query.any_set();
      can.id = cmd.id | (can.expect_reply ? 0x8000 : 0x0000);
      can.size = 0;

      can.bus = [&]() {
        const auto it = options_.servo_bus_map.find(cmd.id);
        if (it == options_.servo_bus_map.end()) {
          return 1;
        }
        return it->second;
      }();
      moteus::WriteCanFrame write_frame(can.data, &can.size);
      switch (cmd.mode) {
        case Mode::kStopped: {
          moteus::EmitStopCommand(&write_frame);
          break;
        }
        case Mode::kPosition:
        case Mode::kZeroVelocity: {
          moteus::EmitPositionCommand(&write_frame, cmd.position, cmd.resolution);
          break;
        }
        default: {
          throw std::logic_error("unsupported mode");
        }
      }
      moteus::EmitQueryCommand(&write_frame, cmd.query);
    }

    rx_can_.resize(data_.commands.size() * 2);

    pi3hat::Pi3Hat::Input input;
    input.tx_can = { tx_can_.data(), tx_can_.size() };
    input.rx_can = { rx_can_.data(), rx_can_.size() };

    Output result;
    const auto output = pi3hat_->Cycle(input);
    *data_.timeout = output.timeout;
    *data_.reply_duration = output.reply_duration;
    *data_.send_duration = output.send_duration;
    *data_.cycle_duration = output.cycle_duration;
    for (size_t i = 0; i < output.rx_can_size && i < data_.replies.size(); i++) {
      const auto& can = rx_can_[i];

      data_.replies[i].id = (can.id & 0x7f00) >> 8;
      data_.replies[i].result = moteus::ParseQueryResult(can.data, can.size);
      result.query_result_size = i + 1;
    }
    if (output.timeout) {
     std::cout << "Error, pi3 hat timeout" << std::endl;
    }
    *data_.child_cycle_duration = child_cycle_timer.tac() * 1000;
    return result;
  }

  const Options options_;


  /// This block of variables are all controlled by the mutex.
  std::mutex mutex_;
  std::condition_variable condition_;
  bool active_ = false;;
  bool done_ = false;
  CallbackFunction callback_;
  Data data_;


  /// All further variables are only used from within the child thread.

  std::shared_ptr<pi3hat::Pi3Hat> pi3hat_ = nullptr;

  // These are kept persistently so that no memory allocation is
  // required in steady state.
  std::vector<pi3hat::CanFrame> tx_can_;
  std::vector<pi3hat::CanFrame> rx_can_;

  real_time_tools::Timer child_cycle_timer;
  int timeout = 0;
};


}
}
