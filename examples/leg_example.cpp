
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
#include <math.h>
#include "real_time_tools/spinner.hpp"
#include "real_time_tools/hard_spinner.hpp"

#include "real_time_tools/thread.hpp"
#include "real_time_tools/timer.hpp"
#include <lcm/lcm-cpp.hpp>
#include <real_time_tools/realtime_check.hpp>
#include <real_time_tools/process_manager.hpp>
#include "leg_log.hpp"
#include "kodlab_mjbots_sdk/realtime_robot.h"
#include "kodlab_mjbots_sdk/polar_leg.h"

using namespace mjbots;

using MoteusInterface = moteus::Pi3HatMoteusInterface;

struct Arguments {
  Arguments(const std::vector<std::string> &args) {
    for (size_t i = 0; i < args.size(); i++) {
      const auto &arg = args[i];
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
  int main_cpu = 2;
  int can_cpu = 3;
  double period_s = 0.001;
  int primary_id = 1;
  int primary_bus = 1;
  int secondary_id = 2;
  int secondary_bus = 1;
};


/// This holds the user-defined control logic.
class SampleController {
 public:
  SampleController(const Arguments& arguments): arguments_(arguments) {
    if (arguments_.primary_id == arguments_.secondary_id) {
      throw std::runtime_error("The servos must have unique IDs");
    }
    robot = std::make_unique<Realtime_Robot>(Realtime_Robot(2,
                                                            {arguments_.primary_id, arguments_.secondary_id},
                                                            {arguments_.primary_bus, arguments_.secondary_bus},
                                                            arguments_.can_cpu,
                                                            {0.1949, 0.0389},
                                                            {1, -1},
                                                            3,
                                                            10000));
  }

  void calc_torques() {
    if(start){
      double q1_goal = -0.4;
      double q2_goal = 0.8;
      double q_kp = 6;
      double q_kd = 0.2;

      std::vector<float> torques = {0,0};
      torques[0] = (q_kp * (q1_goal -robot->get_joint_positions()[0]) - q_kd * robot->get_joint_velocities()[0]);
      torques[1] = (q_kp * (q2_goal -robot->get_joint_positions()[1]) - q_kd * robot->get_joint_velocities()[1]);
      torques[0] = torques[0] + 1 * 9.81 * 0.15 * 0.56 * sinf(robot->get_joint_positions()[0]);

      robot->set_torques(torques);
      if (std::abs(q1_goal -robot->get_joint_positions()[0]) < 0.05 &&
          std::abs(q2_goal -robot->get_joint_positions()[1])< 0.05 &&
          std::abs(robot->get_joint_velocities()[0]) < 0.08 &&
          std::abs(robot->get_joint_velocities()[1]) < 0.08){
        start = false;
        m_leg.fk(robot->get_joint_positions(), r, theta);
        r0 = r;
        std::cout<<"Starting Limb mode"<<std::endl;
      }
    } else{
      m_leg.fk(robot->get_joint_positions(), r, theta);
      m_leg.fk_vel(robot-> get_joint_positions(), robot->get_joint_velocities(), d_r, d_theta);

      f_r = k * (r0 - r) - b * d_r;
      f_theta = - kp * theta - kd * d_theta;

      auto torques = m_leg.inverse_dynamics(robot->get_joint_positions(), f_r, f_theta);

      //ffwd term for gravity comp
      torques[0] = torques[0] + 1 * 9.81 * 0.15 * 0.56 * sinf(robot->get_joint_positions()[0]);
      robot->set_torques(torques);
    }


  }

  void process_reply() {
    robot->process_reply();
  }

  void send_command() {
    robot->send_command();
  }

  void prepare_log(leg_log &my_data) {
    for (int servo = 0; servo < 2; servo++) {
      my_data.positions[servo] = robot->get_joint_positions()[servo];
      my_data.velocities[servo] = robot->get_joint_velocities()[servo];
      my_data.modes[servo] = static_cast<int>(robot->get_joint_modes()[servo]);
      my_data.torque_cmd[servo] = robot->get_joint_torque_cmd()[servo];
      my_data.torque_measure[servo]=robot->get_joint_torque_measured()[servo];
    }
    my_data.polar_position[0] = r;
    my_data.polar_position[1] = theta;
    my_data.polar_vel[0] = d_r;
    my_data.polar_vel[1] = d_theta;
    my_data.polar_wrench[0] = f_r;
    my_data.polar_wrench[1] = f_theta;
  }

  void *Run() {
    int cycle_count = 0;
    std::vector<int> cpu = {arguments_.main_cpu};
    real_time_tools::fix_current_process_to_cpu(cpu, ::getpid());

    lcm::LCM lcm;
    leg_log my_data{};

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

        prepare_log(my_data);

        send_command();

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
  Polar_Leg m_leg = Polar_Leg(0.15, 0.15);
  float k = 400;
  float b = 2;
  float r0 = 0;
  float kp = 10;
  float kd = 0.1;
  float r, theta, d_r, d_theta;
  float f_r, f_theta;
  bool start = true;
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
