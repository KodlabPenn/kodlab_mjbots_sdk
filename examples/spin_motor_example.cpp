// BSD 3-Clause License
// Copyright (c) 2021 The Trustees of the University of Pennsylvania. All Rights Reserved
// Authors:
// Shane Rozen-Levy <srozen01@seas.upenn.edu>

/* Basic example script demonstrating how to use the mjbots_control_loop to 2 motors. The functions to implement are
 * calc_torques and prepare_log. In this example we send a torque cmd of all zeros and log the motor information.
 */

#include "kodlab_mjbots_sdk/mjbots_control_loop.h"
#include "many_motor_log.hpp"
#include "kodlab_mjbots_sdk/abstract_lcm_subscriber.h"

class Spin_Motor : public Mjbots_Control_Loop<many_motor_log>{
  using Mjbots_Control_Loop<many_motor_log>::Mjbots_Control_Loop;
  void calc_torques() override{
    std::vector<float> torques(m_num_motors, 0);
    m_robot->set_torques(torques);
  }

  void prepare_log()  override{
    for (int servo = 0; servo < m_num_motors; servo++) {
      m_log_data.positions[servo] = m_robot->get_joint_positions()[servo];
      m_log_data.velocities[servo] = m_robot->get_joint_velocities()[servo];
      m_log_data.modes[servo] = static_cast<int>(m_robot->get_joint_modes()[servo]);
      m_log_data.torques[servo] = m_robot->get_joint_torque_cmd()[servo];
    }
  }
};

int main(int argc, char **argv) {
  Control_Loop_Options options;
  // Define the motors in the robot
  options.m_motor_list.emplace_back(1, 1);
  options.m_motor_list.emplace_back(2, 1);
  options.m_log_channel_name = "motor_data";

  // Create control loop
  Spin_Motor control_loop(options);
  // Starts the loop, and then join it
  control_loop.start();
  control_loop.join();
  return 0;
}
