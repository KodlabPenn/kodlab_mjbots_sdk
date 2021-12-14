// BSD 3-Clause License
// Copyright (c) 2021 The Trustees of the University of Pennsylvania. All Rights Reserved
// Authors:
// Shane Rozen-Levy <srozen01@seas.upenn.edu>

/*
 * This example demonstrates how you can use the mjbots_control_loop and the cartesian leg to create a hopping robot
 */

#include "kodlab_mjbots_sdk/mjbots_control_loop.h"
#include "leg_log.hpp"
#include "leg_gain.hpp"
#include "many_motor_log.hpp"
#include "kodlab_mjbots_sdk/lcm_subscriber.h"
#include "kodlab_mjbots_sdk/cartesian_leg.h"

class Hopping : public MjbotsControlLoop<leg_log, leg_gain>{
  using MjbotsControlLoop::MjbotsControlLoop;

  // We use CalcTorques as the main control loop
  void CalcTorques() override{
    std::vector<float> torques = {0,0};

    // Run the FK to get leg state
    m_leg.FK(robot_->GetJointPositions(), z, x);
    m_leg.FkVel(robot_->GetJointPositions(), robot_->GetJointVelocities(), d_z, d_x);

    //Check hybrid modes and switch if need be
    if(m_mode != hybrid_mode::SOFT_START && z0-z > 0.002  && d_z < 0){
      m_mode = hybrid_mode::STANCE;
    } else if (m_mode != hybrid_mode::SOFT_START && z0-z < 0.001 && d_z > 0){
      m_mode = hybrid_mode::FLIGHT;
    }

    // Switch per mode and Run behavior
    switch (m_mode) {
      case hybrid_mode::SOFT_START:{
        // In soft Start, control to a joint space position
        double q1_goal = -0.6;
        double q2_goal = 1.2;
        double q_kp = 6;
        double q_kd = 0.2;

        torques[0] = (q_kp * (q1_goal - robot_->GetJointPositions()[0]) - q_kd * robot_->GetJointVelocities()[0]);
        torques[1] = (q_kp * (q2_goal - robot_->GetJointPositions()[1]) - q_kd * robot_->GetJointVelocities()[1]);

        // If reached target and slow enough, switch to limp space, and set leg length as rest length
        if (std::abs(q1_goal - robot_->GetJointPositions()[0]) < 0.05 &&
            std::abs(q2_goal - robot_->GetJointPositions()[1])< 0.05 &&
            std::abs(robot_->GetJointVelocities()[0]) < 0.08 &&
            std::abs(robot_->GetJointVelocities()[1]) < 0.08){
          m_mode = FLIGHT;
          z0 = z;
          std::cout<<"Starting Limb mode"<<std::endl;
        }
        break;
      }
      case hybrid_mode::FLIGHT:{
        // In flight we use pd loops to control in limb space
        f_z = k_stiff * (z0 - z) - b_stiff * d_z;
        // We Constrain f_x to prevent large jumps in x force
        f_x = SoftStart::Constrain(-kp * x - kd * d_x, -20, 20);

        // convert from limb space to joint space
        torques = m_leg.InverseDynamics(robot_->GetJointPositions(), f_z, f_x);
        break;
      }
      case hybrid_mode::STANCE:{
        // In stance we use an Avik style active damping controller with gravity ffwd term
        float av = sqrtf((z-z0) * (z-z0) * w_v * w_v + d_z * d_z);
        float F = kv * d_z/av + m * 9.81 * 1;

        // Confirm z direction force is positive
        f_z = fmax(k * (z0 - z) - b * d_z + F, 0.0);
        // Set x direction force to 0 since robot is constrained in the x direction
        f_x = 0;

        // Convert from limb space to join space
        torques = m_leg.InverseDynamics(robot_->GetJointPositions(), f_z, f_x);
        break;
      }
    }
    //ffwd term for gravity comp
    torques[0] = torques[0] + 1 * 9.81 * 0.15 * 0.56 * sinf(robot_->GetJointPositions()[0]);
    robot_->SetTorques(torques);
  }

  void PrepareLog()  override{
    for (int servo = 0; servo < 2; servo++) {
      log_data_.positions[servo] = robot_->GetJointPositions()[servo];
      log_data_.velocities[servo] = robot_->GetJointVelocities()[servo];
      log_data_.modes[servo] = static_cast<int>(robot_->GetJointModes()[servo]);
      log_data_.torque_cmd[servo] = robot_->GetJointTorqueCmd()[servo];
      log_data_.torque_measure[servo]= robot_->GetJointTorqueMeasured()[servo];
    }
    log_data_.limb_position[0] = z-z0;
    log_data_.limb_position[1] = x;
    log_data_.limb_vel[0] = d_z;
    log_data_.limb_vel[1] = d_x;
    log_data_.limb_wrench[0] = f_z;
    log_data_.limb_wrench[1] = f_x;
    log_data_.hybrid_mode = m_mode;
  }

  void ProcessInput()override{
    std::cout<<"Response received"<<std::endl;
    kv = lcm_sub_.data_.kv;
    k = lcm_sub_.data_.k;
    k_stiff=lcm_sub_.data_.k_stiff;
    b = lcm_sub_.data_.b;
    b_stiff = lcm_sub_.data_.b_stiff;
    kp = lcm_sub_.data_.kp;
    kd = lcm_sub_.data_.kd;
  }

  enum hybrid_mode
  {
    SOFT_START = 0,
    FLIGHT = 1,
    STANCE = 2
  };

  CartesianLeg m_leg = CartesianLeg(0.15, 0.15);
  float kv = 0; /// Active damping gain
  float k = 800; /// Stance spring stiffness
  float k_stiff = 1600; /// Flight spring stiffness
  float m = 1.2; /// Robot mass
  float b = 15; /// Damping in stance
  float b_stiff =15; /// Damping in flight
  float z0 = 0; /// Rest length of leg
  float kp = 800; /// flight spring stiffness in x direction
  float kd = 10; /// flight damping in x direction
  float z, x, d_z, d_x; /// Limb space state
  float f_z, f_x; /// Limp space effort
  float w_v = sqrtf(k/m); /// Natural frequency
  hybrid_mode m_mode = hybrid_mode::SOFT_START;
};

int main(int argc, char **argv) {
  ControlLoopOptions options;
  options.motor_list.emplace_back(1, 1, 1, 0.1949);
  options.motor_list.emplace_back(2, 1, -1, 0.0389);
  options.log_channel_name = "leg_data";
  options.input_channel_name = "leg_gains";
  options.soft_start_duration = 5000;
  options.max_torque = 12;
  Hopping control_loop(options);
  control_loop.Start();
  control_loop.Join();
  return 0;
}
