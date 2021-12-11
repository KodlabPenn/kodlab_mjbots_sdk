/*
 * This example demonstrates how you can use the mjbots_control_loop and the cartesian leg to create a hopping robot
 */

#include "kodlab_mjbots_sdk/mjbots_control_loop.h"
#include "leg_log.hpp"
#include "kodlab_mjbots_sdk/abstract_lcm_subscriber.h"
#include "kodlab_mjbots_sdk/cartesian_leg.h"

class Hopping : public Mjbots_Control_Loop<leg_log>{
  using Mjbots_Control_Loop<leg_log>::Mjbots_Control_Loop;

  // We use calc_torques as the main control loop
  void calc_torques() override{
    std::vector<float> torques = {0,0};

    // Run the fk to get leg state
    m_leg.fk(m_robot->get_joint_positions(), z, x);
    m_leg.fk_vel(m_robot-> get_joint_positions(), m_robot->get_joint_velocities(), d_z, d_x);

    //Check hybrid modes and switch if need be
    if(m_mode != hybrid_mode::SOFT_START && z0-z > 0.002  && d_z < 0){
      m_mode = hybrid_mode::STANCE;
    } else if (m_mode != hybrid_mode::SOFT_START && z0-z < 0.001 && d_z > 0){
      m_mode = hybrid_mode::FLIGHT;
    }

    // Switch per mode and run behavior
    switch (m_mode) {
      case hybrid_mode::SOFT_START:{
        // In soft start, control to a joint space position
        double q1_goal = -0.6;
        double q2_goal = 1.2;
        double q_kp = 6;
        double q_kd = 0.2;

        torques[0] = (q_kp * (q1_goal -m_robot->get_joint_positions()[0]) - q_kd * m_robot->get_joint_velocities()[0]);
        torques[1] = (q_kp * (q2_goal -m_robot->get_joint_positions()[1]) - q_kd * m_robot->get_joint_velocities()[1]);

        // If reached target and slow enough, switch to limp space, and set leg length as rest length
        if (std::abs(q1_goal -m_robot->get_joint_positions()[0]) < 0.05 &&
            std::abs(q2_goal -m_robot->get_joint_positions()[1])< 0.05 &&
            std::abs(m_robot->get_joint_velocities()[0]) < 0.08 &&
            std::abs(m_robot->get_joint_velocities()[1]) < 0.08){
          m_mode = FLIGHT;
          z0 = z;
          std::cout<<"Starting Limb mode"<<std::endl;
        }
        break;
      }
      case hybrid_mode::FLIGHT:{
        // In flight we use pd loops to control in limb space
        f_z = k_stiff * (z0 - z) - b_stiff * d_z;
        // We constrain f_x to prevent large jumps in x force
        f_x = Soft_Start::constrain(- kp * x - kd * d_x, -20, 20);

        // convert from limb space to joint space
        torques = m_leg.inverse_dynamics(m_robot->get_joint_positions(), f_z, f_x);
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
        torques = m_leg.inverse_dynamics(m_robot->get_joint_positions(), f_z, f_x);
        break;
      }
    }
    //ffwd term for gravity comp
    torques[0] = torques[0] + 1 * 9.81 * 0.15 * 0.56 * sinf(m_robot->get_joint_positions()[0]);
    m_robot->set_torques(torques);
  }

  void prepare_log()  override{
    for (int servo = 0; servo < 2; servo++) {
      m_log_data.positions[servo] = m_robot->get_joint_positions()[servo];
      m_log_data.velocities[servo] = m_robot->get_joint_velocities()[servo];
      m_log_data.modes[servo] = static_cast<int>(m_robot->get_joint_modes()[servo]);
      m_log_data.torque_cmd[servo] = m_robot->get_joint_torque_cmd()[servo];
      m_log_data.torque_measure[servo]=m_robot->get_joint_torque_measured()[servo];
    }
    m_log_data.limb_position[0] = z-z0;
    m_log_data.limb_position[1] = x;
    m_log_data.limb_vel[0] = d_z;
    m_log_data.limb_vel[1] = d_x;
    m_log_data.limb_wrench[0] = f_z;
    m_log_data.limb_wrench[1] = f_x;
    m_log_data.hybrid_mode = m_mode;
  }

  enum hybrid_mode
  {
    SOFT_START = 0,
    FLIGHT = 1,
    STANCE = 2
  };

  Cartesian_Leg m_leg = Cartesian_Leg(0.15, 0.15);
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
  enable_ctrl_c();
  Control_Loop_Options options;
  options.m_motor_list.emplace_back(1, 1, 1, 0.1949);
  options.m_motor_list.emplace_back(2, 1, -1, 0.0389);
  options.m_channel_name = "leg_data";
  options.m_soft_start_duration = 5000;
  options.m_max_torque = 12;
  Hopping control_loop(options);
  control_loop.start();
  control_loop.join();
  return 0;
}
