#include "kodlab_mjbots_sdk/mjbots_behavior.h"
#include "many_motor_log.hpp"
#include "kodlab_mjbots_sdk/abstract_lcm_subscriber.h"

class Spin_Motor : public mjbots_behavior_logging<many_motor_log>{
  using mjbots_behavior_logging<many_motor_log>::mjbots_behavior_logging;
  void calc_torques() override{
    std::vector<float> torques(m_num_motors, 0);
    m_robot->set_torques(torques);
  }

  void prepare_log()  override{
    std::cout<<"\n Trying to setup log"<<std::endl;
    for (int servo = 0; servo < m_num_motors; servo++) {
      m_log_data.positions[servo] = m_robot->get_joint_positions()[servo];
      m_log_data.velocities[servo] = m_robot->get_joint_velocities()[servo];
      m_log_data.modes[servo] = static_cast<int>(m_robot->get_joint_modes()[servo]);
      m_log_data.torques[servo] = m_robot->get_joint_torque_cmd()[servo];
    }
  }
};

int main(int argc, char **argv) {
  Behavior_Options options;
  options.motor_list_.push_back(Motor(1,1));
  options.motor_list_.push_back(Motor(2,1));
  Spin_Motor behavior(options, "motor_data");
  behavior.start();
  behavior.join();
  return 0;
}
