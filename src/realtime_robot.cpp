//
// Created by shane on 11/3/21.
//

#include "kodlab_mjbots_sdk/realtime_robot.h"

#include <utility>
#include "iostream"

void Realtime_Robot::initialize_command() {
  for (const auto& pair : m_servo_bus_map) {
    m_commands.push_back({});
    m_commands.back().id = pair.first; //id
  }

  mjbots::moteus::PositionResolution res; // This is just for the command
  res.position = mjbots::moteus::Resolution::kInt16;
  res.velocity = mjbots::moteus::Resolution::kInt16;
  res.feedforward_torque = mjbots::moteus::Resolution::kInt16;
  res.kp_scale = mjbots::moteus::Resolution::kInt8;
  res.kd_scale = mjbots::moteus::Resolution::kInt8;
  res.maximum_torque = mjbots::moteus::Resolution::kIgnore;
  res.stop_position = mjbots::moteus::Resolution::kIgnore;
  res.watchdog_timeout = mjbots::moteus::Resolution::kInt8;
  for (auto& cmd : m_commands) {
    cmd.resolution = res;
    cmd.position.watchdog_timeout = 0.1;
    cmd.mode = mjbots::moteus::Mode::kStopped;
  }
}

void Realtime_Robot::prepare_torque_command() {
  for (auto& cmd : m_commands) {
    cmd.mode = mjbots::moteus::Mode::kPosition;
    cmd.position.kd_scale = 0;
    cmd.position.kp_scale = 0;
  }

}


mjbots::moteus::QueryResult Realtime_Robot::Get(const std::vector<mjbots::moteus::Pi3HatMoteusInterface::ServoReply> &replies,
                                                int id) {
  for (const auto& item : replies) {
    if (item.id == id) { return item.result; }
  }
  return {};
}
Realtime_Robot::Realtime_Robot(int num_servos,
                               std::vector<int> servo_id_list,
                               std::vector<int> servo_bus_list,
                               int can_cpu){

  m_num_servos = num_servos;
  m_servo_id_list = servo_id_list;
  m_servo_bus_list = servo_bus_list;

  for (size_t i = 0; i < m_num_servos; ++i)
    m_servo_bus_map[m_servo_id_list[i]] = m_servo_bus_list[i];

  mjbots::moteus::Pi3HatMoteusInterface::Options moteus_options;
  moteus_options.cpu = can_cpu;
  moteus_options.servo_bus_map = m_servo_bus_map;

  m_moteus_interface = std::make_unique<mjbots::moteus::Pi3HatMoteusInterface>(moteus_options);

  m_moteus_interface->start();
  initialize_command();
  m_replies = std::vector<mjbots::moteus::Pi3HatMoteusInterface::ServoReply>{m_commands.size()};
  m_moteus_data.commands = { m_commands.data(), m_commands.size() };
  m_moteus_data.replies = { m_replies.data(), m_replies.size() };

  std::cout<<"Trying to send command" << std::endl;
  send_command();
  std::cout<<"command sent" << std::endl;

  for(int servo = 0; servo< m_num_servos; servo++){
    m_positions.push_back(0);
    m_velocities.push_back(0);
    m_torques.push_back(0);
    m_modes.push_back(mjbots::moteus::Mode::kStopped);
  }

  process_reply();

  prepare_torque_command();


}
void Realtime_Robot::process_reply() {

  // Make sure the m_can_result is valid before waiting otherwise undefined behavior
  if (m_can_result.valid()){
    m_can_result.wait();
  }

  for(int servo =0; servo< m_num_servos; servo ++){
    const auto servo_reply = Get(m_replies, m_servo_id_list[servo]);

    m_positions[servo]=servo_reply.position;
    m_velocities[servo]=servo_reply.velocity;
    m_modes[servo]=servo_reply.mode;
  }
}

void Realtime_Robot::send_command() {
  auto promise = std::make_shared<std::promise<mjbots::moteus::Pi3HatMoteusInterface::Output>>();
  m_moteus_interface->Cycle(
      m_moteus_data,
      [promise](const mjbots::moteus::Pi3HatMoteusInterface::Output& output) {
        // This is called from an arbitrary thread, so we just set
        // the promise value here.
        promise->set_value(output);
      });
  m_can_result = promise->get_future();
}

void Realtime_Robot::set_torques(std::vector<float> torques) {
  for(int servo =0; servo< m_num_servos; servo ++){
    m_commands[servo].position.feedforward_torque = torques[servo];
  }
}

std::vector<float> Realtime_Robot::get_joint_positions() {
  return m_positions;
}

std::vector<float> Realtime_Robot::get_joint_velocities() {
  return m_velocities;
}

std::vector<mjbots::moteus::Mode> Realtime_Robot::get_joint_modes() {
  return  m_modes;
}

std::vector<float> Realtime_Robot::get_joint_torques() {
  return m_torques;
}
