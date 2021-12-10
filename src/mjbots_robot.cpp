//
// Created by shane on 11/3/21.
//

#include "kodlab_mjbots_sdk/mjbots_robot.h"

#include <utility>
#include "iostream"

void Mjbots_Robot::initialize_command() {
  for (const auto& id : m_servo_id_list) {
    m_commands.push_back({});
    m_commands.back().id = id; //id
  }

  mjbots::moteus::PositionResolution res; // This is just for the command
  res.position = mjbots::moteus::Resolution::kInt16; //Can be 0
  res.velocity = mjbots::moteus::Resolution::kInt16; //Can be 0
  res.feedforward_torque = mjbots::moteus::Resolution::kInt16;
  res.kp_scale = mjbots::moteus::Resolution::kInt8; //Can be 0 iff kp is set to 0 on motors
  res.kd_scale = mjbots::moteus::Resolution::kInt8; //Can be 0 iff kd is set to 0 on motors during config
  res.maximum_torque = mjbots::moteus::Resolution::kIgnore;
  res.stop_position = mjbots::moteus::Resolution::kIgnore;
  res.watchdog_timeout = mjbots::moteus::Resolution::kIgnore;
  for (auto& cmd : m_commands) {
    cmd.resolution = res;
    cmd.mode = mjbots::moteus::Mode::kStopped;
    std::cout<<cmd.id<<std::endl;
  }
}

void Mjbots_Robot::prepare_torque_command() {
  for (auto& cmd : m_commands) {
    cmd.mode = mjbots::moteus::Mode::kPosition;
    cmd.position.kd_scale = 0;
    cmd.position.kp_scale = 0;
  }
}


mjbots::moteus::QueryResult Mjbots_Robot::Get(const std::vector<mjbots::moteus::Pi3HatMoteusInterface::ServoReply> &replies,
                                              int id) {
  for (const auto& item : replies) {
    if (item.id == id) { return item.result; }
  }
  return {};
}

Mjbots_Robot::Mjbots_Robot(const std::vector<Motor>& motor_list, const Realtime_Params& realtime_params, float max_torque,
                           int soft_start_duration):
                               m_soft_start(max_torque, soft_start_duration) {
  // Process motor list
  m_num_servos = motor_list.size();
  for (const auto & motor_elem : motor_list){
    m_servo_id_list.push_back(motor_elem.id);
    m_servo_bus_list.push_back(motor_elem.can_bus);
    m_offsets.push_back(motor_elem.offset);
    m_directions.push_back(motor_elem.direction);
  }
  for (size_t i = 0; i < m_num_servos; ++i)
    m_servo_bus_map[m_servo_id_list[i]] = m_servo_bus_list[i];

  // Create moteus interface
  mjbots::moteus::Pi3HatMoteusInterface::Options moteus_options;
  moteus_options.cpu = realtime_params.can_cpu;
  moteus_options.realtime_priority = realtime_params.can_rtp;
  moteus_options.servo_bus_map = m_servo_bus_map;
  m_moteus_interface = std::make_unique<mjbots::moteus::Pi3HatMoteusInterface>(moteus_options);

  // Initialize and send basic command
  initialize_command();
  m_replies = std::vector<mjbots::moteus::Pi3HatMoteusInterface::ServoReply>{m_commands.size()};
  m_moteus_data.commands = { m_commands.data(), m_commands.size() };
  m_moteus_data.replies = { m_replies.data(), m_replies.size() };
  send_command();

  // Prepare variables for saving response and process reply
  for(int servo = 0; servo< m_num_servos; servo++){
    m_positions.push_back(0);
    m_velocities.push_back(0);
    m_torque_cmd.push_back(0);
    m_torque_measured.push_back(0);
    m_modes.push_back(mjbots::moteus::Mode::kStopped);
  }
  process_reply();

  // Setup message for basic torque commands
  prepare_torque_command();
}

void Mjbots_Robot::process_reply() {

  // Make sure the m_can_result is valid before waiting otherwise undefined behavior
  if (m_can_result.valid()){
    m_can_result.wait();
  }

  for(int servo =0; servo< m_num_servos; servo ++){
    const auto servo_reply = Get(m_replies, m_servo_id_list[servo]);

    m_positions[servo]=m_directions[servo] * (servo_reply.position * 2 * M_PI )+ m_offsets[servo];
    m_velocities[servo]= m_directions[servo] * ( servo_reply.velocity * 2 * M_PI);
    m_torque_measured[servo]= m_directions[servo] * (servo_reply.torque);
    m_modes[servo]=servo_reply.mode;
  }
  m_timeout = m_moteus_data.timeout;
  if(m_timeout){
    std::cout<<"Error, pi3 hat timeout"<<std::endl;
  }
}

void Mjbots_Robot::send_command() {
  m_cycle_count ++;
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

void Mjbots_Robot::set_torques(std::vector<float> torques) {
  m_soft_start.constrainTorques(torques, m_cycle_count);
  m_torque_cmd = torques;

  for(int servo =0; servo< m_num_servos; servo ++){
    m_commands[servo].position.feedforward_torque = m_directions[servo] * (torques[servo]);
  }
}

std::vector<float> Mjbots_Robot::get_joint_positions() {
  return m_positions;
}

std::vector<float> Mjbots_Robot::get_joint_velocities() {
  return m_velocities;
}

std::vector<mjbots::moteus::Mode> Mjbots_Robot::get_joint_modes() {
  return  m_modes;
}

std::vector<float> Mjbots_Robot::get_joint_torque_cmd() {
  return m_torque_cmd;
}
std::vector<float> Mjbots_Robot::get_joint_torque_measured() {
  return m_torque_measured;
}
void Mjbots_Robot::set_mode_stop() {
  for (auto& cmd : m_commands) {
    cmd.mode = mjbots::moteus::Mode::kStopped;
  }
}

void Mjbots_Robot::shutdown() {
  m_moteus_interface->shutdown();
}
