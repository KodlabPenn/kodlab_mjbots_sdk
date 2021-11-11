//
// Created by shane on 11/3/21.
//

#pragma once

#include <vector>
#include <map>
#include <future>
#include "kodlab_mjbots_sdk/moteus_protocol.h"
#include "kodlab_mjbots_sdk/pi3hat_moteus_interface.h"

class Realtime_Robot {
 private:
  int m_num_servos;
  std::vector<int> m_servo_id_list;
  std::vector<int> m_servo_bus_list;
  std::map<int, int> m_servo_bus_map;
  std::vector<float> m_positions;
  std::vector<float> m_velocities;
  std::vector<float> m_torque_cmd;
  std::vector<float> m_torque_measured;
  std::vector<mjbots::moteus::Mode> m_modes;

  std::vector<mjbots::moteus::Pi3HatMoteusInterface::ServoCommand> m_commands;
  std::vector<mjbots::moteus::Pi3HatMoteusInterface::ServoReply> m_replies;
  std::unique_ptr<mjbots::moteus::Pi3HatMoteusInterface> m_moteus_interface;
  mjbots::moteus::Pi3HatMoteusInterface::Data m_moteus_data;
  std::future<mjbots::moteus::Pi3HatMoteusInterface::Output> m_can_result;

  void initialize_command();

  void prepare_torque_command();

  mjbots::moteus::QueryResult Get(const std::vector<mjbots::moteus::Pi3HatMoteusInterface::ServoReply>& replies, int id);


 public:
  Realtime_Robot(int num_servos,
                 std::vector<int> servo_id_list,
                 std::vector<int> servo_bus_list,
                 int can_cpu);

  void process_reply();

  void send_command();

  void set_torques(std::vector<float> torques);

  void set_mode_stop();

  void shutdown();

  std::vector<float> get_joint_positions();

  std::vector<float> get_joint_velocities();

  std::vector<float> get_joint_torque_cmd();

  std::vector<float> get_joint_torque_measured();

  std::vector<mjbots::moteus::Mode> get_joint_modes();

};

