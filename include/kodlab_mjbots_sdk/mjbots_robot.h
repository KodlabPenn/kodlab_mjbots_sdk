//
// Created by shane on 11/3/21.
//

#pragma once

#include <vector>
#include <map>
#include <future>
#include "kodlab_mjbots_sdk/moteus_protocol.h"
#include "kodlab_mjbots_sdk/pi3hat_moteus_interface.h"
#include "kodlab_mjbots_sdk/soft_start.h"

struct Motor{
  Motor(int id_, int can_bus_): can_bus(can_bus_), id(id_){}
  Motor(int id_, int can_bus_, int direction_, float offset_):
        can_bus(can_bus_), id(id_), direction(direction_), offset(offset_){}
  int can_bus;
  int id;
  int direction = 1;
  float offset  = 0;
};

struct Realtime_Params{
  int can_cpu = 3;
  int main_cpu = 2;
  int main_rtp = 97;
  int can_rtp = 98;
};


class Mjbots_Robot {
 private:
  int m_num_servos;
  std::vector<int> m_servo_id_list;
  std::vector<int> m_servo_bus_list;
  std::map<int, int> m_servo_bus_map;
  std::vector<float> m_positions;
  std::vector<float> m_velocities;
  std::vector<float> m_torque_cmd;
  std::vector<float> m_torque_measured;
  std::vector<float> m_offsets;
  std::vector<int> m_directions;
  bool m_timeout = false;
  std::vector<mjbots::moteus::Mode> m_modes;
  long int m_cycle_count = 0;

  std::vector<mjbots::moteus::Pi3HatMoteusInterface::ServoCommand> m_commands;
  std::vector<mjbots::moteus::Pi3HatMoteusInterface::ServoReply> m_replies;
  std::unique_ptr<mjbots::moteus::Pi3HatMoteusInterface> m_moteus_interface;
  mjbots::moteus::Pi3HatMoteusInterface::Data m_moteus_data;
  std::future<mjbots::moteus::Pi3HatMoteusInterface::Output> m_can_result;
  Soft_Start m_soft_start;

  void initialize_command();

  void prepare_torque_command();

  static mjbots::moteus::QueryResult Get(const std::vector<mjbots::moteus::Pi3HatMoteusInterface::ServoReply>& replies, int id);


 public:
  Mjbots_Robot(const std::vector<Motor>& motor_list,
               const Realtime_Params& realtime_params,
               float max_torque = 20,
               int soft_start_duration = 1);

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

