// BSD 3-Clause License
// Copyright (c) 2021 The Trustees of the University of Pennsylvania. All Rights Reserved
// Authors:
// Shane Rozen-Levy <srozen01@seas.upenn.edu>


#include "kodlab_mjbots_sdk/interfaces.h"

#include <iostream>
#include <algorithm>
#include <mujoco/mjmodel.h>
#include <mujoco/mujoco.h>

#include "cstdio"
#include "cstdlib"
#include "cstring"
#include <GLFW/glfw3.h>
// Libraries for sleep
#include <chrono>
#include <thread>


#include "kodlab_mjbots_sdk/log.h"
#include "kodlab_mjbots_sdk/string.h"  // kodlab::string::ScalarVectorToString

namespace kodlab::mjbots {


MjbotsSimulationInterface::MjbotsSimulationInterface(std::vector<std::shared_ptr<JointMoteus>> joint_ptrs,
                                                 const RealtimeParams &realtime_params,
                                                 ::mjbots::pi3hat::Euler imu_mounting_deg,
                                                 int imu_rate_hz,
                                                 std::shared_ptr<::kodlab::IMUData<float>> imu_data_ptr,
                                                 std::optional<::mjbots::pi3hat::Euler> imu_world_offset_deg,
                                                 bool dry_run,
                                                 bool print_torques,
                                                 bool send_pd_commands)
    : imu_data_(imu_data_ptr ? imu_data_ptr : std::make_shared<::kodlab::IMUData<float>>()),
      dry_run_(dry_run),
      print_torques_(print_torques),
      send_pd_commands_(send_pd_commands),
      mj_animator_(1000)
{ 
  LOG_IF_WARN(dry_run_, "\nDRY RUN: NO TORQUES COMMANDED");
  joints = joint_ptrs;
  num_joints_ = joints.size();

  

  if(imu_world_offset_deg.has_value()){
    kodlab::rotations::EulerAngles<float> imu_world_offset = {
        static_cast<float>(M_PI / 180.0 * imu_world_offset_deg->roll),
        static_cast<float>(M_PI / 180.0 * imu_world_offset_deg->pitch),
        static_cast<float>(M_PI / 180.0 * imu_world_offset_deg->yaw)
        };
    imu_data_->set_world_offset(imu_world_offset.ToQuaternion());
  }

  // Initialize and send basic command
  // InitializeCommand();
}


void MjbotsSimulationInterface::Init() {
    // init Mujoco GUI, model, and data
    mj_animator_.Init(xml_model_path);
    mj_animator_.setFrameRate(60);
    mj_model_=mj_animator_.getModel();
    mj_data_=mj_animator_.getData();
    mutex_ptr_=mj_animator_.getMutex();
    // initialize robot's state
    // std::cout<<"Start initailizing robot's state"<<std::endl;
    for (int i=0; i<initial_pos_.size(); i++) mj_data_->qpos[i]=initial_pos_[i];
    for (int i=0; i<initial_vel_.size(); i++) mj_data_->qvel[i]=initial_vel_[i];

    // Update once to enable dynamics in MuJoCo
    SendCommand();
    ProcessReply();

    
    // start animation thread
    mj_animator_.Start();
    sleep(1);
}

void MjbotsSimulationInterface::ProcessReply() {
  mutex_ptr_->lock();
  mj_step(mj_model_, mj_data_);
  // Copy results to object so controller can use
  int servo=0;
  for (auto & joint : joints) {
    joint->UpdateState(mj_data_->qpos[servo], mj_data_->qvel[servo],
                        mj_data_->sensordata[servo]);
  }

  mutex_ptr_->unlock();
}

void MjbotsSimulationInterface::SendCommand() {
  mutex_ptr_->lock();
  cycle_count_++;
  for (int servo = 0; servo < num_joints_; servo++) {
    mj_data_->ctrl[servo] = (dry_run_ ? 0 : joints[servo]->get_servo_torque());
  }
  mutex_ptr_->unlock();
}

void MjbotsSimulationInterface::SetModeStop() {
  ;
}

void MjbotsSimulationInterface::Stop() {
  // Send a few stop commands
  // free visualization storage
    mj_animator_.Stop();
    mj_animator_.Join();
}

void MjbotsSimulationInterface::Shutdown() {
  
    // terminate GLFW (crashes with Linux NVidia drivers)
    #if defined(__APPLE__) || defined(_WIN32)
        glfwTerminate();
    #endif
}

const ::kodlab::IMUData<float>& MjbotsSimulationInterface::GetIMUData() {
  return *imu_data_;
}

const std::shared_ptr<::kodlab::IMUData<float>> MjbotsSimulationInterface::GetIMUDataSharedPtr() {
  return imu_data_;
}

} // namespace kodlab::mjbots
