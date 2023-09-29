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
  lcm_ = std::make_shared<lcm::LCM>();
  publisher_ = LcmPublisher<MujocoData>(lcm_, "mujoco_data");

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
    mj_model_ = mj_animator_.getModel();
    mj_data_ = mj_animator_.getData();
    mutex_ptr_ = mj_animator_.getMutex();
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
  // Copy results to object so controller can use
  for (auto & joint : joints) {
    int joint_id = mj_name2id(mj_model_, mjOBJ_JOINT, joint->get_name().c_str());
    if (joint_id == -1){
      std::cout << "Name, '" << joint->get_name() << "' not found, killing..."
                << std::endl;
      mutex_ptr_->unlock();
      ActivateCtrlC(); 
      return;
    }
    int joint_pos_index = mj_model_->jnt_qposadr[joint_id];
    int joint_vel_index = mj_model_->jnt_dofadr[joint_id];
    // std::cout << joint->get_name() << " " << joint_id << " "<<joint_pos_index<<" " << std::endl;
    
    joint->UpdateState( mj_data_->qpos[joint_pos_index],
                        mj_data_->qvel[joint_vel_index],
                        0);//TODO DEBUG WARNING !!! Should be servo torque
  }

  int quat_id = mj_name2id(mj_model_, mjOBJ_SENSOR, "QUAT");
  int gyro_id = mj_name2id(mj_model_, mjOBJ_SENSOR, "GYRO");
  int accel_id = mj_name2id(mj_model_, mjOBJ_SENSOR, "ACCEL");

  Eigen::Quaternionf quat_meas = Eigen::Quaternionf::Identity();
  Eigen::Vector3f ang_rate_meas = Eigen::Vector3f::Zero();
  Eigen::Vector3f accel_meas = Eigen::Vector3f::Zero();

  if(quat_id==-1){
      std::cout << "QUAT sensor not found" << std::endl;
  }else{
    int quat_index = mj_model_->sensor_adr[quat_id];
    quat_meas = Eigen::Quaternionf(mj_data_->sensordata[quat_index],
                                   mj_data_->sensordata[quat_index + 1],
                                   mj_data_->sensordata[quat_index + 2],
                                   mj_data_->sensordata[quat_index + 3]);
  }

  if(gyro_id==-1){
      std::cout << "GYRO sensor not found" << std::endl;
  }else{
    int gyro_index = mj_model_->sensor_adr[gyro_id];
    ang_rate_meas = Eigen::Vector3f(mj_data_->sensordata[gyro_index],
                                    mj_data_->sensordata[gyro_index + 1],
                                    mj_data_->sensordata[gyro_index + 2]);
  }

  if(accel_id==-1){
      std::cout << "ACCEL sensor not found" << std::endl;
  }else{
    int accel_index = mj_model_->sensor_adr[accel_id];
    accel_meas = Eigen::Vector3f(mj_data_->sensordata[accel_index],
                                 mj_data_->sensordata[accel_index + 1],
                                 mj_data_->sensordata[accel_index + 2]);
  }

  imu_data_->Update(quat_meas,ang_rate_meas,accel_meas);
  mutex_ptr_->unlock();

  MujocoData mujoco_lcm_message;
  mujoco_lcm_message.time = mj_data_->time;
  mujoco_lcm_message.nq = mj_model_->nq;
  mujoco_lcm_message.nv = mj_model_->nv;
  mujoco_lcm_message.na = mj_model_->na;
  mujoco_lcm_message.nu = mj_model_->nu;
  mujoco_lcm_message.nsensor = mj_model_->nsensor;
  mujoco_lcm_message.ncon = mj_data_->ncon;

  for(int i=0;i<mujoco_lcm_message.nq;i++){
    mujoco_lcm_message.qpos.push_back(mj_data_->qpos[i]);
  }
  for(int i=0;i<mujoco_lcm_message.nv;i++){
    mujoco_lcm_message.qvel.push_back(mj_data_->qvel[i]);
  }
  for(int i=0;i<mujoco_lcm_message.na;i++){
    mujoco_lcm_message.act.push_back(mj_data_->act[i]);
  }
  for(int i=0;i<mujoco_lcm_message.nsensor;i++){
    mujoco_lcm_message.sensordata.push_back(mj_data_->sensordata[i]);
  }
  for(int i=0;i<mujoco_lcm_message.nu;i++){
    mujoco_lcm_message.ctrl.push_back(mj_data_->ctrl[i]);
  }
  publisher_.set_message_data(mujoco_lcm_message);
  publisher_.Publish();


}

void MjbotsSimulationInterface::SendCommand() {
  while (mj_animator_.getPause()&&!CtrlCDetected()){
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  mutex_ptr_->lock();
  cycle_count_++;
  for (auto & joint : joints) {
    int actuator_id = mj_name2id(mj_model_, mjOBJ_ACTUATOR, (joint->get_name()+"_act").c_str());
    if (actuator_id == -1){
      std::cout << "Actuator name, '" << (joint->get_name()+"_act").c_str() << "' not found, killing..."
                << std::endl;
      mutex_ptr_->unlock();
      ActivateCtrlC(); 
      return;
    }
    //TODO Actuator index returns -1 so maybe it doesnt work how we expect, worth
    // exploring if we every want to include non motor/multi dof joints.
    int actuator_index = mj_model_->actuator_actadr[actuator_id];
    // std::cout << joint->get_name()+"_act" << " " << actuator_id << " " << std::endl;

    mj_data_->ctrl[actuator_id] = (dry_run_ ? 0 : joint->get_servo_torque());
  }
  mj_step(mj_model_, mj_data_);
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
