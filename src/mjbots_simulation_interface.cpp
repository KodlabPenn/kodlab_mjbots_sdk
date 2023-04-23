// BSD 3-Clause License
// Copyright (c) 2021 The Trustees of the University of Pennsylvania. All Rights Reserved
// Authors:
// Shane Rozen-Levy <srozen01@seas.upenn.edu>


#include "kodlab_mjbots_sdk/mjbots_simulation_interface.h"

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
void MjbotsSimulationInterface::InitializeCommand() {
  for (const auto &joint : joints) {
    commands_.push_back({});
    commands_.back().id = joint->get_can_id(); //id
  }

  ::mjbots::moteus::PositionResolution res; // This is just for the command
  if(send_pd_commands_){
    res.position = ::mjbots::moteus::Resolution::kInt16;
    res.velocity = ::mjbots::moteus::Resolution::kInt16;
    res.feedforward_torque = ::mjbots::moteus::Resolution::kInt16;
    res.kp_scale = ::mjbots::moteus::Resolution::kInt16;
    res.kd_scale = ::mjbots::moteus::Resolution::kInt16;
    res.maximum_torque = ::mjbots::moteus::Resolution::kInt8;
  }else{
    res.position = ::mjbots::moteus::Resolution::kIgnore;
    res.velocity = ::mjbots::moteus::Resolution::kIgnore;
    res.feedforward_torque = ::mjbots::moteus::Resolution::kInt16;
    res.kp_scale = ::mjbots::moteus::Resolution::kIgnore;
    res.kd_scale = ::mjbots::moteus::Resolution::kIgnore;
    res.maximum_torque = ::mjbots::moteus::Resolution::kIgnore;
  }
  res.stop_position = ::mjbots::moteus::Resolution::kIgnore;
  res.watchdog_timeout = ::mjbots::moteus::Resolution::kIgnore;
  for (auto &cmd : commands_) {
    cmd.resolution = res;
    cmd.mode = ::mjbots::moteus::Mode::kStopped;
    if(send_pd_commands_){
      cmd.query.torque = ::mjbots::moteus::Resolution::kInt16;
    }
  }
}



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
      send_pd_commands_(send_pd_commands)
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
    // activate software
    mj_activate("~/.mujoco/mujoco-2.3.3/bin/mjkey.txt");

    // load and compile model
    char error[1000] = "Could not load binary model";
    m = mj_loadXML(xml_model_path.c_str(), 0, error, 1000);
    if( !m )
      mju_error_s("Load model error: %s", error);

    // set timestep
    m->opt.timestep=1.0/control_frequency;
    // make data
    d = mj_makeData(m);


    // init GLFW
    if( !glfwInit() )
        mju_error("Could not initialize GLFW");

    // create window, make OpenGL context current, request v-sync
    window = glfwCreateWindow(1244, 700, "Demo", NULL, NULL);
    glfwMakeContextCurrent(window);
    glfwSwapInterval(1);

    // initialize visualization data structures
    mjv_defaultCamera(&cam);
    mjv_defaultOption(&opt);
    mjv_defaultScene(&scn);
    mjr_defaultContext(&con);
    mjv_makeScene(m, &scn, 2000);                // space for 2000 objects
    mjr_makeContext(m, &con, mjFONTSCALE_150);   // model-specific context

    // install GLFW mouse and keyboard callbacks
    // glfwSetKeyCallback(window, [](GLFWwindow* window_, int key, int scancode, int action, int mode){ MjbotsSimulationInterface::keyboard(window_, key, scancode, action, mode) ;});
    glfwSetWindowUserPointer(window, this);
    glfwSetKeyCallback(window,keyCallbackStatic);
    glfwSetCursorPosCallback(window, mouseMoveCallbackStatic);
    glfwSetMouseButtonCallback(window, mouseCallbackStatic);
    glfwSetScrollCallback(window, scrollCallbackStatic);
    
    // initial position
    d->qpos[0] = 1.57;

    // run main loop, target real-time simulation and 60 fps rendering
    mjtNum timezero = d->time;
    double_t update_rate = 0.01;

    // making sure the first time step updates the ctrl previous_time
    last_update = timezero-1.0/ctrl_update_freq;

    SendCommand();
    ProcessReply();
    // Setup message for basic torque commands
    SendCommand();
    ProcessReply();

}

void MjbotsSimulationInterface::ProcessReply() {
  mj_step(m, d);
  // get framebuffer viewport
  mjrRect viewport = {0, 0, 0, 0};
  glfwGetFramebufferSize(window, &viewport.width, &viewport.height);

    // update scene and render
  mjv_updateScene(m, d, &opt, NULL, &cam, mjCAT_ALL, &scn);
  mjr_render(viewport, &scn, &con);

  // swap OpenGL buffers (blocking call due to v-sync)
  glfwSwapBuffers(window);

  // process pending GUI events, call GLFW callbacks
  glfwPollEvents();
  
  // Copy results to object so controller can use
  int servo=0;
  for (auto & joint : joints) {
    joint->UpdateState(d->qpos[servo], d->qvel[servo],
                        d->sensordata[servo]);
  }
}

void MjbotsSimulationInterface::SendCommand() {
  cycle_count_++;
  for (int servo = 0; servo < num_joints_; servo++) {
    d->ctrl[servo] = (dry_run_ ? 0 : joints[servo]->get_servo_torque());
  }
}

void MjbotsSimulationInterface::SetModeStop() {
  for (auto &cmd : commands_) {
    cmd.mode = ::mjbots::moteus::Mode::kStopped;
  }
}

void MjbotsSimulationInterface::Stop() {
  // Send a few stop commands
  // free visualization storage
    mjv_freeScene(&scn);
    mjr_freeContext(&con);

    

}

void MjbotsSimulationInterface::Shutdown() {
  // moteus_interface_->shutdown();
  // free MuJoCo model and data, deactivate
    mj_deleteData(d);
    mj_deleteModel(m);
    mj_deactivate();

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
