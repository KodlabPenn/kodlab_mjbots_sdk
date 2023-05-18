// BSD 3-Clause License
// Copyright (c) 2021 The Trustees of the University of Pennsylvania. All Rights Reserved
// Authors:
// Shane Rozen-Levy <srozen01@seas.upenn.edu>

#pragma once


#ifndef INTERFACE_INCLUDED
  #error "do not include mjbots_simulation_interface directly, include interfaces.h"
#endif

#include <vector>
#include <map>
#include <future>
#include <memory>
#include <optional>
#include "kodlab_mjbots_sdk/moteus_protocol.h"
#include "kodlab_mjbots_sdk/joint_moteus.h"
#include "kodlab_mjbots_sdk/pi3hat_moteus_interface.h"
#include "kodlab_mjbots_sdk/soft_start.h"
#include "kodlab_mjbots_sdk/imu_data.h"
#include "kodlab_mjbots_sdk/robot_interface.h"
#include "kodlab_mjbots_sdk/abstract_realtime_object.h"

#include <iostream>
#include <mujoco/mjmodel.h>
#include <mujoco/mujoco.h>

#include "cstdio"
#include "cstdlib"
#include "cstring"
#include <GLFW/glfw3.h>
// Libraries for sleep
#include <chrono>
#include <thread>


namespace kodlab {
class MujocoAnimator : public AbstractRealtimeObject{
  public:
  
  MujocoAnimator(float_t ctrl_update_frequency=1000, double frame_rate=60)
  : AbstractRealtimeObject(99, 1)
   {
     ctrl_update_freq=ctrl_update_frequency;
     frame_rate_=frame_rate;
   }

  void Init(std::string xml_model_path){
    // activate software
    mj_activate("~/.mujoco/mujoco-2.3.3/bin/mjkey.txt");

    // load and compile model
    char error[1000] = "Could not load binary model";
    mj_model=mj_loadXML(xml_model_path.c_str(), 0, error, 1000);
    if( !mj_model )
      mju_error_s("Load model error: %s", error);

    // set timestep
    mj_model->opt.timestep=1.0/ctrl_update_freq;
    // make data
    mj_data=mj_makeData(mj_model);
  }

  void Update(){
    // get framebuffer viewport
    mjrRect viewport = {0, 0, 0, 0};
    glfwGetFramebufferSize(window_, &viewport.width, &viewport.height);
      // update scene and render
    mutex_ptr_->lock();
    mjv_updateScene(mj_model, mj_data, &opt, NULL, &cam, mjCAT_ALL, &scn);
    mutex_ptr_->unlock();
    mjr_render(viewport, &scn, &con);
    // swap OpenGL buffers (blocking call due to v-sync)
    glfwSwapBuffers(window_);
    // process pending GUI events, call GLFW callbacks
    glfwPollEvents();
  }

  void Run() override {
    EnableCtrlC();
    // init GLFW
    if( !glfwInit() )
        mju_error("Could not initialize GLFW");

    // create window, make OpenGL context current, request v-sync
    window_ = glfwCreateWindow(1244, 700, "MuJoCo Animation", NULL, NULL);
    glfwMakeContextCurrent(window_);
    glfwSwapInterval(1);

    // initialize visualization data structures
    mjv_defaultCamera(&cam);
    mjv_defaultOption(&opt);
    mjv_defaultScene(&scn);
    mjr_defaultContext(&con);
    mjv_makeScene(mj_model, &scn, 2000);                // space for 2000 objects
    mjr_makeContext(mj_model, &con, mjFONTSCALE_150);   // model-specific context

    // install GLFW mouse and keyboard callbacks
    glfwSetWindowUserPointer(window_, this);
    glfwSetKeyCallback(window_,keyCallbackStatic);
    glfwSetCursorPosCallback(window_, mouseMoveCallbackStatic);
    glfwSetMouseButtonCallback(window_, mouseCallbackStatic);
    glfwSetScrollCallback(window_, scrollCallbackStatic);
    // sleep(3);
    while (!CTRL_C_DETECTED) {
    // while (true) {
      Update();      //update scene
      std::this_thread::sleep_for(std::chrono::microseconds((int)(1e6/frame_rate_)));
      // sleep(1.0/ctrl_update_freq);
    }
  }
  void Stop(){
    mjv_freeScene(&scn);
    mjr_freeContext(&con);
    // free MuJoCo model and data, deactivate
    mj_deleteData(mj_data);
    mj_deleteModel(mj_model);
    mj_deactivate();
  }


  void setFrameRate(double frame_rate){frame_rate_=frame_rate;}
  mjModel* getModel(){return mj_model;}
  mjData* getData(){return mj_data;}
  std::mutex* getMutex(){return mutex_ptr_;}
  private:
  mjModel* mj_model; 
  mjData* mj_data; 

  mjvCamera cam;                      // abstract camera
  mjvOption opt;                      // visualization options
  mjvScene scn;                       // abstract scene
  mjrContext con;                     // custom GPU context
  
  GLFWwindow* window_;
  // mouse interaction
  bool button_left = false;
  bool button_middle = false;
  bool button_right =  false;
  double lastx = 0;
  double lasty = 0;

  // holders of one step history of time and position to calculate dertivatives
  mjtNum position_history = 0;
  mjtNum previous_time = 0;

  // controller related variables
  float_t ctrl_update_freq;
  mjtNum last_update = 0.0;
  mjtNum ctrl;

  std::mutex mutex_;
  std::mutex* mutex_ptr_=&mutex_ ;
  
  double frame_rate_;

  // keyboard callback
  void keyboard(GLFWwindow* window_, int key, int scancode, int act, int mods)
  {
      // backspace: reset simulation
      if( act==GLFW_PRESS && key==GLFW_KEY_BACKSPACE )
      {
          mj_resetData(mj_model, mj_data);
          mj_forward(mj_model, mj_data);
      }
  }

  static void keyCallbackStatic(GLFWwindow* window_,int key,int scancode,int action,int mods){
    MujocoAnimator* that = static_cast<MujocoAnimator*>(glfwGetWindowUserPointer(window_));
    that->keyboard(window_, key, scancode, action, mods);
  }


  // mouse button callback
  void mouse_button(GLFWwindow* window_, int button, int act, int mods)
  {
      // update button state
      button_left =   (glfwGetMouseButton(window_, GLFW_MOUSE_BUTTON_LEFT)==GLFW_PRESS);
      button_middle = (glfwGetMouseButton(window_, GLFW_MOUSE_BUTTON_MIDDLE)==GLFW_PRESS);
      button_right =  (glfwGetMouseButton(window_, GLFW_MOUSE_BUTTON_RIGHT)==GLFW_PRESS);

      // update mouse position
      glfwGetCursorPos(window_, &lastx, &lasty);
  }

  static void mouseCallbackStatic(GLFWwindow* window_, int button, int act, int mods){
    MujocoAnimator* that = static_cast<MujocoAnimator*>(glfwGetWindowUserPointer(window_));
    that->mouse_button(window_, button, act, mods);
  }

  // mouse move callback
  void mouse_move(GLFWwindow* window_, double xpos, double ypos)
  {
      // no buttons down: nothing to do
      if( !button_left && !button_middle && !button_right )
          return;

      // compute mouse displacement, save
      double dx = xpos - lastx;
      double dy = ypos - lasty;
      lastx = xpos;
      lasty = ypos;

      // get current window size
      int width, height;
      glfwGetWindowSize(window_, &width, &height);

      // get shift key state
      bool mod_shift = (glfwGetKey(window_, GLFW_KEY_LEFT_SHIFT)==GLFW_PRESS ||
                        glfwGetKey(window_, GLFW_KEY_RIGHT_SHIFT)==GLFW_PRESS);

      // determine action based on mouse button
      mjtMouse action;
      if( button_right )
          action = mod_shift ? mjMOUSE_MOVE_H : mjMOUSE_MOVE_V;
      else if( button_left )
          action = mod_shift ? mjMOUSE_ROTATE_H : mjMOUSE_ROTATE_V;
      else
          action = mjMOUSE_ZOOM;

      // move camera
      mjv_moveCamera(mj_model, action, dx/height, dy/height, &scn, &cam);
  }
  
  static void mouseMoveCallbackStatic(GLFWwindow* window_, double xpos, double ypos){
    MujocoAnimator* that = static_cast<MujocoAnimator*>(glfwGetWindowUserPointer(window_));
    that->mouse_move(window_, xpos, ypos);
  }

  // scroll callback
  void scroll(GLFWwindow* window_, double xoffset, double yoffset)
  {
      // emulate vertical mouse motion = 5% of window height
      mjv_moveCamera(mj_model, mjMOUSE_ZOOM, 0, -0.05*yoffset, &scn, &cam);
  }

  static void scrollCallbackStatic(GLFWwindow* window_, double xoffset, double yoffset){
    MujocoAnimator* that = static_cast<MujocoAnimator*>(glfwGetWindowUserPointer(window_));
    that->scroll(window_, xoffset, yoffset);
  }

};
}

namespace kodlab::mjbots {



/*!
 * @brief Object allowing interaction with the Mjbots Moteus motor controller
 *        hardware
 */
class MjbotsSimulationInterface : public kodlab::RobotInterface {
 public:

  /*!
   * @brief constructs an simulation_robot_interface to communicate with MUJOCO simulation
   * @param joint_list a vector of shared pointers to joints defining the motors in the robot
   * @param realtime_params the realtime parameters defining cpu and realtime priority
   * @param imu_mounting_deg Orientation of the imu on the pi3hat. Assumes gravity points in the +z direction
   * @param imu_rate_hz Frequency of the imu updates from the pi3hat
   * @param imu_data_ptr Shared pointer to imu_data to use or nullptr if it should make its own
   * @param imu_world_offset_deg [Optional] IMU orientation offset. Useful for re-orienting gravity, etc.
   * @param dry_run if true, sends zero-torques to Moteus controllers
   * @param print_torques if true, prints torque commands
   * @param send_pd_commands if true, packets to the moteus include pd gains and setpoints
   */
  MjbotsSimulationInterface(std::vector<std::shared_ptr<JointMoteus>> joint_list,
                       const RealtimeParams &realtime_params,
                       ::mjbots::pi3hat::Euler imu_mounting_deg = ::mjbots::pi3hat::Euler(),
                       int imu_rate_hz = 1000,
                       std::shared_ptr<::kodlab::IMUData<float>> imu_data_ptr = nullptr,
                       std::optional<::mjbots::pi3hat::Euler > imu_world_offset_deg = std::nullopt,
                       bool dry_run = false,
                       bool print_torques = false,
                       bool send_pd_commands = false
                       );

  /**
   * @brief Send and recieve initial communications effectively starting the robot
   * 
   */
  void Init() override;

  /*!
   * @brief Checks to make sure the response is ready and then adds the response to the data members in the robot interface
   * should be called after send command.
   * WARNING this is a blocking function call
   */
  void ProcessReply() override;

  /*!
   * @brief initiates a cycle of communication with the pi3hat. Sends torques and requests responses.
   * WARNING this is a non blocking function call, to get the response use process reply.
   * WARNING you must call ProcessReply after calling SendCommand before sending the next command
   */
  void SendCommand() override;
  
  /*!
   * @brief sets the moteus message to be stop, Run this followed by send command to stop the motors
   */
  void SetModeStop() override;

  /*!
   * @brief Stops the robot by setting and sending stop commands
   */
  void Stop() override;

  /*!
   * @brief shuts down the can thread
   */
  void Shutdown() override;

  /*!
   * @brief accessor for the joint modes
   * @return the joint modes
   */
  
  /*!
   * @brief accessor for the IMU data of the robot
   * @return const reference to the IMU data object for the robot
   */
  const ::kodlab::IMUData<float>& GetIMUData();

  /*!
   * @brief accessor for the IMU data of the robot
   * @return const IMU data shared pointer for the robot
   */
  const std::shared_ptr<::kodlab::IMUData<float>> GetIMUDataSharedPtr();        
  

  void SetModelPath(std::string path) override {xml_model_path=path;}
  void SetFrequency(int freq) override {control_frequency=freq;}
  void SetInitialState(std::vector<double> initial_pos,std::vector<double> initial_vel) override {
    for (double p:initial_pos) initial_pos_.push_back(p);
    for (double v:initial_vel) initial_vel_.push_back(v);
  }
 private:
  int control_frequency;                                     /// Control frequency (Now used as simulation frequency as well)
  std::string xml_model_path;
  
  bool dry_run_;                                     ///< dry run active flag
  bool print_torques_;                               ///< print torques active flag
  bool send_pd_commands_;                            ///< Include pd gains and setpoints in the moteus packet
  std::shared_ptr<::kodlab::IMUData<float>> imu_data_;                           /// Robot IMU data
  std::vector<double> initial_pos_;
  std::vector<double> initial_vel_;

  MujocoAnimator mj_animator_;
  // MuJoCo data structures
  mjModel* mj_model_ = NULL;                  // MuJoCo model
  mjData* mj_data_ = NULL;                   // MuJoCo data
  

  std::mutex* mutex_ptr_ = nullptr;
};
} // namespace kodlab::mjbots
