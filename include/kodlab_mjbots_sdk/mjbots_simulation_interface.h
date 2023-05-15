// BSD 3-Clause License
// Copyright (c) 2021 The Trustees of the University of Pennsylvania. All Rights Reserved
// Authors:
// Shane Rozen-Levy <srozen01@seas.upenn.edu>
#ifndef _mjbots_simulation_interface
#define _mjbots_simulation_interface 

#pragma once

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


namespace kodlab::mjbots {



/*!
 * @brief Object allowing interaction with the Mjbots Moteus motor controller
 *        hardware
 */
class MjbotsSimulationInterface : public RobotInterface {
 public:

  /*!
   * @brief constructs an mjbots_robot_interface to communicate with a collection of moeteusses
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
  void Init();

  /*!
   * @brief Checks to make sure the response is ready and then adds the response to the data members in the robot interface
   * should be called after send command.
   * WARNING this is a blocking function call
   */
  void ProcessReply();

  /*!
   * @brief initiates a cycle of communication with the pi3hat. Sends torques and requests responses.
   * WARNING this is a non blocking function call, to get the response use process reply.
   * WARNING you must call ProcessReply after calling SendCommand before sending the next command
   */
  void SendCommand();
  
  /*!
   * @brief sets the moteus message to be stop, Run this followed by send command to stop the motors
   */
  void SetModeStop();

  /*!
   * @brief Stops the robot by setting and sending stop commands
   */
  void Stop();

  /*!
   * @brief shuts down the can thread
   */
  void Shutdown();

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
  

  void SetModelPath(std::string path){xml_model_path=path;}
  void SetFrequency(int freq){control_frequency=freq;}
 private:
  int control_frequency;                                     /// Control frequency (Now used as simulation frequency as well)
  std::string xml_model_path;
  
  bool dry_run_;                                     ///< dry run active flag
  bool print_torques_;                               ///< print torques active flag
  bool send_pd_commands_;                            ///< Include pd gains and setpoints in the moteus packet
  std::shared_ptr<::kodlab::IMUData<float>> imu_data_;                           /// Robot IMU data
  /******************************************Implementation**************************************************************/
  // MuJoCo data structures
  mjModel* m = NULL;                  // MuJoCo model
  mjData* d = NULL;                   // MuJoCo data
  mjvCamera cam;                      // abstract camera
  mjvOption opt;                      // visualization options
  mjvScene scn;                       // abstract scene
  mjrContext con;                     // custom GPU context
  
  GLFWwindow* window;
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
  float_t ctrl_update_freq = 100;
  mjtNum last_update = 0.0;
  mjtNum ctrl;


  // keyboard callback
  void keyboard(GLFWwindow* window, int key, int scancode, int act, int mods)
  {
      // backspace: reset simulation
      if( act==GLFW_PRESS && key==GLFW_KEY_BACKSPACE )
      {
          mj_resetData(m, d);
          mj_forward(m, d);
      }
  }

  static void keyCallbackStatic(GLFWwindow* window,int key,int scancode,int action,int mods){
    MjbotsSimulationInterface* that = static_cast<MjbotsSimulationInterface*>(glfwGetWindowUserPointer(window));
    that->keyboard(window, key, scancode, action, mods);
  }


  // mouse button callback
  void mouse_button(GLFWwindow* window, int button, int act, int mods)
  {
      // update button state
      button_left =   (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT)==GLFW_PRESS);
      button_middle = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_MIDDLE)==GLFW_PRESS);
      button_right =  (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT)==GLFW_PRESS);

      // update mouse position
      glfwGetCursorPos(window, &lastx, &lasty);
  }

  static void mouseCallbackStatic(GLFWwindow* window, int button, int act, int mods){
    MjbotsSimulationInterface* that = static_cast<MjbotsSimulationInterface*>(glfwGetWindowUserPointer(window));
    that->mouse_button(window, button, act, mods);
  }

  // mouse move callback
  void mouse_move(GLFWwindow* window, double xpos, double ypos)
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
      glfwGetWindowSize(window, &width, &height);

      // get shift key state
      bool mod_shift = (glfwGetKey(window, GLFW_KEY_LEFT_SHIFT)==GLFW_PRESS ||
                        glfwGetKey(window, GLFW_KEY_RIGHT_SHIFT)==GLFW_PRESS);

      // determine action based on mouse button
      mjtMouse action;
      if( button_right )
          action = mod_shift ? mjMOUSE_MOVE_H : mjMOUSE_MOVE_V;
      else if( button_left )
          action = mod_shift ? mjMOUSE_ROTATE_H : mjMOUSE_ROTATE_V;
      else
          action = mjMOUSE_ZOOM;

      // move camera
      mjv_moveCamera(m, action, dx/height, dy/height, &scn, &cam);
  }
  
  static void mouseMoveCallbackStatic(GLFWwindow* window, double xpos, double ypos){
    MjbotsSimulationInterface* that = static_cast<MjbotsSimulationInterface*>(glfwGetWindowUserPointer(window));
    that->mouse_move(window, xpos, ypos);
  }

  // scroll callback
  void scroll(GLFWwindow* window, double xoffset, double yoffset)
  {
      // emulate vertical mouse motion = 5% of window height
      mjv_moveCamera(m, mjMOUSE_ZOOM, 0, -0.05*yoffset, &scn, &cam);
  }

  static void scrollCallbackStatic(GLFWwindow* window, double xoffset, double yoffset){
    MjbotsSimulationInterface* that = static_cast<MjbotsSimulationInterface*>(glfwGetWindowUserPointer(window));
    that->scroll(window, xoffset, yoffset);
  }
};
} // namespace kodlab::mjbots

#endif