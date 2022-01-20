//
// Created by shane on 1/20/22.
//

#pragma once

#include <memory>
#include <kodlab_mjbots_sdk/mjbots_robot_interface.h>
#include "jerboa_lib/physical_params.h"
#include "jerboa_lib/detect_mode_transition.h"

class TVH{
 public:
  void UpdateState(const std::shared_ptr<kodlab::mjbots::MjbotsRobotInterface>& robot, float t);

  void SetEffort(const std::shared_ptr<kodlab::mjbots::MjbotsRobotInterface>& robot, const float tail_torque);

  /*!
 * @brief accessor for tail angle
 * @return the tail angle
 */
  float GetTailAngle()const;

  /*!
   * @brief accessor for tail angular velocity
   * @return tail angular velocity
   */
  float GetTailSpeed()const;

  /*!
   * @brief accessor for leg compression
   * @return the leg compression
   */
  float GetLegCompression()const;

  /*!
   * @brief accessor for leg length
   * @return the leg length
   */
  float GetLegLength()const;

  /*!
   * @brief accessor for derivative of leg length
   * @return leg length velocity
   */
  float GetLegSpeed()const;

  HybridMode GetMode();

  void Disable();


  PhysicalParams params_;
 private:
  /*!
   * @brief computes the leg kinematics to get length length, leg comp, and leg vel
   * @param LE leg encoder value
   * @param dLE derivative of leg encoder value
   */
  void legKinematics(float LE, float dLE);

  ModeObserver mode_observer_;
  float tail_angle_;
  float leg_encoder_;
  float leg_length_;
  float leg_comp_;
  float tail_speed_;
  float leg_encoder_speed_;
  float leg_speed_;

};
