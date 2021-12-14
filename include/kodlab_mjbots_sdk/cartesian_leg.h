// BSD 3-Clause License
// Copyright (c) 2021 The Trustees of the University of Pennsylvania. All Rights Reserved
// Authors:
// Shane Rozen-Levy <srozen01@seas.upenn.edu>

#pragma once

#include <vector>

namespace kodlab {
/*!
 * @brief Implementation of a cartesian leg. Assumes zero is the leg pointing straight down in the z direction.
 * Kinematics is such that its hip relative to toe, and assumes the motor axis of rotation is into the page (y axis)
 */
class CartesianLeg {
 public:

  /*!
   * @brief create the cartesian leg using link lengths
   * @param l1 length of the femur
   * @param l2 length of the shin
   */
  CartesianLeg(float l1, float l2);

  /*!
   * @brief @brief computes the location of the hip relative to the toe
   * @param angles the joint angles
   * @param z[out] the height of the hip
   * @param x[out] the fore-aft displacement of the hip
   */
  void FK(const std::vector<float> &angles, float &z, float &x) const;

  /*!
   * @brief compute the necessary torques to produce the desired forces on the robot
   * @param angles[in] the joint angles
   * @param z_effort the desired vertical force on the robot
   * @param x_effort the desired fore-aft force on the robot
   * @return a vector of torques to send to the joints
   */
  std::vector<float> InverseDynamics(const std::vector<float> &angles, float z_effort, float x_effort) const;

  /*!
   * @brief computes the velocity of the hip relative to the toe
   * @param angles the joint angles
   * @param d_angles the angular velocity
   * @param d_z[out] vertical velocity
   * @param d_x[out] fore-aft velocity
   */
  void FkVel(const std::vector<float> &angles, const std::vector<float> &d_angles, float &d_z, float &d_x) const;

 private:
  float l1_; /// Length of femur
  float l2_; /// Length of shin
};
}