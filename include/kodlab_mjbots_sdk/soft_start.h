// BSD 3-Clause License
// Copyright (c) 2021 The Trustees of the University of Pennsylvania. All Rights Reserved
// Authors:
// Shane Rozen-Levy <srozen01@seas.upenn.edu>


#pragma once
#include <vector>
#include <math.h>

/*!
 * @brief implements a basic soft Start/torque ramp. For the duration, torque will ramp from 0 to max_torque linearly
 */
class SoftStart {
 private:
  float slope_ = 0;        /// Slope of the ramp
  float max_torque_ = 100; /// Max torque allowed in Nm
  int duration_ = 0;       /// Duration of soft Start in steps

 public:

  /*!
   * @brief constrains the vector torques based on soft Start
   * @param torques[in, out] the torques to be constrained
   * @param count the number of steps it has been
   */
  void ConstrainTorques(std::vector<float>& torques, int count);

  /*!
   * @brief constrains all values in values between min_val and max_val
   * @param values[in, out] vector to be constrained
   * @param min_val minimum value allowed in vector
   * @param max_val maximum value allowed in vector
   */
  static void Constrain(std::vector<float>& values, float min_val, float max_val);

  /*!
   * @brief constrains a single value between min_val and max_val
   * @param values[in]
   * @param min_val minimum value allowed
   * @param max_val maximum value allowed
   * @return constrained value
   */
  static float Constrain(float values, float min_val, float max_val);

  /*!
   * @brief constructor for soft Start
   * @param max_torque maximum torque allowed in robot
   * @param duration how long for ramp to last in steps
   */
  SoftStart(float max_torque, int duration);
};

