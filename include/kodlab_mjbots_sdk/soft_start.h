// BSD 3-Clause License
// Copyright (c) 2021 The Trustees of the University of Pennsylvania. All Rights Reserved
// Authors:
// Shane Rozen-Levy <srozen01@seas.upenn.edu>
// J. Diego Caporale <jdcap@seas.upenn.edu>


#pragma once
#include <vector>
#include <cmath>
#include "real_time_tools/timer.hpp"

namespace kodlab {
/*!
 * @brief implements a basic soft Start/torque ramp. For the duration, torque will ramp from 0 to max_torque linearly
 */
class SoftStart {
 private:
  float slope_ = 0;        /// Slope of the ramp
  float max_torque_ = 100; /// Max torque allowed in Nm
  float duration_ms_ = 0;   /// Duration of soft start in milliseconds
  static real_time_tools::Timer timer_;
  static bool timer_initialized_;
 public:

  /*!
   * @brief constrains the vector torques based on soft Start
   * @param torques[in, out] the torques to be constrained
   */
  void ConstrainTorques(std::vector<float> &torques) const;

  /*!
 * @brief constrains the torque based on soft start
 * @param torque[in, out] the torque to be constrained
 */
  void ConstrainTorque(float &torque) const;

  /*!
   * @brief constructor for soft start
   * @param max_torque maximum torque allowed in joint
   * @param duration_ms how long for ramp to last in ms
   */
  SoftStart(float max_torque, float duration_ms);

  /*!
   * @brief Starts the soft start timer
   */
  static void InitializeTimer();
};

namespace TorqueLimiter {
/*!
 * @brief constrains a single value between min_val and max_val
 * @param values[in]
 * @param min_val minimum value allowed
 * @param max_val maximum value allowed
 * @return constrained value
 */
float Constrain(float value, float min_val, float max_val);

/*!
 * @brief constrains all values in values between min_val and max_val
 * @param values[in, out] vector to be constrained
 * @param min_val minimum value allowed in vector
 * @param max_val maximum value allowed in vector
 */
void Constrain(std::vector<float> &values, float min_val, float max_val);

}
} // namespace kodlab
