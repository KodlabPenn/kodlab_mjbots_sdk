//
// Created by shane on 11/10/21.
//

#pragma once
#include <vector>
#include <math.h>

/*!
 * @brief implements a basic soft start/torque ramp. For the duration, torque will ramp from 0 to max_torque linearly
 */
class Soft_Start {
 private:
  float m_slope = 0;        /// Slope of the ramp
  float m_max_torque = 100; /// Max torque allowed in Nm
  int m_duration = 0;       /// Duration of soft start in steps

 public:

  /*!
   * @brief constrains the vector torques based on soft start
   * @param torques[in, out] the torques to be constrained
   * @param count the number of steps it has been
   */
  void constrainTorques(std::vector<float>& torques, int count);

  /*!
   * @brief constrains all values in values between min_val and max_val
   * @param values[in, out] vector to be constrained
   * @param min_val minimum value allowed in vector
   * @param max_val maximum value allowed in vector
   */
  static void constrain(std::vector<float>& values, float min_val, float max_val);

  /*!
   * @brief constrains a single value between min_val and max_val
   * @param values[in]
   * @param min_val minimum value allowed
   * @param max_val maximum value allowed
   * @return constrained value
   */
  static float constrain(float values, float min_val, float max_val);

  /*!
   * @brief constructor for soft start
   * @param max_torque maximum torque allowed in robot
   * @param duration how long for ramp to last in steps
   */
  Soft_Start(float max_torque, int duration);
};

